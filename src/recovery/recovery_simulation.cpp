#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {
constexpr double kGravity = 9.80665;
constexpr double kSeaLevelDensity = 1.225;
constexpr double kScaleHeightM = 8500.0;

bool finiteVector(const Vec3& value) {
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

double maxAbs(const Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

} // namespace

RecoverySimulation::RecoverySimulation(const RecoveryConfiguration& config) {
    reset(config);
}

void RecoverySimulation::reset(const RecoveryConfiguration& supplied) {
    config_ = supplied;
    if (config_.initialState.positionEcefM.norm() < kRecoveryEarthRadiusM * 0.5) {
        const double playback = config_.playbackSpeed;
        const bool maxPlayback = config_.maxPlayback;
        config_ = makeDefaultRecoveryConfiguration();
        config_.playbackSpeed = playback;
        config_.maxPlayback = maxPlayback;
    }
    config_.durationSec = clamp(config_.durationSec, 30.0, 1800.0);
    config_.guidanceFrequencyHz = clamp(config_.guidanceFrequencyHz, 1.0, 50.0);
    config_.landingQpHorizon = std::clamp(config_.landingQpHorizon, 4, 80);
    config_.landingQpDtSec = clamp(config_.landingQpDtSec, 0.05, 2.0);
    config_.vehicle.minThrottle = clamp(config_.vehicle.minThrottle, 0.0, 1.0);
    config_.vehicle.maxThrottle = clamp(config_.vehicle.maxThrottle,
                                        config_.vehicle.minThrottle, 1.0);
    config_.vehicle.maxThrustN = std::max(1.0, config_.vehicle.maxThrustN);
    config_.vehicle.dryMassKg = std::max(1.0, config_.vehicle.dryMassKg);

    frame_ = makeLandingNedFrame(config_.landingSite);
    truth_.time = config_.initialState.time;
    truth_.positionEcefM = config_.initialState.positionEcefM;
    truth_.velocityEcefMps = config_.initialState.velocityEcefMps;
    truth_.bodyToEcef = config_.initialState.bodyToEcef.normalized();
    truth_.angularRateBodyRadPerSec = config_.initialState.angularRateBodyRadPerSec;
    truth_.massKg = std::max(config_.vehicle.dryMassKg, config_.initialState.massTotalKg);
    truth_.propellantKg = clamp(config_.initialState.propellantMassKg, 0.0,
                                truth_.massKg - config_.vehicle.dryMassKg);
    truth_.massKg = config_.vehicle.dryMassKg + truth_.propellantKg;
    initialPropellantKg_ = truth_.propellantKg;
    rotation_ = {truth_.bodyToEcef, truth_.angularRateBodyRadPerSec};
    mission_.reset();
    allocator_.reset();
    engine_.reset();
    landingGuidance_.reset();
    attitudeController_.configure(config_.attitudeControl, ControlVehicle::LaunchVehicle,
        config_.vehicle.inertiaKgM2,
        {config_.vehicle.maxRcsTorqueNm, config_.vehicle.maxRcsTorqueNm,
         config_.vehicle.maxRcsTorqueNm});
    flipStartAttitude_ = truth_.bodyToEcef;
    const Vec3 flipDirectionNed{-1.0, 0.0, -0.45};
    flipTargetAttitude_ = attitudeFromBodyX(nedVectorToEcef(flipDirectionNed, frame_),
                                            frame_.eastEcef);
    guidance_ = {};
    guidance_.desiredAttitude = flipStartAttitude_;
    prediction_ = predictor_.predict(truth_, frame_, config_);
    lastSolverStatus_ = RecoverySolverStatus::NotRun;
    lastSolverTimeMs_ = 0.0;
    lastSolverFallback_ = false;
    nextGuidanceTimeSec_ = truth_.time;
    nextSampleTimeSec_ = truth_.time;
    lastThrustN_ = 0.0;
    lastDynamicPressurePa_ = 0.0;
    lastBrakingDistanceM_ = 0.0;
    lastControlTorqueNm_ = {};
    lastActuator_ = {};
    solverCalls_ = 0;
    solverSuccesses_ = 0;
    totalSolverTimeMs_ = 0.0;
    maxSolverTimeMs_ = 0.0;
    touchdownPositionNed_ = {};
    touchdownVelocityNed_ = {};
    touchdownCaptured_ = false;
    history_.clear();
    recordSample(true);
}

bool RecoverySimulation::complete() const {
    return mission_.phase() == RecoveryPhase::Touchdown
        || mission_.phase() == RecoveryPhase::Failed
        || truth_.time >= config_.durationSec;
}

double RecoverySimulation::brakingDistance(const Vec3& velocityNed, double massKg) const {
    const double thrustScale = std::max(0.0, 1.0 + config_.vehicle.thrustBiasPercent / 100.0);
    const double available = config_.vehicle.maxThrustN * config_.vehicle.maxThrottle * thrustScale
                           / std::max(config_.vehicle.dryMassKg, massKg) - kGravity;
    if (!std::isfinite(available) || available <= 1.0e-6) return 1.0e9;
    const double downwardSpeed = std::max(0.0, velocityNed.z);
    return downwardSpeed * downwardSpeed / (2.0 * available);
}

void RecoverySimulation::step(double dt) {
    if (dt <= 0.0 || complete()) return;
    dt = std::min(dt, config_.durationSec - truth_.time);
    RecoveryEstimatedState estimated = navigation_.estimate(truth_);
    Vec3 positionNed = ecefPositionToNed(estimated.positionEcefM, frame_);
    Vec3 velocityNed = ecefVectorToNed(estimated.velocityEcefMps, frame_);
    prediction_ = predictor_.predict(estimated, frame_, config_);
    lastBrakingDistanceM_ = brakingDistance(velocityNed, estimated.massKg);
    const double attitudeError = quaternionAngularDistance(rotation_.attitude,
        guidance_.desiredAttitude) * kRadToDeg;
    const double rateError = (rotation_.angularRate - guidance_.desiredAngularRateBodyRadPerSec).norm()
                           * kRadToDeg;
    const RecoveryPhase previousPhase = mission_.phase();
    mission_.update(estimated, positionNed, velocityNed, attitudeError, rateError,
                    prediction_, lastBrakingDistanceM_, config_);
    if (mission_.phase() != previousPhase) nextGuidanceTimeSec_ = truth_.time;
    if (truth_.time + 1.0e-10 >= nextGuidanceTimeSec_) {
        updateGuidance(estimated, positionNed, velocityNed, prediction_);
        nextGuidanceTimeSec_ = truth_.time + 1.0 / config_.guidanceFrequencyHz;
    }

    const ControllerOutput attitudeOutput = attitudeController_.update(
        rotation_.attitude, guidance_.desiredAttitude, rotation_.angularRate,
        guidance_.desiredAngularRateBodyRadPerSec, dt);
    lastActuator_ = allocator_.update(guidance_, attitudeOutput.desiredTorqueBody,
                                     estimated, mission_.phase(), dt, config_);
    double thrust = engine_.update(lastActuator_.engineOn, lastActuator_.requestedThrustN,
                                   truth_.propellantKg, dt, config_.vehicle);
    const double requestedPropellant = thrust
        / (std::max(1.0, config_.vehicle.specificImpulseSec) * kGravity) * dt;
    if (requestedPropellant > truth_.propellantKg && requestedPropellant > 0.0) {
        thrust *= truth_.propellantKg / requestedPropellant;
        truth_.propellantKg = 0.0;
    } else {
        truth_.propellantKg = std::max(0.0, truth_.propellantKg - requestedPropellant);
    }
    truth_.massKg = config_.vehicle.dryMassKg + truth_.propellantKg;
    lastThrustN_ = thrust;

    const Vec3 thrustBody{1.0, std::tan(lastActuator_.tvcYawRad),
                          -std::tan(lastActuator_.tvcPitchRad)};
    const Vec3 thrustDirectionEcef = rotation_.attitude.rotate(thrustBody.normalized());
    const Vec3 thrustAccelerationNed = ecefVectorToNed(thrustDirectionEcef, frame_)
                                    * (thrust / std::max(1.0, truth_.massKg));

    const double altitude = std::max(0.0, -positionNed.z);
    const double density = kSeaLevelDensity * std::max(0.0, config_.atmosphereDensityScale)
                         * std::exp(-altitude / kScaleHeightM);
    const Vec3 relativeAir = velocityNed - config_.windNedMps;
    const double airspeed = relativeAir.norm();
    lastDynamicPressurePa_ = 0.5 * density * airspeed * airspeed;
    const Vec3 dragAcceleration = airspeed > 1.0e-9
        ? relativeAir * (-0.5 * density * config_.vehicle.dragCoefficient
                         * config_.vehicle.referenceAreaM2 * airspeed
                         / std::max(1.0, truth_.massKg)) : Vec3{};
    const Vec3 airflowBody = rotation_.attitude.inverse().rotate(
        nedVectorToEcef(-relativeAir, frame_));
    GridFinAerodynamicInput finInput{};
    finInput.mach = airspeed / 340.0;
    finInput.dynamicPressurePa = lastDynamicPressurePa_;
    finInput.angleOfAttackRad = std::atan2(airflowBody.z,
                                           std::max(1.0e-9, airflowBody.x));
    finInput.sideslipRad = airspeed > 1.0e-9
        ? std::asin(clamp(airflowBody.y / airspeed, -1.0, 1.0)) : 0.0;
    finInput.deflectionRad = lastActuator_.gridFinRad;
    const GridFinAerodynamicOutput finOutput = gridFinAerodynamics_.evaluate(
        finInput, config_.vehicle);
    const Vec3 gridFinAcceleration = finOutput.forceNedN / std::max(1.0, truth_.massKg);
    const Vec3 accelerationNed = Vec3{0.0, 0.0, kGravity} + thrustAccelerationNed
                               + dragAcceleration + gridFinAcceleration;
    positionNed += velocityNed * dt + accelerationNed * (0.5 * dt * dt);
    velocityNed += accelerationNed * dt;
    truth_.positionEcefM = nedPositionToEcef(positionNed, frame_);
    truth_.velocityEcefMps = nedVectorToEcef(velocityNed, frame_);

    lastControlTorqueNm_ = lastActuator_.appliedMomentBodyNm + finOutput.momentBodyNm;
    const Vec3 dampingTorque = rotation_.angularRate * (-15000.0);
    integrateRotationalRK4(rotation_, Mat3::diagonal(config_.vehicle.inertiaKgM2),
                           lastControlTorqueNm_ + dampingTorque, dt);
    truth_.bodyToEcef = rotation_.attitude;
    truth_.angularRateBodyRadPerSec = rotation_.angularRate;
    truth_.time += dt;

    if (!finiteVector(positionNed) || !finiteVector(velocityNed)
        || !std::isfinite(rotation_.attitude.norm())) {
        mission_.fail(L"数值发散 / Numerical divergence");
    } else if (positionNed.z >= 0.0) {
        touchdownPositionNed_ = positionNed;
        touchdownPositionNed_.z = 0.0;
        touchdownVelocityNed_ = velocityNed;
        touchdownCaptured_ = true;
        positionNed.z = 0.0;
        truth_.positionEcefM = nedPositionToEcef(positionNed, frame_);
        truth_.velocityEcefMps = {};
        if (mission_.phase() == RecoveryPhase::Landing) {
            const bool safeTouchdown = std::hypot(touchdownPositionNed_.x, touchdownPositionNed_.y)
                    <= config_.touchdownPositionToleranceM
                && std::hypot(touchdownVelocityNed_.x, touchdownVelocityNed_.y)
                    <= config_.touchdownHorizontalVelocityMps
                && std::abs(touchdownVelocityNed_.z) <= config_.touchdownVerticalVelocityMps;
            if (safeTouchdown) mission_.touchdown();
            else mission_.fail(L"触地状态超过安全限制 / Unsafe touchdown limits exceeded");
        } else {
            mission_.fail(L"提前撞地 / Early ground impact");
        }
    } else if (truth_.propellantKg <= 1.0e-9
               && mission_.phase() != RecoveryPhase::AeroDescent
               && mission_.phase() != RecoveryPhase::Coast) {
        mission_.fail(L"推进剂耗尽 / Propellant depleted");
    } else if (truth_.time >= config_.durationSec) {
        mission_.fail(L"仿真超时 / Simulation timeout");
    }
    recordSample(complete());
}

} // namespace gnc
