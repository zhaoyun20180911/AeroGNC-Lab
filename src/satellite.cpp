#include "gnc/simulation.hpp"
#include "gnc/disturbance.hpp"
#include "gnc/orbit.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace gnc {
namespace {

constexpr double kEarthMu = kEarthGravitationalParameter;
constexpr double kEarthRadius = kEarthEquatorialRadiusM;
constexpr double kStandardGravity = 9.80665;

Quaternion lvlhReference(const Vec3& position, const Vec3& velocity) {
    const Vec3 radial = position.normalized();
    const Vec3 tangent = (velocity - radial * velocity.dot(radial)).normalized();
    const Vec3 bodyZ = -radial;                       // Payload boresight points to nadir.
    const Vec3 bodyX = tangent;                       // +X follows along-track velocity.
    const Vec3 bodyY = bodyZ.cross(bodyX).normalized();

    Mat3 bodyToInertial{};
    bodyToInertial.m[0][0] = bodyX.x;
    bodyToInertial.m[1][0] = bodyX.y;
    bodyToInertial.m[2][0] = bodyX.z;
    bodyToInertial.m[0][1] = bodyY.x;
    bodyToInertial.m[1][1] = bodyY.y;
    bodyToInertial.m[2][1] = bodyY.z;
    bodyToInertial.m[0][2] = bodyZ.x;
    bodyToInertial.m[1][2] = bodyZ.y;
    bodyToInertial.m[2][2] = bodyZ.z;
    return Quaternion::fromRotationMatrix(bodyToInertial);
}

Quaternion frameFromBoresight(const Vec3& boresightEci, const Vec3& preferredXEci) {
    const Vec3 bodyZ = boresightEci.normalized();
    Vec3 bodyX = (preferredXEci - bodyZ * preferredXEci.dot(bodyZ)).normalized();
    if (bodyX.normSquared() < 1.0e-12) bodyX = Vec3{0.0, 0.0, 1.0}.cross(bodyZ).normalized();
    if (bodyX.normSquared() < 1.0e-12) bodyX = {1.0, 0.0, 0.0};
    const Vec3 bodyY = bodyZ.cross(bodyX).normalized();
    Mat3 rotation{};
    rotation.m[0][0] = bodyX.x; rotation.m[1][0] = bodyX.y; rotation.m[2][0] = bodyX.z;
    rotation.m[0][1] = bodyY.x; rotation.m[1][1] = bodyY.y; rotation.m[2][1] = bodyY.z;
    rotation.m[0][2] = bodyZ.x; rotation.m[1][2] = bodyZ.y; rotation.m[2][2] = bodyZ.z;
    return Quaternion::fromRotationMatrix(rotation);
}

struct SatelliteReference {
    Quaternion attitude;
    Vec3 angularRateInertial;
};

SatelliteReference satelliteReference(const SatelliteConfig& config, double time,
                                      const Vec3& position, const Vec3& velocity,
                                      const Quaternion& slewHold) {
    switch (config.mission.objective) {
    case SatelliteObjective::InertialPointing:
        return {Quaternion::fromEulerZYX(config.mission.targetEulerDeg.x * kDegToRad,
                                         config.mission.targetEulerDeg.y * kDegToRad,
                                         config.mission.targetEulerDeg.z * kDegToRad), {}};
    case SatelliteObjective::TargetTracking: {
        constexpr double earthRotationRate = 7.2921159e-5;
        const double latitude = config.mission.targetLatitudeDeg * kDegToRad;
        const double longitude = config.mission.targetLongitudeDeg * kDegToRad + earthRotationRate * time;
        const Vec3 target = kEarthRadius * Vec3{std::cos(latitude) * std::cos(longitude),
                                                std::cos(latitude) * std::sin(longitude), std::sin(latitude)};
        return {frameFromBoresight(target - position, velocity), {}};
    }
    case SatelliteObjective::SlewManeuver:
        if (time < config.mission.slewStartSec) return {slewHold, {}};
        return {Quaternion::fromEulerZYX(config.mission.slewTargetEulerDeg.x * kDegToRad,
                                         config.mission.slewTargetEulerDeg.y * kDegToRad,
                                         config.mission.slewTargetEulerDeg.z * kDegToRad), {}};
    default:
        return {lvlhReference(position, velocity), position.cross(velocity) / position.normSquared()};
    }
}

Vec3 orbitAcceleration(const Vec3& position) {
    const double radius = position.norm();
    return position * (-kEarthMu / (radius * radius * radius));
}

void integrateOrbitRK4(Vec3& position, Vec3& velocity, const Vec3& externalAccelerationEci,
                       double dt) {
    const Vec3 k1Position = velocity;
    const Vec3 k1Velocity = orbitAcceleration(position) + externalAccelerationEci;
    const Vec3 k2Position = velocity + k1Velocity * (0.5 * dt);
    const Vec3 k2Velocity = orbitAcceleration(position + k1Position * (0.5 * dt))
                          + externalAccelerationEci;
    const Vec3 k3Position = velocity + k2Velocity * (0.5 * dt);
    const Vec3 k3Velocity = orbitAcceleration(position + k2Position * (0.5 * dt))
                          + externalAccelerationEci;
    const Vec3 k4Position = velocity + k3Velocity * dt;
    const Vec3 k4Velocity = orbitAcceleration(position + k3Position * dt)
                          + externalAccelerationEci;

    position += (k1Position + k2Position * 2.0 + k3Position * 2.0 + k4Position) * (dt / 6.0);
    velocity += (k1Velocity + k2Velocity * 2.0 + k3Velocity * 2.0 + k4Velocity) * (dt / 6.0);
}

struct RtnFrame {
    Vec3 radial;
    Vec3 alongTrack;
    Vec3 normal;
};

RtnFrame rtnFrame(const Vec3& position, const Vec3& velocity) {
    const Vec3 radial = position.normalized();
    const Vec3 normal = position.cross(velocity).normalized();
    return {radial, normal.cross(radial).normalized(), normal};
}

Vec3 rtnToEci(const Vec3& rtn, const RtnFrame& frame) {
    return frame.radial * rtn.x + frame.alongTrack * rtn.y + frame.normal * rtn.z;
}

Vec3 eciToRtn(const Vec3& eci, const RtnFrame& frame) {
    return {eci.dot(frame.radial), eci.dot(frame.alongTrack), eci.dot(frame.normal)};
}

double specificOrbitalEnergy(const Vec3& position, const Vec3& velocity) {
    return 0.5 * velocity.normSquared() - kEarthMu / std::max(1.0, position.norm());
}

Mat3 satellitePlantInertia(const SatelliteConfig& config) {
    Mat3 inertia = Mat3::diagonal(config.inertiaKgM2);
    if (config.inertiaPerturbation) {
        inertia.m[0][0] *= 1.08;
        inertia.m[1][1] *= 0.94;
        inertia.m[2][2] *= 1.05;
        const double coupling = 0.012 * std::min({config.inertiaKgM2.x, config.inertiaKgM2.y,
                                                 config.inertiaKgM2.z});
        inertia.m[0][1] = inertia.m[1][0] = coupling;
        inertia.m[1][2] = inertia.m[2][1] = -0.7 * coupling;
    }
    return inertia;
}

Vec3 toDegrees(const Vec3& radians) {
    return radians * kRadToDeg;
}

double vectorMaxAbs(const Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

double attitudeErrorMagnitudeDeg(const SimulationSample& sample) {
    return quaternionAngularDistance(sample.attitude, sample.referenceAttitude) * kRadToDeg;
}

PerformanceMetrics calculateMetrics(const std::vector<SimulationSample>& samples, bool rocket) {
    PerformanceMetrics metrics{};
    if (samples.empty()) {
        return metrics;
    }

    double errorSquaredSum{};
    std::size_t lastOutsideSettledBand{};
    bool everOutsideSettledBand = false;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const SimulationSample& sample = samples[index];
        const double error = attitudeErrorMagnitudeDeg(sample);
        metrics.maxAttitudeErrorDeg = std::max(metrics.maxAttitudeErrorDeg, error);
        errorSquaredSum += error * error;
        metrics.maxControlTorque = std::max(metrics.maxControlTorque, vectorMaxAbs(sample.controlTorque));
        metrics.maxAngularRateDegPerSec = std::max(
            metrics.maxAngularRateDegPerSec, vectorMaxAbs(sample.angularRateDegPerSec));
        metrics.maxDynamicPressure = std::max(metrics.maxDynamicPressure, sample.dynamicPressure);
        metrics.saturated = metrics.saturated || sample.actuatorSaturated;
        metrics.maxActuatorValue = std::max(metrics.maxActuatorValue,
            rocket ? std::max(std::abs(sample.tvcPitchDeg), std::abs(sample.tvcYawDeg))
                   : vectorMaxAbs(sample.wheelSpeedRpm));
        if (!rocket) {
            metrics.maxPositionError = std::max(metrics.maxPositionError, sample.positionError);
            metrics.maxOrbitControlThrustN = std::max(metrics.maxOrbitControlThrustN, sample.thrust);
        }
        if (error > (rocket ? 1.0 : 0.5)) {
            lastOutsideSettledBand = index;
            everOutsideSettledBand = true;
        }
    }
    metrics.rmsAttitudeErrorDeg = std::sqrt(errorSquaredSum / static_cast<double>(samples.size()));
    if (!everOutsideSettledBand) {
        metrics.settlingTimeSec = 0.0;
    } else if (lastOutsideSettledBand + 1 < samples.size()) {
        metrics.settlingTimeSec = samples[lastOutsideSettledBand + 1].time;
    }
    metrics.finalMass = samples.back().mass;
    if (!rocket) {
        metrics.finalPositionError = samples.back().positionError;
        metrics.finalVelocityError = samples.back().velocityError;
        metrics.cumulativeDeltaVMps = samples.back().cumulativeDeltaVMps;
        metrics.propellantUsedKg = std::max(0.0,
            samples.front().propellantRemaining - samples.back().propellantRemaining);
    }
    return metrics;
}

} // namespace

SatelliteSimulation::SatelliteSimulation(const SatelliteConfig& config) {
    reset(config);
}

void SatelliteSimulation::reset(const SatelliteConfig& config) {
    config_ = config;
    config_.orbitAltitudeKm = clamp(config_.orbitAltitudeKm, 120.0, 36000.0);
    config_.durationSec = clamp(config_.durationSec, 1.0, 86400.0);
    config_.wheelMaxTorqueNm = std::max(1.0e-5, config_.wheelMaxTorqueNm);
    config_.wheelMaxSpeedRpm = std::max(1.0, config_.wheelMaxSpeedRpm);
    config_.orbitControl.maxThrustN = std::max(0.0, config_.orbitControl.maxThrustN);
    config_.orbitControl.specificImpulseSec = std::max(1.0, config_.orbitControl.specificImpulseSec);
    config_.orbitControl.propellantMassKg = clamp(
        config_.orbitControl.propellantMassKg, 0.0, std::max(0.0, config_.massKg - 1.0));
    config_.orbitControl.positionDeadbandM = std::max(0.0, config_.orbitControl.positionDeadbandM);
    config_.orbitControl.velocityDeadbandMps = std::max(0.0, config_.orbitControl.velocityDeadbandMps);

    time_ = 0.0;
    nextSampleTime_ = 0.0;
    if (std::abs(config_.orbit.semiMajorAxisKm - 6878.137) < 1.0e-6
        && std::abs(config_.orbitAltitudeKm - 500.0) > 1.0e-9) {
        config_.orbit.semiMajorAxisKm = kEarthRadius / 1000.0 + config_.orbitAltitudeKm;
    }
    if (!validOrbitElements(config_.orbit)) config_.orbit = {};
    const CartesianOrbitState orbitState = classicalElementsToCartesian(config_.orbit);
    positionEci_ = orbitState.positionEciM;
    velocityEci_ = orbitState.velocityEciMps;
    nominalPositionEci_ = positionEci_;
    nominalVelocityEci_ = velocityEci_;
    const Quaternion provisionalReference = config_.mission.objective == SatelliteObjective::NadirPointing
        ? lvlhReference(positionEci_, velocityEci_)
        : satelliteReference(config_, 0.0, positionEci_, velocityEci_, Quaternion::identity()).attitude;
    const Quaternion initialOffset = Quaternion::fromEulerZYX(
        config_.initialErrorDeg.x * kDegToRad,
        config_.initialErrorDeg.y * kDegToRad,
        config_.initialErrorDeg.z * kDegToRad);
    rotation_.attitude = (provisionalReference * initialOffset).normalized();
    slewHoldAttitude_ = rotation_.attitude;
    const SatelliteReference initialReference = satelliteReference(
        config_, 0.0, positionEci_, velocityEci_, slewHoldAttitude_);
    rotation_.angularRate = rotation_.attitude.conjugate().rotate(initialReference.angularRateInertial)
        + config_.initialRateDegPerSec * kDegToRad;
    wheelSpeedRadPerSec_ = {};
    lastTorque_ = {};
    lastOrbitControlForceBodyN_ = {};
    massKg_ = std::max(1.0, config_.massKg);
    propellantRemainingKg_ = config_.orbitControl.propellantMassKg;
    dryMassKg_ = massKg_ - propellantRemainingKg_;
    cumulativeDeltaVMps_ = 0.0;
    appliedImpulseDeltaVMps_ = 0.0;
    lastSaturated_ = false;
    lastOrbitControlActive_ = false;
    lastOrbitControlSaturated_ = false;
    deltaVImpulseApplied_ = false;
    history_.clear();
    controller_.configure(config_.control, ControlVehicle::Satellite, config_.inertiaKgM2,
                          {config_.wheelMaxTorqueNm, config_.wheelMaxTorqueNm, config_.wheelMaxTorqueNm});
    recordSample(true);
}

void SatelliteSimulation::step(double dt) {
    if (dt <= 0.0 || complete()) {
        return;
    }
    dt = std::min(dt, config_.durationSec - time_);

    if (config_.disturbanceEnabled && config_.disturbances.deltaVImpulseEnabled
        && !deltaVImpulseApplied_
        && time_ <= config_.disturbances.deltaVImpulseTimeSec
        && time_ + dt >= config_.disturbances.deltaVImpulseTimeSec) {
        const Vec3 deltaVEci = rtnToEci(config_.disturbances.deltaVImpulseRtnMps,
                                        rtnFrame(positionEci_, velocityEci_));
        velocityEci_ += deltaVEci;
        appliedImpulseDeltaVMps_ = deltaVEci.norm();
        deltaVImpulseApplied_ = true;
    }

    const Vec3 positionErrorEci = positionEci_ - nominalPositionEci_;
    const Vec3 velocityErrorEci = velocityEci_ - nominalVelocityEci_;
    Vec3 orbitControlForceEci{};
    lastOrbitControlActive_ = false;
    lastOrbitControlSaturated_ = false;
    if (config_.orbitControl.enabled && propellantRemainingKg_ > 1.0e-12
        && config_.orbitControl.maxThrustN > 0.0
        && (positionErrorEci.norm() > config_.orbitControl.positionDeadbandM
            || velocityErrorEci.norm() > config_.orbitControl.velocityDeadbandMps)) {
        const Vec3 commandedAcceleration = orbitAcceleration(nominalPositionEci_)
            - orbitAcceleration(positionEci_)
            - positionErrorEci * std::max(0.0, config_.orbitControl.positionGainPerSec2)
            - velocityErrorEci * std::max(0.0, config_.orbitControl.velocityGainPerSec);
        orbitControlForceEci = commandedAcceleration * massKg_;
        const double requestedThrust = orbitControlForceEci.norm();
        if (requestedThrust > config_.orbitControl.maxThrustN) {
            orbitControlForceEci *= config_.orbitControl.maxThrustN / requestedThrust;
            lastOrbitControlSaturated_ = true;
        }
        const double thrust = orbitControlForceEci.norm();
        const double massFlow = thrust / (config_.orbitControl.specificImpulseSec * kStandardGravity);
        const double burnFraction = massFlow > 0.0
            ? std::min(1.0, propellantRemainingKg_ / (massFlow * dt)) : 0.0;
        if (burnFraction < 1.0) lastOrbitControlSaturated_ = true;
        orbitControlForceEci *= burnFraction;
        const double actualThrust = orbitControlForceEci.norm();
        propellantRemainingKg_ = std::max(0.0,
            propellantRemainingKg_ - massFlow * burnFraction * dt);
        cumulativeDeltaVMps_ += actualThrust / std::max(1.0, massKg_) * dt;
        massKg_ = dryMassKg_ + propellantRemainingKg_;
        lastOrbitControlActive_ = actualThrust > 0.0;
    }
    lastOrbitControlForceBodyN_ = rotation_.attitude.inverse().rotate(orbitControlForceEci);
    const Vec3 orbitControlAcceleration = orbitControlForceEci / std::max(1.0, massKg_);
    integrateOrbitRK4(nominalPositionEci_, nominalVelocityEci_, {}, dt);
    integrateOrbitRK4(positionEci_, velocityEci_, orbitControlAcceleration, dt);
    const SatelliteReference reference = satelliteReference(
        config_, time_, positionEci_, velocityEci_, slewHoldAttitude_);
    const Quaternion target = reference.attitude;
    const Vec3 targetRateBody = rotation_.attitude.conjugate().rotate(reference.angularRateInertial);

    const ControllerOutput controllerOutput = controller_.update(
        rotation_.attitude, target, rotation_.angularRate, targetRateBody, dt);
    const Vec3 desiredTorque = controllerOutput.desiredTorqueBody;

    const double wheelLimitRadPerSec = config_.wheelMaxSpeedRpm * (2.0 * kPi / 60.0);
    const double wheelInertia = std::max(1.0e-6, config_.wheelInertiaKgM2);
    double desired[3]{desiredTorque.x, desiredTorque.y, desiredTorque.z};
    double speeds[3]{wheelSpeedRadPerSec_.x, wheelSpeedRadPerSec_.y, wheelSpeedRadPerSec_.z};
    double bodyTorque[3]{};
    lastSaturated_ = controllerOutput.torqueLimited || lastOrbitControlSaturated_;
    for (int axis = 0; axis < 3; ++axis) {
        bodyTorque[axis] = clamp(desired[axis], -config_.wheelMaxTorqueNm, config_.wheelMaxTorqueNm);
        if (bodyTorque[axis] != desired[axis]) {
            lastSaturated_ = true;
        }
        double motorTorque = -bodyTorque[axis];
        if (std::abs(speeds[axis]) >= wheelLimitRadPerSec && speeds[axis] * motorTorque > 0.0) {
            motorTorque = 0.0;
            bodyTorque[axis] = 0.0;
            lastSaturated_ = true;
        }
        speeds[axis] += (motorTorque / wheelInertia) * dt;
        if (std::abs(speeds[axis]) > wheelLimitRadPerSec) {
            speeds[axis] = std::copysign(wheelLimitRadPerSec, speeds[axis]);
            lastSaturated_ = true;
        }
    }
    wheelSpeedRadPerSec_ = {speeds[0], speeds[1], speeds[2]};
    const Vec3 actuatorTorque{bodyTorque[0], bodyTorque[1], bodyTorque[2]};
    const Vec3 disturbance = config_.disturbanceEnabled
        ? satelliteDisturbanceTorque(config_.disturbances, time_) : Vec3{};
    lastTorque_ = actuatorTorque;
    integrateRotationalRK4(rotation_, satellitePlantInertia(config_), actuatorTorque + disturbance, dt);
    time_ += dt;
    recordSample(complete());
}

void SatelliteSimulation::recordSample(bool force) {
    const Quaternion target = satelliteReference(
        config_, time_, positionEci_, velocityEci_, slewHoldAttitude_).attitude;
    const Quaternion error = attitudeError(rotation_.attitude, target);
    const Vec3 wheelRpm = wheelSpeedRadPerSec_ * (60.0 / (2.0 * kPi));
    const Vec3 positionDifference = positionEci_ - nominalPositionEci_;
    const Vec3 velocityDifference = velocityEci_ - nominalVelocityEci_;
    const RtnFrame frame = rtnFrame(nominalPositionEci_, nominalVelocityEci_);
    currentSample_ = {};
    currentSample_.time = time_;
    currentSample_.position = positionEci_;
    currentSample_.velocity = velocityEci_;
    currentSample_.nominalPosition = nominalPositionEci_;
    currentSample_.nominalVelocity = nominalVelocityEci_;
    currentSample_.attitude = rotation_.attitude;
    currentSample_.referenceAttitude = target;
    currentSample_.eulerDeg = toDegrees(rotation_.attitude.toEulerZYX());
    currentSample_.referenceEulerDeg = toDegrees(target.toEulerZYX());
    currentSample_.attitudeErrorDeg = toDegrees(error.toEulerZYX());
    currentSample_.angularRateDegPerSec = toDegrees(rotation_.angularRate);
    currentSample_.controlTorque = lastTorque_;
    currentSample_.wheelSpeedRpm = wheelRpm;
    currentSample_.mass = massKg_;
    currentSample_.altitude = positionEci_.norm() - kEarthRadius;
    currentSample_.speed = velocityEci_.norm();
    currentSample_.orbitRadius = positionEci_.norm();
    currentSample_.nominalAltitude = nominalPositionEci_.norm() - kEarthRadius;
    currentSample_.nominalSpeed = nominalVelocityEci_.norm();
    currentSample_.positionError = positionDifference.norm();
    currentSample_.velocityError = velocityDifference.norm();
    currentSample_.orbitPositionErrorRtnM = eciToRtn(positionDifference, frame);
    currentSample_.orbitVelocityErrorRtnMps = eciToRtn(velocityDifference, frame);
    currentSample_.radialPositionError = currentSample_.orbitPositionErrorRtnM.x;
    currentSample_.alongTrackPositionError = currentSample_.orbitPositionErrorRtnM.y;
    currentSample_.crossTrackError = currentSample_.orbitPositionErrorRtnM.z;
    currentSample_.altitudeError = currentSample_.altitude - currentSample_.nominalAltitude;
    currentSample_.orbitControlForceBodyN = lastOrbitControlForceBodyN_;
    currentSample_.thrust = lastOrbitControlForceBodyN_.norm();
    currentSample_.propellantRemaining = propellantRemainingKg_;
    currentSample_.cumulativeDeltaVMps = cumulativeDeltaVMps_;
    currentSample_.appliedImpulseDeltaVMps = appliedImpulseDeltaVMps_;
    currentSample_.specificEnergyError = specificOrbitalEnergy(positionEci_, velocityEci_)
                                       - specificOrbitalEnergy(nominalPositionEci_, nominalVelocityEci_);
    currentSample_.actuatorSaturated = lastSaturated_;
    currentSample_.controlActive = controller_.active();
    currentSample_.guidanceActive = lastOrbitControlActive_;
    currentSample_.orbitControlSaturated = lastOrbitControlSaturated_;
    currentSample_.propellantDepleted = config_.orbitControl.enabled
                                     && propellantRemainingKg_ <= 1.0e-12;
    currentSample_.translationalImpulseApplied = deltaVImpulseApplied_;

    if (force || time_ + 1.0e-10 >= nextSampleTime_) {
        if (history_.empty() || std::abs(history_.back().time - time_) > 1.0e-9) {
            history_.push_back(currentSample_);
        } else {
            history_.back() = currentSample_;
        }
        nextSampleTime_ = time_ + 0.10;
    }
}

PerformanceMetrics SatelliteSimulation::metrics() const {
    return calculateMetrics(history_, false);
}

bool exportCsv(const std::wstring& path, ScenarioKind scenario,
               const std::vector<SimulationSample>& samples) {
    std::ofstream stream(std::filesystem::path(path), std::ios::binary);
    if (!stream) {
        return false;
    }
    stream << "time_s,position_x_m,position_y_m,position_z_m,velocity_x_mps,velocity_y_mps,velocity_z_mps,"
              "q_w,q_x,q_y,q_z,roll_deg,pitch_deg,yaw_deg,reference_roll_deg,reference_pitch_deg,"
              "reference_yaw_deg,error_roll_deg,error_pitch_deg,error_yaw_deg,omega_x_deg_s,omega_y_deg_s,"
              "omega_z_deg_s,torque_x_nm,torque_y_nm,torque_z_nm,mass_kg,altitude_m,speed_mps,"
              "dynamic_pressure_pa,actuator_saturated";
    if (scenario == ScenarioKind::Satellite) {
        stream << ",wheel_x_rpm,wheel_y_rpm,wheel_z_rpm,orbit_radius_m,"
                  "nominal_position_x_m,nominal_position_y_m,nominal_position_z_m,"
                  "nominal_velocity_x_mps,nominal_velocity_y_mps,nominal_velocity_z_mps,"
                  "position_error_r_m,position_error_t_m,position_error_n_m,"
                  "velocity_error_r_mps,velocity_error_t_mps,velocity_error_n_mps,"
                  "orbit_force_body_x_n,orbit_force_body_y_n,orbit_force_body_z_n,orbit_thrust_n,"
                  "propellant_remaining_kg,cumulative_delta_v_mps,specific_energy_error_j_per_kg,"
                  "delta_v_impulse_applied,applied_impulse_delta_v_mps,orbit_control_active,orbit_control_saturated";
    } else {
        stream << ",nominal_position_x_m,nominal_position_y_m,nominal_position_z_m,"
                  "nominal_velocity_x_mps,nominal_velocity_y_mps,nominal_velocity_z_mps,"
                  "reference_q_w,reference_q_x,reference_q_y,reference_q_z,"
                  "nominal_reference_q_w,nominal_reference_q_x,nominal_reference_q_y,nominal_reference_q_z,"
                  "nominal_altitude_m,nominal_speed_mps,nominal_pitch_deg,actual_pitch_deg,"
                  "position_error_m,cross_track_error_m,radial_position_error_m,along_track_position_error_m,"
                  "altitude_error_m,velocity_error_mps,guidance_pitch_correction_deg,guidance_yaw_correction_deg,"
                  "propellant_remaining_kg,specific_energy_error_j_per_kg,"
                  "tvc_pitch_deg,tvc_yaw_deg,thrust_n,drag_n,mission_id,stage_id,orbital_coast,"
                  "ascent_tracking_active,guidance_active,terminal_guidance_active,propellant_depleted,engine_cutoff";
    }
    stream << "\r\n" << std::setprecision(12);
    for (const SimulationSample& sample : samples) {
        stream << sample.time << ','
               << sample.position.x << ',' << sample.position.y << ',' << sample.position.z << ','
               << sample.velocity.x << ',' << sample.velocity.y << ',' << sample.velocity.z << ','
               << sample.attitude.w << ',' << sample.attitude.x << ',' << sample.attitude.y << ',' << sample.attitude.z << ','
               << sample.eulerDeg.x << ',' << sample.eulerDeg.y << ',' << sample.eulerDeg.z << ','
               << sample.referenceEulerDeg.x << ',' << sample.referenceEulerDeg.y << ',' << sample.referenceEulerDeg.z << ','
               << sample.attitudeErrorDeg.x << ',' << sample.attitudeErrorDeg.y << ',' << sample.attitudeErrorDeg.z << ','
               << sample.angularRateDegPerSec.x << ',' << sample.angularRateDegPerSec.y << ',' << sample.angularRateDegPerSec.z << ','
               << sample.controlTorque.x << ',' << sample.controlTorque.y << ',' << sample.controlTorque.z << ','
               << sample.mass << ',' << sample.altitude << ',' << sample.speed << ',' << sample.dynamicPressure << ','
               << (sample.actuatorSaturated ? 1 : 0);
        if (scenario == ScenarioKind::Satellite) {
            stream << ',' << sample.wheelSpeedRpm.x << ',' << sample.wheelSpeedRpm.y << ','
                   << sample.wheelSpeedRpm.z << ',' << sample.orbitRadius << ','
                   << sample.nominalPosition.x << ',' << sample.nominalPosition.y << ','
                   << sample.nominalPosition.z << ',' << sample.nominalVelocity.x << ','
                   << sample.nominalVelocity.y << ',' << sample.nominalVelocity.z << ','
                   << sample.orbitPositionErrorRtnM.x << ',' << sample.orbitPositionErrorRtnM.y << ','
                   << sample.orbitPositionErrorRtnM.z << ',' << sample.orbitVelocityErrorRtnMps.x << ','
                   << sample.orbitVelocityErrorRtnMps.y << ',' << sample.orbitVelocityErrorRtnMps.z << ','
                   << sample.orbitControlForceBodyN.x << ',' << sample.orbitControlForceBodyN.y << ','
                   << sample.orbitControlForceBodyN.z << ',' << sample.thrust << ','
                   << sample.propellantRemaining << ',' << sample.cumulativeDeltaVMps << ','
                   << sample.specificEnergyError << ','
                   << (sample.translationalImpulseApplied ? 1 : 0) << ','
                   << sample.appliedImpulseDeltaVMps << ',' << (sample.guidanceActive ? 1 : 0) << ','
                   << (sample.orbitControlSaturated ? 1 : 0);
        } else {
            stream << ',' << sample.nominalPosition.x << ',' << sample.nominalPosition.y << ','
                   << sample.nominalPosition.z << ',' << sample.nominalVelocity.x << ','
                   << sample.nominalVelocity.y << ',' << sample.nominalVelocity.z << ','
                   << sample.referenceAttitude.w << ',' << sample.referenceAttitude.x << ','
                   << sample.referenceAttitude.y << ',' << sample.referenceAttitude.z << ','
                   << sample.nominalReferenceAttitude.w << ',' << sample.nominalReferenceAttitude.x << ','
                   << sample.nominalReferenceAttitude.y << ',' << sample.nominalReferenceAttitude.z << ','
                   << sample.nominalAltitude << ',' << sample.nominalSpeed << ','
                   << sample.nominalPitchDeg << ',' << sample.actualPitchDeg << ','
                   << sample.positionError << ',' << sample.crossTrackError << ','
                   << sample.radialPositionError << ',' << sample.alongTrackPositionError << ','
                   << sample.altitudeError << ',' << sample.velocityError << ','
                   << sample.guidanceCorrectionDeg.y << ',' << sample.guidanceCorrectionDeg.z << ','
                   << sample.propellantRemaining << ',' << sample.specificEnergyError << ','
                   << sample.tvcPitchDeg << ',' << sample.tvcYawDeg << ','
                   << sample.thrust << ',' << sample.drag << ',' << sample.missionId << ','
                   << sample.stageId << ',' << (sample.orbitalCoast ? 1 : 0) << ','
                   << (sample.ascentTrackingActive ? 1 : 0) << ','
                   << (sample.guidanceActive ? 1 : 0) << ','
                   << (sample.terminalGuidanceActive ? 1 : 0) << ','
                   << (sample.propellantDepleted ? 1 : 0) << ','
                   << (sample.engineCutoff ? 1 : 0);
        }
        stream << "\r\n";
    }
    return stream.good();
}

} // namespace gnc
