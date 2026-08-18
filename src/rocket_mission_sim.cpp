#include "gnc/simulation.hpp"
#include "gnc/disturbance.hpp"
#include "gnc/orbit.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gnc {
namespace {

constexpr double kEarthMu = 3.986004418e14;
constexpr double kEarthRadius = 6378137.0;
constexpr double kEarthRotationRate = 7.2921159e-5;
constexpr double kStandardGravity = 9.80665;
constexpr double kSeaLevelDensity = 1.225;
constexpr double kAtmosphereScaleHeight = 8500.0;

Vec3 gravityEci(const Vec3& position) {
    const double radius = position.norm();
    return position * (-kEarthMu / (radius * radius * radius));
}

void integrateTwoBody(Vec3& position, Vec3& velocity, double dt) {
    const Vec3 k1r = velocity;
    const Vec3 k1v = gravityEci(position);
    const Vec3 k2r = velocity + k1v * (0.5 * dt);
    const Vec3 k2v = gravityEci(position + k1r * (0.5 * dt));
    const Vec3 k3r = velocity + k2v * (0.5 * dt);
    const Vec3 k3v = gravityEci(position + k2r * (0.5 * dt));
    const Vec3 k4r = velocity + k3v * dt;
    const Vec3 k4v = gravityEci(position + k3r * dt);
    position += (k1r + k2r * 2.0 + k3r * 2.0 + k4r) * (dt / 6.0);
    velocity += (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
}

double densityAtAltitude(double altitudeM) {
    if (altitudeM >= 120000.0) return 0.0;
    return kSeaLevelDensity * std::exp(-std::max(0.0, altitudeM) / kAtmosphereScaleHeight);
}

double percentScale(double percent) {
    return std::max(0.01, 1.0 + percent / 100.0);
}

Vec3 interpolateInertia(const RocketMissionSummary& summary, double actualMass,
                        const RocketModelDeviations& deviations) {
    const double nominalWet = summary.nominalWetMassKg();
    const double nominalDry = summary.payloadMassKg + summary.stage2DryMassKg;
    const double nominalEquivalentMass = actualMass / percentScale(deviations.massPercent);
    const double fraction = clamp((nominalEquivalentMass - nominalDry) /
        std::max(1.0, nominalWet - nominalDry), 0.0, 1.0);
    return (summary.nominalDryInertiaKgM2
        + (summary.nominalWetInertiaKgM2 - summary.nominalDryInertiaKgM2) * fraction)
        * percentScale(deviations.inertiaPercent);
}

Vec3 inertiaDerivative(const RocketMissionSummary& summary, double actualMassDerivative,
                       const RocketModelDeviations& deviations) {
    const double nominalMassRange = std::max(1.0, summary.nominalWetMassKg()
        - summary.payloadMassKg - summary.stage2DryMassKg);
    return (summary.nominalWetInertiaKgM2 - summary.nominalDryInertiaKgM2)
        * (actualMassDerivative / (percentScale(deviations.massPercent) * nominalMassRange))
        * percentScale(deviations.inertiaPercent);
}

double maxAbs(const Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

double specificOrbitalEnergy(const Vec3& position, const Vec3& velocity) {
    return 0.5 * velocity.normSquared() - kEarthMu / std::max(1.0, position.norm());
}

Vec3 rotateToward(const Vec3& fromValue, const Vec3& toValue, double maximumAngle) {
    const Vec3 from = fromValue.normalized();
    const Vec3 to = toValue.normalized();
    const double angle = std::acos(clamp(from.dot(to), -1.0, 1.0));
    if (angle <= maximumAngle || angle < 1.0e-12) return to;
    Vec3 axis = from.cross(to).normalized();
    if (axis.normSquared() < 1.0e-12) {
        axis = std::abs(from.x) < 0.8 ? from.cross({1.0, 0.0, 0.0}).normalized()
                                      : from.cross({0.0, 1.0, 0.0}).normalized();
    }
    return Quaternion::fromAxisAngle(axis, maximumAngle).rotate(from).normalized();
}

Quaternion attitudeFromForward(const Vec3& forwardEci, const Quaternion& nominalAttitude) {
    const Vec3 xAxis = forwardEci.normalized();
    const Vec3 nominalYAxis = nominalAttitude.rotate({0.0, 1.0, 0.0});
    Vec3 yAxis = (nominalYAxis - xAxis * xAxis.dot(nominalYAxis)).normalized();
    if (yAxis.normSquared() < 1.0e-12) {
        const Vec3 fallback = std::abs(xAxis.z) < 0.8 ? Vec3{0.0, 0.0, 1.0}
                                                      : Vec3{0.0, 1.0, 0.0};
        yAxis = fallback.cross(xAxis).normalized();
    }
    const Vec3 zAxis = xAxis.cross(yAxis).normalized();
    Mat3 rotation{};
    rotation.m[0][0] = xAxis.x; rotation.m[1][0] = xAxis.y; rotation.m[2][0] = xAxis.z;
    rotation.m[0][1] = yAxis.x; rotation.m[1][1] = yAxis.y; rotation.m[2][1] = yAxis.z;
    rotation.m[0][2] = zAxis.x; rotation.m[1][2] = zAxis.y; rotation.m[2][2] = zAxis.z;
    return Quaternion::fromRotationMatrix(rotation).normalized();
}

} // namespace

RocketMissionSimulation::RocketMissionSimulation(const RocketMissionConfig& config) {
    reset(config);
}

void RocketMissionSimulation::reset(const RocketMissionConfig& config) {
    config_ = config;
    config_.durationSec = clamp(config_.durationSec, 1.0, 86400.0);
    time_ = 0.0;
    nextSampleTime_ = 0.0;
    tvcPitchRad_ = 0.0;
    tvcYawRad_ = 0.0;
    previousStageId_ = 0;
    lastTorque_ = {};
    lastDynamicPressure_ = 0.0;
    lastDrag_ = 0.0;
    lastSaturated_ = false;
    nominalCoastInitialized_ = false;
    insertionCaptured_ = false;
    stage1Separated_ = false;
    stage2CutoffLatched_ = false;
    orbitalCoast_ = false;
    lastGuidanceActive_ = false;
    lastTerminalGuidanceActive_ = false;
    lastPropellantDepleted_ = false;
    lastEngineCutoff_ = false;
    nominalInsertionTimeSec_ = 0.0;
    targetSpecificEnergyJPerKg_ = 0.0;
    targetOrbitNormalEci_ = {};
    commandedAttitude_ = {};
    previousCommandDirectionEci_ = {};
    lastGuidanceCorrectionDeg_ = {};
    lastActualThrust_ = 0.0;
    lastSpecificEnergyError_ = 0.0;
    insertionSample_ = {};
    history_.clear();
    currentSample_ = {};
    if (!config_.mission || config_.mission->trajectory.empty()) return;

    const NominalTrajectoryPoint initial = config_.mission->trajectory.front();
    positionEci_ = initial.positionEciM;
    velocityEci_ = initial.velocityEciMps;
    const Quaternion offset = Quaternion::fromEulerZYX(
        config_.initialAttitudeErrorDeg.x * kDegToRad,
        config_.initialAttitudeErrorDeg.y * kDegToRad,
        config_.initialAttitudeErrorDeg.z * kDegToRad);
    rotation_.attitude = (initial.referenceAttitude * offset).normalized();
    rotation_.angularRate = initial.referenceAngularRateRadPerSec
        + config_.initialRateErrorDegPerSec * kDegToRad;
    const double massScale = percentScale(config_.deviations.massPercent);
    stage1PropellantRemainingKg_ = config_.mission->summary.stage1PropellantMassKg * massScale;
    stage2PropellantRemainingKg_ = config_.mission->summary.stage2PropellantMassKg * massScale;
    mass_ = config_.mission->summary.nominalWetMassKg() * massScale;
    previousStageId_ = initial.stageId;
    commandedAttitude_ = initial.referenceAttitude;
    previousCommandDirectionEci_ = initial.referenceAttitude.rotate({1.0, 0.0, 0.0});
    const auto insertion = std::find_if(config_.mission->trajectory.begin(),
        config_.mission->trajectory.end(), [](const NominalTrajectoryPoint& point) {
            return point.stageId >= 4;
        });
    const NominalTrajectoryPoint& target = insertion == config_.mission->trajectory.end()
        ? config_.mission->trajectory.back() : *insertion;
    nominalInsertionTimeSec_ = target.timeSec;
    targetSpecificEnergyJPerKg_ = specificOrbitalEnergy(target.positionEciM, target.velocityEciMps);
    targetOrbitNormalEci_ = target.positionEciM.cross(target.velocityEciMps).normalized();
    const double actualInitialThrust = initial.thrustN * percentScale(config_.deviations.thrustPercent);
    const double maximumPitchYawTorque = actualInitialThrust * config_.mission->summary.thrustLeverArmM
        * std::sin(config_.mission->summary.maxTvcAngleDeg * kDegToRad);
    controller_.configure(config_.control, ControlVehicle::LaunchVehicle,
        interpolateInertia(config_.mission->summary, mass_, config_.deviations),
        {config_.mission->summary.maxRollRcsTorqueNm, maximumPitchYawTorque, maximumPitchYawTorque});
    recordSample(true);
}

void RocketMissionSimulation::advanceNominalCoast(double dt) {
    if (!config_.mission) return;
    if (!nominalCoastInitialized_) {
        const auto& last = config_.mission->trajectory.back();
        nominalCoastPositionEci_ = last.positionEciM;
        nominalCoastVelocityEci_ = last.velocityEciMps;
        nominalCoastInitialized_ = true;
    }
    integrateTwoBody(nominalCoastPositionEci_, nominalCoastVelocityEci_, dt);
}

void RocketMissionSimulation::step(double dt) {
    if (!valid() || dt <= 0.0 || complete()) return;
    dt = std::min(dt, config_.durationSec - time_);
    const RocketMissionData& mission = *config_.mission;
    const RocketMissionSummary& summary = mission.summary;
    NominalTrajectoryPoint nominal = mission.interpolate(time_);
    if (time_ > mission.endTimeSec() && nominalCoastInitialized_) {
        nominal.positionEciM = nominalCoastPositionEci_;
        nominal.velocityEciMps = nominalCoastVelocityEci_;
        nominal.altitudeM = nominal.positionEciM.norm() - kEarthRadius;
        nominal.speedMps = nominal.velocityEciMps.norm();
    }
    const double massScale = percentScale(config_.deviations.massPercent);

    if (!stage1Separated_ && nominal.stageId >= 2) {
        mass_ -= summary.stage1DryMassKg * massScale + stage1PropellantRemainingKg_;
        stage1PropellantRemainingKg_ = 0.0;
        stage1Separated_ = true;
    }
    previousStageId_ = nominal.stageId;

    const bool stage1Phase = !stage1Separated_ && nominal.stageId == 1;
    const bool nominalStage2Burn = nominal.stageId == 3 && nominal.thrustN > 0.0;
    const double currentSpecificEnergy = specificOrbitalEnergy(positionEci_, velocityEci_);
    lastSpecificEnergyError_ = currentSpecificEnergy - targetSpecificEnergyJPerKg_;
    const RocketGuidanceConfig& guidance = config_.guidance;
    const bool terminalWindow = guidance.enabled && guidance.terminalOrbitGuidance
        && time_ >= nominalInsertionTimeSec_ - std::max(0.0, guidance.terminalLeadTimeSec);
    const double cutoffAltitude = positionEci_.norm() - kEarthRadius;
    const bool cutoffWindow = time_ >= nominalInsertionTimeSec_
        - std::min(30.0, std::max(0.0, guidance.terminalLeadTimeSec));
    if (!stage2CutoffLatched_ && nominal.stageId >= 3 && terminalWindow
        && guidance.adaptiveCutoff && cutoffWindow
        && cutoffAltitude >= summary.targetAltitudeKm * 1000.0 - 60000.0
        && lastSpecificEnergyError_ >= -std::max(0.0, guidance.specificEnergyToleranceJPerKg)) {
        stage2CutoffLatched_ = true;
    }
    const bool extensionAllowed = terminalWindow && guidance.adaptiveCutoff
        && nominal.stageId >= 4 && !stage2CutoffLatched_
        && time_ <= nominalInsertionTimeSec_ + std::max(0.0, guidance.maxBurnExtensionSec)
        && lastSpecificEnergyError_ < -std::max(0.0, guidance.specificEnergyToleranceJPerKg);
    const bool stage2Demand = !stage2CutoffLatched_ && (nominalStage2Burn || extensionAllowed);
    const bool propellantAvailable = stage1Phase ? stage1PropellantRemainingKg_ > 1.0e-8
                                                 : stage2PropellantRemainingKg_ > 1.0e-8;
    const bool engineDemand = (stage1Phase && nominal.thrustN > 0.0) || stage2Demand;
    const bool powered = engineDemand && propellantAvailable;

    const double nominalStageThrust = stage1Phase ? summary.stage1ThrustN : summary.stage2ThrustN;
    const double rawActualThrust = powered
        ? nominalStageThrust * percentScale(config_.deviations.thrustPercent) : 0.0;
    const double stageIsp = stage1Phase ? summary.stage1SpecificImpulseSec
                                        : summary.stage2SpecificImpulseSec;
    const double actualIsp = stageIsp * percentScale(config_.deviations.specificImpulsePercent);
    const double rawMassFlow = powered ? rawActualThrust / (actualIsp * kStandardGravity) : 0.0;
    const double availablePropellant = stage1Phase ? stage1PropellantRemainingKg_
                                                   : stage2PropellantRemainingKg_;
    const double burnFraction = rawMassFlow > 0.0
        ? std::min(1.0, availablePropellant / (rawMassFlow * dt)) : 0.0;
    const double actualThrust = rawActualThrust * burnFraction;
    const double massFlow = rawMassFlow * burnFraction;
    lastActualThrust_ = actualThrust;

    const Vec3 nominalForward = nominal.referenceAttitude.rotate({1.0, 0.0, 0.0}).normalized();
    Vec3 commandDirection = nominalForward;
    lastGuidanceActive_ = guidance.enabled && powered;
    lastTerminalGuidanceActive_ = lastGuidanceActive_ && terminalWindow && !stage1Phase;
    lastGuidanceCorrectionDeg_ = {};
    if (lastGuidanceActive_) {
        const Vec3 positionError = nominal.positionEciM - positionEci_;
        const Vec3 velocityError = nominal.velocityEciMps - velocityEci_;
        Vec3 correctionAcceleration = positionError * std::max(0.0, guidance.positionGainPerSec2)
                                    + velocityError * std::max(0.0, guidance.velocityGainPerSec);
        if (lastTerminalGuidanceActive_ && targetOrbitNormalEci_.normSquared() > 0.5) {
            const double planePosition = positionEci_.dot(targetOrbitNormalEci_);
            const double planeVelocity = velocityEci_.dot(targetOrbitNormalEci_);
            correctionAcceleration += targetOrbitNormalEci_
                * (-std::max(0.0, guidance.positionGainPerSec2) * planePosition
                   -std::max(0.0, guidance.velocityGainPerSec) * planeVelocity);
        }
        const double nominalMass = std::max(1.0, nominal.massKg);
        const double referenceAcceleration = std::max(0.1, nominalStageThrust / nominalMass);
        const Vec3 unconstrained = (nominalForward + correctionAcceleration / referenceAcceleration).normalized();
        commandDirection = rotateToward(nominalForward, unconstrained,
            clamp(guidance.maxCorrectionAngleDeg, 0.0, 30.0) * kDegToRad);
        const double maximumCommandStep = clamp(guidance.maxCommandRateDegPerSec, 0.0, 20.0)
                                        * kDegToRad * dt;
        if (previousCommandDirectionEci_.normSquared() > 0.5) {
            commandDirection = rotateToward(previousCommandDirectionEci_, commandDirection,
                                            maximumCommandStep);
        }
        const Vec3 directionInNominalBody = nominal.referenceAttitude.inverse().rotate(commandDirection);
        lastGuidanceCorrectionDeg_.y = std::atan2(directionInNominalBody.z, directionInNominalBody.x)
                                     * kRadToDeg;
        lastGuidanceCorrectionDeg_.z = std::atan2(-directionInNominalBody.y, directionInNominalBody.x)
                                     * kRadToDeg;
        commandedAttitude_ = attitudeFromForward(commandDirection, nominal.referenceAttitude);
    } else {
        commandedAttitude_ = nominal.referenceAttitude;
    }
    previousCommandDirectionEci_ = commandDirection;

    const Vec3 inertiaDiagonal = interpolateInertia(summary, mass_, config_.deviations);
    const ControllerOutput controllerOutput = controller_.update(
        rotation_.attitude, commandedAttitude_, rotation_.angularRate,
        nominal.referenceAngularRateRadPerSec, dt);
    const Vec3 desiredTorque = powered ? controllerOutput.desiredTorqueBody : Vec3{};

    const double momentAuthority = std::max(1.0, actualThrust * summary.thrustLeverArmM);
    const double maxTvc = summary.maxTvcAngleDeg * kDegToRad;
    const double pitchCommand = clamp(std::asin(clamp(desiredTorque.y / momentAuthority, -1.0, 1.0)),
                                      -maxTvc, maxTvc);
    const double yawCommand = clamp(std::asin(clamp(desiredTorque.z / momentAuthority, -1.0, 1.0)),
                                    -maxTvc, maxTvc);
    const double maxTvcStep = summary.maxTvcRateDegPerSec * kDegToRad * dt;
    tvcPitchRad_ += clamp(pitchCommand - tvcPitchRad_, -maxTvcStep, maxTvcStep);
    tvcYawRad_ += clamp(yawCommand - tvcYawRad_, -maxTvcStep, maxTvcStep);
    lastSaturated_ = controllerOutput.torqueLimited
                  || std::abs(pitchCommand) >= maxTvc - 1.0e-10
                  || std::abs(yawCommand) >= maxTvc - 1.0e-10
                  || std::abs(pitchCommand - tvcPitchRad_) > 1.0e-8
                  || std::abs(yawCommand - tvcYawRad_) > 1.0e-8;

    const double bias = config_.deviations.tvcZeroBiasDeg * kDegToRad;
    const Vec3 thrustDirectionBody = Vec3{
        std::cos(tvcPitchRad_ + bias) * std::cos(tvcYawRad_),
        -std::sin(tvcYawRad_),
        std::sin(tvcPitchRad_ + bias)}.normalized();
    const Vec3 thrustBody = thrustDirectionBody * actualThrust;
    const Vec3 engineLever{-summary.thrustLeverArmM, 0.0, 0.0};
    Vec3 actuatorTorque = powered ? engineLever.cross(thrustBody) : Vec3{};
    actuatorTorque.x = powered
        ? clamp(desiredTorque.x, -summary.maxRollRcsTorqueNm, summary.maxRollRcsTorqueNm) : 0.0;
    if (powered && std::abs(actuatorTorque.x - desiredTorque.x) > 1.0e-6) lastSaturated_ = true;
    lastTorque_ = controller_.active() ? actuatorTorque : Vec3{};
    const RocketDisturbanceOutput disturbance = rocketDisturbance(
        config_.disturbances, time_, positionEci_);

    struct State { Vec3 r; Vec3 v; Quaternion q; Vec3 omega; double mass; };
    struct Derivative { Vec3 r; Vec3 v; Quaternion q; Vec3 omega; double mass; };
    const auto derivative = [&](const State& state) {
        const double altitude = state.r.norm() - kEarthRadius;
        const Vec3 atmosphereVelocity = Vec3{0.0, 0.0, kEarthRotationRate}.cross(state.r);
        const Vec3 relativeVelocity = state.v - atmosphereVelocity - disturbance.windVelocityEciMps;
        const double relativeSpeed = relativeVelocity.norm();
        const double density = densityAtAltitude(altitude)
                             * percentScale(config_.deviations.atmosphericDensityPercent);
        const double cd = (stage1Phase ? summary.stage1DragCoefficient : summary.stage2DragCoefficient)
                        * percentScale(config_.deviations.dragCoefficientPercent);
        const double area = stage1Phase ? summary.stage1ReferenceAreaM2 : summary.stage2ReferenceAreaM2;
        const double dragMagnitude = 0.5 * density * cd * area * relativeSpeed * relativeSpeed;
        const Vec3 drag = relativeSpeed > 1.0e-9 ? relativeVelocity * (-dragMagnitude / relativeSpeed) : Vec3{};
        const Vec3 thrustEci = state.q.rotate(thrustBody);
        const Vec3 acceleration = gravityEci(state.r)
            + (thrustEci + drag + disturbance.externalForceEciN) / std::max(1.0, state.mass);
        const double mdot = -massFlow;
        const Mat3 inertia = Mat3::diagonal(interpolateInertia(summary, state.mass, config_.deviations));
        const Mat3 inertiaDot = Mat3::diagonal(inertiaDerivative(summary, mdot, config_.deviations));
        return Derivative{state.v, acceleration, attitudeDerivative(state.q, state.omega),
            rigidBodyAngularAcceleration(state.omega, inertia,
                actuatorTorque + disturbance.externalTorqueBodyNm, inertiaDot), mdot};
    };
    const auto advanced = [](const State& state, const Derivative& change, double scale) {
        return State{state.r + change.r * scale, state.v + change.v * scale,
                     state.q + change.q * scale, state.omega + change.omega * scale,
                     state.mass + change.mass * scale};
    };
    State state{positionEci_, velocityEci_, rotation_.attitude, rotation_.angularRate, mass_};
    const Derivative k1 = derivative(state);
    const Derivative k2 = derivative(advanced(state, k1, dt * 0.5));
    const Derivative k3 = derivative(advanced(state, k2, dt * 0.5));
    const Derivative k4 = derivative(advanced(state, k3, dt));
    state.r += (k1.r + k2.r * 2.0 + k3.r * 2.0 + k4.r) * (dt / 6.0);
    state.v += (k1.v + k2.v * 2.0 + k3.v * 2.0 + k4.v) * (dt / 6.0);
    state.q = (state.q + (k1.q + k2.q * 2.0 + k3.q * 2.0 + k4.q) * (dt / 6.0)).normalized();
    state.omega += (k1.omega + k2.omega * 2.0 + k3.omega * 2.0 + k4.omega) * (dt / 6.0);
    state.mass += (k1.mass + 2.0 * k2.mass + 2.0 * k3.mass + k4.mass) * (dt / 6.0);
    positionEci_ = state.r;
    velocityEci_ = state.v;
    rotation_.attitude = state.q;
    rotation_.angularRate = state.omega;
    if (stage1Phase) stage1PropellantRemainingKg_ = std::max(0.0, stage1PropellantRemainingKg_ - massFlow * dt);
    else if (powered) stage2PropellantRemainingKg_ = std::max(0.0, stage2PropellantRemainingKg_ - massFlow * dt);
    const double structuralMass = (summary.payloadMassKg + summary.stage2DryMassKg
        + (stage1Separated_ ? 0.0 : summary.stage1DryMassKg)) * massScale;
    mass_ = structuralMass + stage1PropellantRemainingKg_ + stage2PropellantRemainingKg_;

    const double altitude = positionEci_.norm() - kEarthRadius;
    const Vec3 atmosphereVelocity = Vec3{0.0, 0.0, kEarthRotationRate}.cross(positionEci_);
    const double relativeSpeed = (velocityEci_ - atmosphereVelocity - disturbance.windVelocityEciMps).norm();
    const double density = densityAtAltitude(altitude) * percentScale(config_.deviations.atmosphericDensityPercent);
    const double cd = (stage1Phase ? summary.stage1DragCoefficient : summary.stage2DragCoefficient)
                    * percentScale(config_.deviations.dragCoefficientPercent);
    const double area = stage1Phase ? summary.stage1ReferenceAreaM2 : summary.stage2ReferenceAreaM2;
    lastDynamicPressure_ = 0.5 * density * relativeSpeed * relativeSpeed;
    lastDrag_ = lastDynamicPressure_ * cd * area;

    const double oldTime = time_;
    time_ += dt;
    if (time_ > mission.endTimeSec()) {
        const double coastStep = oldTime >= mission.endTimeSec() ? dt : time_ - mission.endTimeSec();
        advanceNominalCoast(coastStep);
    }
    lastPropellantDepleted_ = engineDemand && !propellantAvailable;
    const NominalTrajectoryPoint nextNominal = mission.interpolate(time_);
    if (stage2CutoffLatched_ || (nextNominal.stageId >= 3 && stage2PropellantRemainingKg_ <= 1.0e-8)) {
        orbitalCoast_ = true;
    } else if (nextNominal.stageId >= 4) {
        const double updatedEnergyError = specificOrbitalEnergy(positionEci_, velocityEci_)
                                        - targetSpecificEnergyJPerKg_;
        const bool extensionStillNeeded = guidance.enabled && guidance.terminalOrbitGuidance
            && guidance.adaptiveCutoff && !stage2CutoffLatched_
            && time_ <= nominalInsertionTimeSec_ + std::max(0.0, guidance.maxBurnExtensionSec)
            && updatedEnergyError < -std::max(0.0, guidance.specificEnergyToleranceJPerKg)
            && stage2PropellantRemainingKg_ > 1.0e-8;
        orbitalCoast_ = !extensionStillNeeded;
    }
    if (orbitalCoast_) {
        lastActualThrust_ = 0.0;
        lastTorque_ = {};
        lastGuidanceActive_ = false;
        lastTerminalGuidanceActive_ = false;
    }
    lastEngineCutoff_ = orbitalCoast_;
    recordSample(complete());
}

void RocketMissionSimulation::recordSample(bool force) {
    if (!valid()) return;
    const RocketMissionData& mission = *config_.mission;
    const bool fileExhausted = time_ > mission.endTimeSec();
    const bool coast = orbitalCoast_;
    NominalTrajectoryPoint nominal = mission.interpolate(time_);
    if (fileExhausted && nominalCoastInitialized_) {
        nominal.positionEciM = nominalCoastPositionEci_;
        nominal.velocityEciMps = nominalCoastVelocityEci_;
        nominal.altitudeM = nominal.positionEciM.norm() - kEarthRadius;
        nominal.speedMps = nominal.velocityEciMps.norm();
        nominal.thrustN = 0.0;
        nominal.stageId = 4;
        nominal.referenceAngularRateRadPerSec = {};
    }
    const Vec3 positionDifference = positionEci_ - nominal.positionEciM;
    const Vec3 velocityDifference = velocityEci_ - nominal.velocityEciMps;
    const Vec3 radialDirection = nominal.positionEciM.normalized();
    const Vec3 orbitNormal = nominal.positionEciM.cross(nominal.velocityEciMps).normalized();
    const Vec3 alongTrackDirection = orbitNormal.cross(radialDirection).normalized();
    const Quaternion referenceAttitude = lastGuidanceActive_ ? commandedAttitude_
                                                             : nominal.referenceAttitude;
    const Quaternion error = attitudeError(rotation_.attitude, referenceAttitude);
    currentSample_ = {};
    currentSample_.time = time_;
    currentSample_.position = positionEci_;
    currentSample_.velocity = velocityEci_;
    currentSample_.nominalPosition = nominal.positionEciM;
    currentSample_.nominalVelocity = nominal.velocityEciMps;
    currentSample_.attitude = rotation_.attitude;
    currentSample_.referenceAttitude = referenceAttitude;
    currentSample_.nominalReferenceAttitude = nominal.referenceAttitude;
    currentSample_.eulerDeg = rotation_.attitude.toEulerZYX() * kRadToDeg;
    currentSample_.referenceEulerDeg = referenceAttitude.toEulerZYX() * kRadToDeg;
    currentSample_.attitudeErrorDeg = error.toEulerZYX() * kRadToDeg;
    currentSample_.angularRateDegPerSec = rotation_.angularRate * kRadToDeg;
    currentSample_.controlTorque = lastTorque_;
    currentSample_.mass = mass_;
    currentSample_.altitude = positionEci_.norm() - kEarthRadius;
    currentSample_.speed = velocityEci_.norm();
    currentSample_.nominalAltitude = nominal.positionEciM.norm() - kEarthRadius;
    currentSample_.nominalSpeed = nominal.velocityEciMps.norm();
    currentSample_.nominalPitchDeg = nominal.pitchReferenceDeg;
    const Vec3 actualForwardEci = rotation_.attitude.rotate({1.0, 0.0, 0.0});
    currentSample_.actualPitchDeg = std::asin(clamp(actualForwardEci.dot(positionEci_.normalized()), -1.0, 1.0))
                                  * kRadToDeg;
    currentSample_.positionError = positionDifference.norm();
    currentSample_.crossTrackError = positionDifference.dot(orbitNormal);
    currentSample_.radialPositionError = positionDifference.dot(radialDirection);
    currentSample_.alongTrackPositionError = positionDifference.dot(alongTrackDirection);
    currentSample_.altitudeError = currentSample_.altitude - currentSample_.nominalAltitude;
    currentSample_.velocityError = velocityDifference.norm();
    currentSample_.guidanceCorrectionDeg = lastGuidanceCorrectionDeg_;
    currentSample_.propellantRemaining = stage1PropellantRemainingKg_ + stage2PropellantRemainingKg_;
    currentSample_.specificEnergyError = specificOrbitalEnergy(positionEci_, velocityEci_)
                                       - targetSpecificEnergyJPerKg_;
    currentSample_.dynamicPressure = lastDynamicPressure_;
    currentSample_.tvcPitchDeg = tvcPitchRad_ * kRadToDeg;
    currentSample_.tvcYawDeg = tvcYawRad_ * kRadToDeg;
    currentSample_.thrust = lastActualThrust_;
    currentSample_.drag = lastDrag_;
    currentSample_.stageId = coast ? 4 : nominal.stageId;
    currentSample_.missionId = mission.summary.missionId;
    currentSample_.orbitalCoast = coast;
    currentSample_.ascentTrackingActive = !coast;
    currentSample_.actuatorSaturated = lastSaturated_;
    currentSample_.controlActive = controller_.active() && lastActualThrust_ > 0.0;
    currentSample_.guidanceActive = lastGuidanceActive_;
    currentSample_.terminalGuidanceActive = lastTerminalGuidanceActive_;
    currentSample_.propellantDepleted = lastPropellantDepleted_;
    currentSample_.engineCutoff = lastEngineCutoff_;
    if (coast && !insertionCaptured_) {
        insertionSample_ = currentSample_;
        insertionCaptured_ = true;
    }
    if (coast) {
        const double unavailable = std::numeric_limits<double>::quiet_NaN();
        currentSample_.attitudeErrorDeg = {unavailable, unavailable, unavailable};
        currentSample_.positionError = unavailable;
        currentSample_.crossTrackError = unavailable;
        currentSample_.radialPositionError = unavailable;
        currentSample_.alongTrackPositionError = unavailable;
        currentSample_.altitudeError = unavailable;
        currentSample_.velocityError = unavailable;
        currentSample_.nominalPitchDeg = unavailable;
        currentSample_.actualPitchDeg = unavailable;
    }
    if (force || time_ + 1.0e-10 >= nextSampleTime_) {
        if (history_.empty() || std::abs(history_.back().time - time_) > 1.0e-9) history_.push_back(currentSample_);
        else history_.back() = currentSample_;
        nextSampleTime_ = time_ + 0.10;
    }
}

PerformanceMetrics RocketMissionSimulation::metrics() const {
    PerformanceMetrics result{};
    if (history_.empty()) return result;
    double squaredError = 0.0;
    std::size_t ascentSampleCount = 0;
    for (const auto& point : history_) {
        result.maxDynamicPressure = std::max(result.maxDynamicPressure, point.dynamicPressure);
        if (!point.ascentTrackingActive) continue;
        const double error = quaternionAngularDistance(point.attitude, point.referenceAttitude) * kRadToDeg;
        squaredError += error * error;
        ++ascentSampleCount;
        result.maxAttitudeErrorDeg = std::max(result.maxAttitudeErrorDeg, error);
        result.maxControlTorque = std::max(result.maxControlTorque, maxAbs(point.controlTorque));
        result.maxActuatorValue = std::max(result.maxActuatorValue,
            std::max(std::abs(point.tvcPitchDeg), std::abs(point.tvcYawDeg)));
        result.maxAngularRateDegPerSec = std::max(result.maxAngularRateDegPerSec,
                                                  maxAbs(point.angularRateDegPerSec));
        if (std::isfinite(point.positionError)) {
            result.maxPositionError = std::max(result.maxPositionError, point.positionError);
        }
        result.saturated = result.saturated || point.actuatorSaturated;
    }
    if (ascentSampleCount > 0) {
        result.rmsAttitudeErrorDeg = std::sqrt(squaredError / static_cast<double>(ascentSampleCount));
    }
    result.finalMass = history_.back().mass;
    const SimulationSample& terminal = insertionCaptured_ ? insertionSample_ : history_.back();
    result.terminalAltitudeError = terminal.altitude
        - config_.mission->summary.targetAltitudeKm * 1000.0;
    result.terminalVelocityError = std::abs(terminal.speed - config_.mission->summary.finalSpeedMps);
    const ClassicalOrbitElements actualElements = cartesianToClassicalElements(
        terminal.position, terminal.velocity);
    result.terminalSemiMajorAxisErrorKm = actualElements.semiMajorAxisKm
                                       - config_.mission->summary.finalSemiMajorAxisKm;
    result.terminalEccentricityError = actualElements.eccentricity
                                     - config_.mission->summary.finalEccentricity;
    result.terminalInclinationErrorDeg = actualElements.inclinationDeg
                                       - config_.mission->summary.finalInclinationDeg;
    return result;
}

} // namespace gnc
