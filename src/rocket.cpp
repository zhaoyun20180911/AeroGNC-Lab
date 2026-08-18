#include "gnc/simulation.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {

constexpr double kEarthRadius = 6378137.0;
constexpr double kSeaLevelGravity = 9.80665;
constexpr double kSeaLevelDensity = 1.225;
constexpr double kAtmosphereScaleHeight = 8500.0;

Quaternion verticalLaunchAttitude() {
    // Body +X is the longitudinal/nose axis, +Y is starboard, +Z completes the RHS.
    // At launch +X_B points up (+Z_L), +Y_B aligns +Y_L, and +Z_B points downrange-negative.
    Mat3 bodyToLocal{};
    bodyToLocal.m[0][0] = 0.0;  bodyToLocal.m[1][0] = 0.0; bodyToLocal.m[2][0] = 1.0;
    bodyToLocal.m[0][1] = 0.0;  bodyToLocal.m[1][1] = 1.0; bodyToLocal.m[2][1] = 0.0;
    bodyToLocal.m[0][2] = -1.0; bodyToLocal.m[1][2] = 0.0; bodyToLocal.m[2][2] = 0.0;
    return Quaternion::fromRotationMatrix(bodyToLocal);
}

double smoothStep(double value) {
    const double u = clamp(value, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

double smoothStepDerivative(double value) {
    const double u = clamp(value, 0.0, 1.0);
    return (value <= 0.0 || value >= 1.0) ? 0.0 : 6.0 * u * (1.0 - u);
}

struct GuidanceOutput {
    Quaternion attitude;
    Vec3 angularRateBody;
    double pitchRadians{};
};

GuidanceOutput rocketGuidance(const RocketConfig& config, double time) {
    const double span = std::max(0.1, config.pitchEndSec - config.pitchStartSec);
    const double normalizedTime = (time - config.pitchStartSec) / span;
    const double finalPitch = config.finalPitchDeg * kDegToRad;
    const double pitch = finalPitch * smoothStep(normalizedTime);
    const double pitchRate = finalPitch * smoothStepDerivative(normalizedTime) / span;
    const Quaternion target = (verticalLaunchAttitude()
        * Quaternion::fromAxisAngle({0.0, 1.0, 0.0}, pitch)).normalized();
    return {target, {0.0, pitchRate, 0.0}, pitch};
}

double fuelFraction(const RocketConfig& config, double mass) {
    return clamp((mass - config.dryMassKg) / std::max(1.0, config.wetMassKg - config.dryMassKg), 0.0, 1.0);
}

Vec3 interpolateInertiaDiagonal(const RocketConfig& config, double mass) {
    const double fraction = fuelFraction(config, mass);
    return config.dryInertiaKgM2 + (config.wetInertiaKgM2 - config.dryInertiaKgM2) * fraction;
}

Vec3 inertiaDerivativeDiagonal(const RocketConfig& config, double massDerivative) {
    const double massRange = std::max(1.0, config.wetMassKg - config.dryMassKg);
    return (config.wetInertiaKgM2 - config.dryInertiaKgM2) * (massDerivative / massRange);
}

double atmosphereDensity(double altitude) {
    return altitude < 120000.0
        ? kSeaLevelDensity * std::exp(-std::max(0.0, altitude) / kAtmosphereScaleHeight)
        : 0.0;
}

double gravityMagnitude(double altitude) {
    const double ratio = kEarthRadius / (kEarthRadius + std::max(0.0, altitude));
    return kSeaLevelGravity * ratio * ratio;
}

double maxAbs(const Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

PerformanceMetrics rocketMetrics(const std::vector<SimulationSample>& samples) {
    PerformanceMetrics metrics{};
    if (samples.empty()) {
        return metrics;
    }
    double squaredError{};
    std::size_t lastOutside{};
    bool outside = false;
    for (std::size_t index = 0; index < samples.size(); ++index) {
        const SimulationSample& sample = samples[index];
        const double error = quaternionAngularDistance(sample.attitude, sample.referenceAttitude) * kRadToDeg;
        metrics.maxAttitudeErrorDeg = std::max(metrics.maxAttitudeErrorDeg, error);
        squaredError += error * error;
        metrics.maxControlTorque = std::max(metrics.maxControlTorque, maxAbs(sample.controlTorque));
        metrics.maxActuatorValue = std::max(metrics.maxActuatorValue,
            std::max(std::abs(sample.tvcPitchDeg), std::abs(sample.tvcYawDeg)));
        metrics.maxAngularRateDegPerSec = std::max(metrics.maxAngularRateDegPerSec,
                                                   maxAbs(sample.angularRateDegPerSec));
        metrics.maxDynamicPressure = std::max(metrics.maxDynamicPressure, sample.dynamicPressure);
        metrics.saturated = metrics.saturated || sample.actuatorSaturated;
        if (error > 1.0) {
            outside = true;
            lastOutside = index;
        }
    }
    metrics.rmsAttitudeErrorDeg = std::sqrt(squaredError / static_cast<double>(samples.size()));
    if (!outside) {
        metrics.settlingTimeSec = 0.0;
    } else if (lastOutside + 1 < samples.size()) {
        metrics.settlingTimeSec = samples[lastOutside + 1].time;
    }
    metrics.finalMass = samples.back().mass;
    return metrics;
}

} // namespace

RocketSimulation::RocketSimulation(const RocketConfig& config) {
    reset(config);
}

void RocketSimulation::reset(const RocketConfig& config) {
    config_ = config;
    config_.wetMassKg = std::max(100.0, config_.wetMassKg);
    config_.dryMassKg = clamp(config_.dryMassKg, 1.0, config_.wetMassKg);
    config_.thrustN = std::max(0.0, config_.thrustN);
    config_.specificImpulseSec = std::max(1.0, config_.specificImpulseSec);
    config_.durationSec = clamp(config_.durationSec, 1.0, 1200.0);
    config_.pitchStartSec = std::max(0.0, config_.pitchStartSec);
    config_.pitchEndSec = std::max(config_.pitchStartSec + 0.1, config_.pitchEndSec);
    config_.maxTvcAngleDeg = clamp(config_.maxTvcAngleDeg, 0.1, 20.0);
    config_.maxTvcRateDegPerSec = clamp(config_.maxTvcRateDegPerSec, 0.1, 90.0);
    config_.thrustLeverArmM = std::max(0.1, config_.thrustLeverArmM);

    time_ = 0.0;
    nextSampleTime_ = 0.0;
    positionLocal_ = {};
    velocityLocal_ = {};
    mass_ = config_.wetMassKg;
    tvcPitchRad_ = 0.0;
    tvcYawRad_ = 0.0;
    lastTorque_ = {};
    lastDynamicPressure_ = 0.0;
    lastDrag_ = 0.0;
    lastSaturated_ = false;
    const GuidanceOutput guidance = rocketGuidance(config_, 0.0);
    const Quaternion offset = Quaternion::fromEulerZYX(
        config_.initialErrorDeg.x * kDegToRad,
        config_.initialErrorDeg.y * kDegToRad,
        config_.initialErrorDeg.z * kDegToRad);
    rotation_.attitude = (guidance.attitude * offset).normalized();
    rotation_.angularRate = guidance.angularRateBody + config_.initialRateDegPerSec * kDegToRad;
    history_.clear();
    recordSample(true);
}

void RocketSimulation::step(double dt) {
    if (dt <= 0.0 || complete()) {
        return;
    }
    dt = std::min(dt, config_.durationSec - time_);
    const GuidanceOutput guidance = rocketGuidance(config_, time_);
    const Quaternion error = attitudeError(rotation_.attitude, guidance.attitude);
    const Vec3 rateError = rotation_.angularRate - guidance.angularRateBody;
    const Vec3 inertiaDiagonal = interpolateInertiaDiagonal(config_, mass_);
    const double fraction = fuelFraction(config_, mass_);
    const double naturalFrequency = 0.62 + 0.16 * (1.0 - fraction);
    constexpr double dampingRatio = 0.95;
    const Vec3 proportionalGain = inertiaDiagonal * (naturalFrequency * naturalFrequency);
    const Vec3 derivativeGain = inertiaDiagonal * (2.0 * dampingRatio * naturalFrequency);
    const Vec3 desiredTorque{
        2.0 * proportionalGain.x * error.x - derivativeGain.x * rateError.x,
        2.0 * proportionalGain.y * error.y - derivativeGain.y * rateError.y,
        2.0 * proportionalGain.z * error.z - derivativeGain.z * rateError.z
    };

    const bool engineOn = mass_ > config_.dryMassKg + 1.0e-6 && config_.thrustN > 0.0;
    const double thrust = engineOn ? config_.thrustN : 0.0;
    const double momentAuthority = std::max(1.0, thrust * config_.thrustLeverArmM);
    const double maxTvc = config_.maxTvcAngleDeg * kDegToRad;
    const double pitchCommand = clamp(std::asin(clamp(desiredTorque.y / momentAuthority, -1.0, 1.0)),
                                      -maxTvc, maxTvc);
    const double yawCommand = clamp(std::asin(clamp(desiredTorque.z / momentAuthority, -1.0, 1.0)),
                                    -maxTvc, maxTvc);
    const double maxRateStep = config_.maxTvcRateDegPerSec * kDegToRad * dt;
    const double pitchChange = clamp(pitchCommand - tvcPitchRad_, -maxRateStep, maxRateStep);
    const double yawChange = clamp(yawCommand - tvcYawRad_, -maxRateStep, maxRateStep);
    tvcPitchRad_ += pitchChange;
    tvcYawRad_ += yawChange;
    lastSaturated_ = std::abs(pitchCommand) >= maxTvc - 1.0e-9
                  || std::abs(yawCommand) >= maxTvc - 1.0e-9
                  || std::abs(pitchCommand - tvcPitchRad_) > 1.0e-8
                  || std::abs(yawCommand - tvcYawRad_) > 1.0e-8;

    const Vec3 thrustDirectionBody = Vec3{
        std::cos(tvcPitchRad_) * std::cos(tvcYawRad_),
        -std::sin(tvcYawRad_),
        std::sin(tvcPitchRad_)
    }.normalized();
    const Vec3 thrustBody = thrustDirectionBody * thrust;
    const Vec3 leverBody{-config_.thrustLeverArmM, 0.0, 0.0};
    Vec3 actuatorTorque = leverBody.cross(thrustBody);
    const double rollAuthority = 0.055 * momentAuthority;
    actuatorTorque.x = clamp(desiredTorque.x, -rollAuthority, rollAuthority);
    if (std::abs(actuatorTorque.x - desiredTorque.x) > 1.0e-6) {
        lastSaturated_ = true;
    }
    Vec3 disturbanceTorque{};
    if (config_.disturbanceEnabled) {
        const double pulse = (time_ > 32.0 && time_ < 34.0) ? 1.0 : 0.0;
        disturbanceTorque = {
            6000.0 * std::sin(0.35 * time_),
            9000.0 + 65000.0 * pulse,
            -7000.0 + 45000.0 * pulse
        };
    }
    lastTorque_ = actuatorTorque;

    struct State {
        Vec3 position;
        Vec3 velocity;
        Quaternion attitude;
        Vec3 angularRate;
        double mass{};
    };
    struct Derivative {
        Vec3 position;
        Vec3 velocity;
        Quaternion attitude;
        Vec3 angularRate;
        double mass{};
    };

    const double nominalMassFlow = engineOn ? thrust / (config_.specificImpulseSec * kSeaLevelGravity) : 0.0;
    const Vec3 lateralDisturbanceLocal = config_.disturbanceEnabled
        ? Vec3{0.0, 4500.0 * std::sin(0.21 * time_), 0.0} : Vec3{};
    const auto derivative = [&](const State& state) {
        const double altitude = std::max(0.0, state.position.z);
        const double density = atmosphereDensity(altitude);
        const double speed = state.velocity.norm();
        const double dragMagnitude = 0.5 * density * config_.dragCoefficient
                                   * config_.referenceAreaM2 * speed * speed;
        const Vec3 dragForce = speed > 1.0e-8 ? state.velocity * (-dragMagnitude / speed) : Vec3{};
        const Vec3 thrustLocal = state.attitude.rotate(thrustBody);
        const Vec3 gravity{0.0, 0.0, -gravityMagnitude(altitude)};
        const Vec3 acceleration = gravity
            + (thrustLocal + dragForce + lateralDisturbanceLocal) / std::max(1.0, state.mass);
        const double massDerivative = state.mass > config_.dryMassKg ? -nominalMassFlow : 0.0;
        const Mat3 inertia = Mat3::diagonal(interpolateInertiaDiagonal(config_, state.mass));
        const Mat3 inertiaDerivative = Mat3::diagonal(inertiaDerivativeDiagonal(config_, massDerivative));
        return Derivative{
            state.velocity,
            acceleration,
            attitudeDerivative(state.attitude, state.angularRate),
            rigidBodyAngularAcceleration(state.angularRate, inertia,
                                          actuatorTorque + disturbanceTorque, inertiaDerivative),
            massDerivative
        };
    };
    const auto advanced = [](const State& state, const Derivative& change, double scale) {
        return State{
            state.position + change.position * scale,
            state.velocity + change.velocity * scale,
            state.attitude + change.attitude * scale,
            state.angularRate + change.angularRate * scale,
            state.mass + change.mass * scale
        };
    };
    State state{positionLocal_, velocityLocal_, rotation_.attitude, rotation_.angularRate, mass_};
    const Derivative k1 = derivative(state);
    const Derivative k2 = derivative(advanced(state, k1, 0.5 * dt));
    const Derivative k3 = derivative(advanced(state, k2, 0.5 * dt));
    const Derivative k4 = derivative(advanced(state, k3, dt));
    state.position += (k1.position + k2.position * 2.0 + k3.position * 2.0 + k4.position) * (dt / 6.0);
    state.velocity += (k1.velocity + k2.velocity * 2.0 + k3.velocity * 2.0 + k4.velocity) * (dt / 6.0);
    state.attitude = (state.attitude
        + (k1.attitude + k2.attitude * 2.0 + k3.attitude * 2.0 + k4.attitude) * (dt / 6.0)).normalized();
    state.angularRate += (k1.angularRate + k2.angularRate * 2.0 + k3.angularRate * 2.0 + k4.angularRate) * (dt / 6.0);
    state.mass += (k1.mass + 2.0 * k2.mass + 2.0 * k3.mass + k4.mass) * (dt / 6.0);

    positionLocal_ = state.position;
    velocityLocal_ = state.velocity;
    rotation_.attitude = state.attitude;
    rotation_.angularRate = state.angularRate;
    mass_ = std::max(config_.dryMassKg, state.mass);
    time_ += dt;
    const double density = atmosphereDensity(std::max(0.0, positionLocal_.z));
    lastDynamicPressure_ = 0.5 * density * velocityLocal_.normSquared();
    lastDrag_ = lastDynamicPressure_ * config_.dragCoefficient * config_.referenceAreaM2;
    recordSample(complete());
}

void RocketSimulation::recordSample(bool force) {
    const GuidanceOutput guidance = rocketGuidance(config_, time_);
    const Quaternion error = attitudeError(rotation_.attitude, guidance.attitude);
    const Quaternion relativeToVertical = verticalLaunchAttitude().inverse() * rotation_.attitude;
    const Quaternion referenceRelativeToVertical = verticalLaunchAttitude().inverse() * guidance.attitude;
    currentSample_ = {};
    currentSample_.time = time_;
    currentSample_.position = positionLocal_;
    currentSample_.velocity = velocityLocal_;
    currentSample_.attitude = rotation_.attitude;
    currentSample_.referenceAttitude = guidance.attitude;
    currentSample_.eulerDeg = relativeToVertical.toEulerZYX() * kRadToDeg;
    currentSample_.referenceEulerDeg = referenceRelativeToVertical.toEulerZYX() * kRadToDeg;
    currentSample_.attitudeErrorDeg = error.toEulerZYX() * kRadToDeg;
    currentSample_.angularRateDegPerSec = rotation_.angularRate * kRadToDeg;
    currentSample_.controlTorque = lastTorque_;
    currentSample_.mass = mass_;
    currentSample_.altitude = positionLocal_.z;
    currentSample_.speed = velocityLocal_.norm();
    currentSample_.dynamicPressure = lastDynamicPressure_;
    currentSample_.tvcPitchDeg = tvcPitchRad_ * kRadToDeg;
    currentSample_.tvcYawDeg = tvcYawRad_ * kRadToDeg;
    currentSample_.thrust = mass_ > config_.dryMassKg + 1.0e-6 ? config_.thrustN : 0.0;
    currentSample_.drag = lastDrag_;
    currentSample_.actuatorSaturated = lastSaturated_;

    if (force || time_ + 1.0e-10 >= nextSampleTime_) {
        if (history_.empty() || std::abs(history_.back().time - time_) > 1.0e-9) {
            history_.push_back(currentSample_);
        } else {
            history_.back() = currentSample_;
        }
        nextSampleTime_ = time_ + 0.05;
    }
}

PerformanceMetrics RocketSimulation::metrics() const {
    return rocketMetrics(history_);
}

} // namespace gnc

