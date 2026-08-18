#include "gnc/disturbance.hpp"

#include <cmath>
#include <cstdint>

namespace gnc {
namespace {

double deterministicNoise(std::uint32_t seed, std::uint64_t interval, std::uint32_t channel) {
    std::uint64_t state = static_cast<std::uint64_t>(seed) + 0x9E3779B97F4A7C15ull
                        + interval * 0xBF58476D1CE4E5B9ull + channel * 0x94D049BB133111EBull;
    state ^= state >> 30;
    state *= 0xBF58476D1CE4E5B9ull;
    state ^= state >> 27;
    state *= 0x94D049BB133111EBull;
    state ^= state >> 31;
    const double unit = static_cast<double>(state & 0x1FFFFFFFFFFFFFull)
                      / static_cast<double>(0x1FFFFFFFFFFFFFull);
    return (2.0 * unit - 1.0) * std::sqrt(3.0); // Unit RMS for a uniform process.
}

Vec3 randomVector(std::uint32_t seed, double timeSec, double updateInterval, double rms) {
    const double interval = std::max(1.0e-6, updateInterval);
    const auto index = static_cast<std::uint64_t>(std::floor(std::max(0.0, timeSec) / interval));
    return {rms * deterministicNoise(seed, index, 0),
            rms * deterministicNoise(seed, index, 1),
            rms * deterministicNoise(seed, index, 2)};
}

bool activeWindow(double time, double start, double duration) {
    return time >= start && time < start + std::max(0.0, duration);
}

Vec3 horizontalDirection(const Vec3& positionEci, double directionDeg) {
    const Vec3 up = positionEci.normalized();
    Vec3 east = Vec3{0.0, 0.0, 1.0}.cross(up).normalized();
    if (east.normSquared() < 1.0e-12) east = {0.0, 1.0, 0.0};
    const Vec3 north = up.cross(east).normalized();
    const double direction = directionDeg * kDegToRad;
    return east * std::cos(direction) + north * std::sin(direction);
}

} // namespace

Vec3 satelliteDisturbanceTorque(const SatelliteDisturbanceConfig& config, double timeSec) {
    Vec3 torque{};
    if (config.constantEnabled) torque += config.constantTorqueNm;
    if (config.pulseEnabled && activeWindow(timeSec, config.pulseStartSec, config.pulseDurationSec)) {
        torque += config.pulseAxis.normalized() * config.pulseMagnitudeNm;
    }
    if (config.sineEnabled) {
        const double phase = 2.0 * kPi * config.sineFrequencyHz * timeSec
                           + config.sinePhaseDeg * kDegToRad;
        torque += config.sineAxis.normalized() * (config.sineAmplitudeNm * std::sin(phase));
    }
    if (config.randomEnabled) {
        torque += randomVector(config.randomSeed, timeSec, config.randomUpdateIntervalSec,
                               std::abs(config.randomRmsNm));
    }
    return torque;
}

RocketDisturbanceOutput rocketDisturbance(const RocketDisturbanceConfig& config,
                                          double timeSec, const Vec3& positionEciM) {
    RocketDisturbanceOutput output{};
    if (config.steadyCrosswindEnabled) {
        output.windVelocityEciMps += horizontalDirection(positionEciM, config.crosswindDirectionDeg)
                                   * config.crosswindSpeedMps;
    }
    if (config.gustEnabled && activeWindow(timeSec, config.gustStartSec, config.gustDurationSec)) {
        const double u = (timeSec - config.gustStartSec) / std::max(1.0e-6, config.gustDurationSec);
        const double envelope = std::sin(kPi * clamp(u, 0.0, 1.0));
        output.windVelocityEciMps += horizontalDirection(positionEciM, config.gustDirectionDeg)
                                   * (config.gustSpeedMps * envelope);
    }
    if (config.pulseForceEnabled
        && activeWindow(timeSec, config.pulseForceStartSec, config.pulseForceDurationSec)) {
        output.externalForceEciN += config.pulseForceDirectionEci.normalized() * config.pulseForceN;
    }
    if (config.pulseTorqueEnabled
        && activeWindow(timeSec, config.pulseTorqueStartSec, config.pulseTorqueDurationSec)) {
        output.externalTorqueBodyNm += config.pulseTorqueAxisBody.normalized() * config.pulseTorqueNm;
    }
    if (config.randomEnabled) {
        output.externalForceEciN += randomVector(config.randomSeed, timeSec,
            config.randomUpdateIntervalSec, std::abs(config.randomForceRmsN));
        output.externalTorqueBodyNm += randomVector(config.randomSeed + 17u, timeSec,
            config.randomUpdateIntervalSec, std::abs(config.randomTorqueRmsNm));
    }
    return output;
}

} // namespace gnc

