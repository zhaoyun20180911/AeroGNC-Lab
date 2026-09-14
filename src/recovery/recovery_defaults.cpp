#include "gnc/recovery.hpp"

#include <cmath>

namespace gnc {

LandingPrediction LandingPointPredictor::predict(const RecoveryEstimatedState& state,
                                                 const NedFrame& frame,
                                                 const RecoveryConfiguration&) const {
    constexpr double gravity = 9.80665;
    LandingPrediction result{};
    const Vec3 position = ecefPositionToNed(state.positionEcefM, frame);
    const Vec3 velocity = ecefVectorToNed(state.velocityEcefMps, frame);
    const double discriminant = velocity.z * velocity.z - 2.0 * gravity * position.z;
    if (!std::isfinite(discriminant) || discriminant < 0.0) return result;
    double time = (-velocity.z + std::sqrt(discriminant)) / gravity;
    if (time <= 0.0) time = (-velocity.z - std::sqrt(discriminant)) / gravity;
    if (!std::isfinite(time) || time <= 0.0) return result;
    time = clamp(time, 0.0, 240.0);
    const double horizontalFactor = 0.72 + 0.28 * std::exp(-time / 55.0);
    result.touchdownPositionNedM = {position.x + velocity.x * time * horizontalFactor,
                                    position.y + velocity.y * time * horizontalFactor, 0.0};
    result.touchdownVelocityNedMps = {velocity.x * horizontalFactor,
                                      velocity.y * horizontalFactor,
                                      velocity.z + gravity * time};
    result.errorNedM = result.touchdownPositionNedM;
    result.timeToGroundSec = time;
    result.valid = std::isfinite(result.errorNedM.normSquared());
    return result;
}

RecoveryConfiguration makeDefaultRecoveryConfiguration() {
    RecoveryConfiguration config{};
    const NedFrame frame = makeLandingNedFrame(config.landingSite);
    // Representative post-separation RTLS handoff: the stage is still climbing
    // at about 80 km altitude with substantial downrange and horizontal speed.
    const Vec3 positionNed{20000.0, 0.0, -80000.0};
    const Vec3 velocityNed{600.0, 0.0, -150.0};
    config.initialState.positionEcefM = nedPositionToEcef(positionNed, frame);
    config.initialState.velocityEcefMps = nedVectorToEcef(velocityNed, frame);
    config.initialState.bodyToEcef = attitudeFromBodyX(config.initialState.velocityEcefMps, frame.eastEcef);
    config.initialState.angularRateBodyRadPerSec = {0.0, 0.15 * kDegToRad, 0.0};
    config.initialState.massTotalKg = 90000.0;
    config.initialState.propellantMassKg = 62000.0;
    config.attitudeControl.mode = ControlMode::Default;
    return config;
}

} // namespace gnc
