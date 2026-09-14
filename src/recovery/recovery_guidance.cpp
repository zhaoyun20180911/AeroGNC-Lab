#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {
constexpr double kGravity = 9.80665;

double smoothStep(double value) {
    const double u = clamp(value, 0.0, 1.0);
    return u * u * (3.0 - 2.0 * u);
}

Vec3 clampHorizontal(const Vec3& acceleration, double limit) {
    Vec3 result = acceleration;
    const double horizontal = std::hypot(result.x, result.y);
    if (horizontal > limit && horizontal > 1.0e-12) {
        result.x *= limit / horizontal;
        result.y *= limit / horizontal;
    }
    return result;
}

} // namespace

void RecoverySimulation::updateGuidance(const RecoveryEstimatedState& estimated,
                                        const Vec3& positionNed, const Vec3& velocityNed,
                                        const LandingPrediction& prediction) {
    GuidanceCommand command{};
    command.desiredAttitude = guidance_.desiredAttitude.normSquared() > 0.5
        ? guidance_.desiredAttitude : estimated.bodyToEcef;
    const RecoveryPhase phase = mission_.phase();
    const double elapsed = mission_.phaseElapsed(estimated.time);

    const auto setPoweredCommand = [&](const Vec3& netAccelerationNed, double requestedThrustN) {
        const Vec3 thrustAccelerationNed = netAccelerationNed - Vec3{0.0, 0.0, kGravity};
        command.desiredAccelerationNedMps2 = netAccelerationNed;
        command.desiredThrustN = clamp(requestedThrustN, 0.0,
            config_.vehicle.maxThrustN * config_.vehicle.maxThrottle);
        if (thrustAccelerationNed.normSquared() > 1.0e-12) {
            command.desiredThrustDirectionEcef = nedVectorToEcef(
                thrustAccelerationNed.normalized(), frame_);
            command.desiredAttitude = attitudeFromBodyX(command.desiredThrustDirectionEcef,
                                                        frame_.eastEcef);
        }
        command.engineOn = true;
    };

    if (phase == RecoveryPhase::Initialize || phase == RecoveryPhase::Flip) {
        const double u = smoothStep(elapsed / std::max(0.1, config_.flipDurationSec));
        command.desiredAttitude = slerp(flipStartAttitude_, flipTargetAttitude_, u);
    } else if (phase == RecoveryPhase::Boostback) {
        const double remaining = std::max(4.0, config_.boostbackMaxDurationSec - elapsed);
        const Vec3 targetError = prediction.valid ? prediction.errorNedM : positionNed;
        Vec3 netAcceleration{};
        netAcceleration.x = -2.0 * targetError.x / (remaining * remaining)
                          - 0.55 * velocityNed.x / remaining;
        netAcceleration.y = -2.0 * targetError.y / (remaining * remaining)
                          - 0.55 * velocityNed.y / remaining;
        netAcceleration = clampHorizontal(netAcceleration, 15.0);
        netAcceleration.z = clamp((-25.0 - velocityNed.z) / remaining, -4.0, 3.0);
        setPoweredCommand(netAcceleration, config_.vehicle.maxThrustN * 0.88);
    } else if (phase == RecoveryPhase::Coast) {
        const Vec3 retrograde = (-estimated.velocityEcefMps).normalized();
        command.desiredAttitude = attitudeFromBodyX(retrograde, frame_.eastEcef);
    } else if (phase == RecoveryPhase::EntryBurn) {
        const Vec3 retrogradeNed = (-velocityNed).normalized();
        const double available = config_.vehicle.maxThrustN * 0.82
                               / std::max(config_.vehicle.dryMassKg, estimated.massKg);
        const Vec3 thrustAcceleration = retrogradeNed * available;
        setPoweredCommand(Vec3{0.0, 0.0, kGravity} + thrustAcceleration,
                          config_.vehicle.maxThrustN * 0.82);
    } else if (phase == RecoveryPhase::AeroDescent) {
        const Vec3 targetError = prediction.valid ? prediction.errorNedM : positionNed;
        const double finLimit = config_.vehicle.gridFinLimitDeg * kDegToRad;
        const double north = clamp(-0.000060 * targetError.x - 0.0015 * velocityNed.x,
                                   -finLimit, finLimit);
        const double east = clamp(-0.000060 * targetError.y - 0.0015 * velocityNed.y,
                                  -finLimit, finLimit);
        command.gridFinCommandRad = {north, east, north, east};
        command.desiredAttitude = attitudeFromBodyX(
            (-estimated.velocityEcefMps).normalized(), frame_.eastEcef);
    } else if (phase == RecoveryPhase::Landing) {
        const LandingGuidanceResult landing = landingGuidance_.update(estimated, frame_, config_);
        lastSolverStatus_ = landing.solverStatus;
        lastSolverTimeMs_ = landing.solveTimeMs;
        lastSolverFallback_ = landing.fallbackUsed;
        ++solverCalls_;
        if (landing.solverStatus == RecoverySolverStatus::Solved) ++solverSuccesses_;
        totalSolverTimeMs_ += landing.solveTimeMs;
        maxSolverTimeMs_ = std::max(maxSolverTimeMs_, landing.solveTimeMs);
        Vec3 netAcceleration = clampHorizontal(landing.netAccelerationNedMps2,
                                                config_.maxHorizontalAccelerationMps2);
        const Vec3 thrustAcceleration = netAcceleration - Vec3{0.0, 0.0, kGravity};
        setPoweredCommand(netAcceleration, estimated.massKg * thrustAcceleration.norm());
    } else {
        command.engineCutoff = true;
    }
    guidance_ = command;
}

} // namespace gnc
