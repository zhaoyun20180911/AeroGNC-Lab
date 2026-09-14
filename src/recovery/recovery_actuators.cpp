#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {
double moveToward(double current, double target, double maximumStep) {
    return current + clamp(target - current, -maximumStep, maximumStep);
}
}

void RecoveryControlAllocator::reset() {
    tvcPitchRad_ = 0.0;
    tvcYawRad_ = 0.0;
    gridFinRad_.fill(0.0);
}

RecoveryActuatorCommand RecoveryControlAllocator::update(
    const GuidanceCommand& guidance, const Vec3& desiredMomentBodyNm,
    const RecoveryEstimatedState& state, RecoveryPhase phase, double dt,
    const RecoveryConfiguration& config) {
    RecoveryActuatorCommand output{};
    const auto& vehicle = config.vehicle;
    const bool powered = phase == RecoveryPhase::Boostback || phase == RecoveryPhase::EntryBurn
                      || phase == RecoveryPhase::Landing;
    output.engineOn = powered && guidance.engineOn && state.propellantKg > 1.0e-9;
    output.requestedThrustN = output.engineOn ? guidance.desiredThrustN : 0.0;
    const Vec3 desiredBody = state.bodyToEcef.inverse().rotate(
        guidance.desiredThrustDirectionEcef).normalized();
    const double tvcLimit = vehicle.tvcLimitDeg * kDegToRad;
    const double targetPitch = output.engineOn
        ? clamp(-std::atan2(desiredBody.z, std::max(1.0e-9, desiredBody.x)), -tvcLimit, tvcLimit) : 0.0;
    const double targetYaw = output.engineOn
        ? clamp(std::atan2(desiredBody.y, std::max(1.0e-9, desiredBody.x)), -tvcLimit, tvcLimit) : 0.0;
    const double tvcStep = vehicle.tvcRateDegPerSec * kDegToRad * dt;
    tvcPitchRad_ = moveToward(tvcPitchRad_, targetPitch, tvcStep);
    tvcYawRad_ = moveToward(tvcYawRad_, targetYaw, tvcStep);
    output.tvcPitchRad = tvcPitchRad_;
    output.tvcYawRad = tvcYawRad_;

    const double finLimit = vehicle.gridFinLimitDeg * kDegToRad;
    const double finStep = vehicle.gridFinRateDegPerSec * kDegToRad * dt;
    const bool finsEnabled = phase == RecoveryPhase::AeroDescent || phase == RecoveryPhase::EntryBurn;
    for (int i = 0; i < 4; ++i) {
        const double target = finsEnabled ? clamp(guidance.gridFinCommandRad[i], -finLimit, finLimit) : 0.0;
        gridFinRad_[i] = moveToward(gridFinRad_[i], target, finStep);
    }
    output.gridFinRad = gridFinRad_;

    const double tvcAuthority = output.engineOn
        ? output.requestedThrustN * vehicle.thrustLeverArmM * std::sin(tvcLimit) : 0.0;
    output.appliedMomentBodyNm.x = clamp(desiredMomentBodyNm.x,
        -vehicle.maxRcsTorqueNm, vehicle.maxRcsTorqueNm);
    output.appliedMomentBodyNm.y = clamp(desiredMomentBodyNm.y,
        -(vehicle.maxRcsTorqueNm + tvcAuthority), vehicle.maxRcsTorqueNm + tvcAuthority);
    output.appliedMomentBodyNm.z = clamp(desiredMomentBodyNm.z,
        -(vehicle.maxRcsTorqueNm + tvcAuthority), vehicle.maxRcsTorqueNm + tvcAuthority);
    return output;
}

void RecoveryEngineModel::reset() {
    onCommandTimeSec_ = 0.0;
    offCommandTimeSec_ = 0.0;
    actualThrustN_ = 0.0;
}

double RecoveryEngineModel::update(bool commandOn, double requestedThrustN,
                                   double propellantKg, double dt,
                                   const RecoveryVehicleParameters& vehicle) {
    if (commandOn && propellantKg > 1.0e-9) {
        onCommandTimeSec_ += dt;
        offCommandTimeSec_ = 0.0;
    } else {
        offCommandTimeSec_ += dt;
        onCommandTimeSec_ = 0.0;
    }
    double target{};
    if (commandOn && propellantKg > 1.0e-9 && onCommandTimeSec_ >= vehicle.ignitionDelaySec) {
        target = clamp(requestedThrustN, vehicle.maxThrustN * vehicle.minThrottle,
                       vehicle.maxThrustN * vehicle.maxThrottle);
    } else if (!commandOn && offCommandTimeSec_ < vehicle.shutdownDelaySec) {
        target = actualThrustN_;
    }
    target *= std::max(0.0, 1.0 + vehicle.thrustBiasPercent / 100.0);
    actualThrustN_ += (target - actualThrustN_)
                    * clamp(dt / std::max(1.0e-4, vehicle.thrustLagSec), 0.0, 1.0);
    if (propellantKg <= 1.0e-9) actualThrustN_ = 0.0;
    return std::max(0.0, actualThrustN_);
}

} // namespace gnc
