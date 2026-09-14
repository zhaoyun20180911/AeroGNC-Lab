#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {

bool finiteState(const RecoveryEstimatedState& state) {
    return std::isfinite(state.time) && std::isfinite(state.massKg) && std::isfinite(state.propellantKg)
        && std::isfinite(state.positionEcefM.normSquared())
        && std::isfinite(state.velocityEcefMps.normSquared())
        && std::isfinite(state.bodyToEcef.normSquared())
        && std::isfinite(state.angularRateBodyRadPerSec.normSquared());
}

} // namespace

void RecoveryMissionManager::reset() {
    phase_ = RecoveryPhase::Initialize;
    phaseStartTimeSec_ = 0.0;
    stableTimeSec_ = 0.0;
    previousUpdateTimeSec_ = 0.0;
    failureReason_.clear();
}

void RecoveryMissionManager::transition(RecoveryPhase next, double time) {
    phase_ = next;
    phaseStartTimeSec_ = time;
    stableTimeSec_ = 0.0;
}

void RecoveryMissionManager::update(const RecoveryEstimatedState& state,
                                    const Vec3& positionNed, const Vec3& velocityNed,
                                    double attitudeErrorDeg, double rateErrorDegPerSec,
                                    const LandingPrediction& prediction, double brakingDistanceM,
                                    const RecoveryConfiguration& config) {
    const double dt = std::max(0.0, state.time - previousUpdateTimeSec_);
    previousUpdateTimeSec_ = state.time;
    if (!finiteState(state)) {
        fail(L"回收状态出现非有限数值 / Non-finite recovery state");
        return;
    }
    if (phase_ == RecoveryPhase::Initialize) {
        transition(RecoveryPhase::Flip, state.time);
        return;
    }
    const double elapsed = phaseElapsed(state.time);
    const double altitude = -positionNed.z;
    switch (phase_) {
    case RecoveryPhase::Flip:
        if (attitudeErrorDeg <= config.flipAttitudeToleranceDeg
            && rateErrorDegPerSec <= config.flipRateToleranceDegPerSec) stableTimeSec_ += dt;
        else stableTimeSec_ = 0.0;
        if ((elapsed >= config.flipDurationSec && stableTimeSec_ >= config.flipStableTimeSec)
            || elapsed >= config.flipDurationSec + 4.0) {
            transition(RecoveryPhase::Boostback, state.time);
        }
        break;
    case RecoveryPhase::Boostback:
        if ((elapsed >= config.boostbackMinDurationSec && prediction.valid
             && std::hypot(prediction.errorNedM.x, prediction.errorNedM.y)
                    <= config.boostbackPredictedErrorToleranceM)
            || elapsed >= config.boostbackMaxDurationSec) {
            transition(RecoveryPhase::Coast, state.time);
        }
        break;
    case RecoveryPhase::Coast:
        if ((velocityNed.z > 40.0 && altitude <= config.entryTriggerAltitudeM) || elapsed >= 75.0) {
            transition(RecoveryPhase::EntryBurn, state.time);
        }
        break;
    case RecoveryPhase::EntryBurn:
        if ((velocityNed.norm() <= config.entryTargetSpeedMps && elapsed >= 2.0)
            || elapsed >= config.entryMaxDurationSec || altitude <= 8500.0) {
            transition(RecoveryPhase::AeroDescent, state.time);
        }
        break;
    case RecoveryPhase::AeroDescent:
        if ((velocityNed.z > 0.0 && altitude <= brakingDistanceM + config.landingBurnSafetyMarginM)
            || altitude <= 900.0) {
            transition(RecoveryPhase::Landing, state.time);
        }
        break;
    default:
        break;
    }
}

void RecoveryMissionManager::touchdown() {
    if (phase_ != RecoveryPhase::Failed) transition(RecoveryPhase::Touchdown, previousUpdateTimeSec_);
}

void RecoveryMissionManager::fail(std::wstring reason) {
    if (phase_ == RecoveryPhase::Touchdown || phase_ == RecoveryPhase::Failed) return;
    phase_ = RecoveryPhase::Failed;
    failureReason_ = std::move(reason);
}

} // namespace gnc
