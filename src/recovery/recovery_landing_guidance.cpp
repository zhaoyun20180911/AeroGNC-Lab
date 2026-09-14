#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {
constexpr double kGravity = 9.80665;
double component(const Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}
void setComponent(Vec3& value, int axis, double v) {
    if (axis == 0) value.x = v;
    else if (axis == 1) value.y = v;
    else value.z = v;
}
}

QpProblem ConvexLandingProblem::build(const Vec3& position, const Vec3& velocity,
                                      const Vec3& previousAcceleration, double massKg,
                                      const RecoveryConfiguration& config) const {
    const int horizon = std::clamp(config.landingQpHorizon, 4, 80);
    const int variables = 3 * horizon;
    const double dt = clamp(config.landingQpDtSec, 0.05, 2.0);
    QpProblem problem{};
    problem.variableCount = variables;
    problem.hessian.assign(static_cast<std::size_t>(variables) * variables, 0.0);
    problem.gradient.assign(variables, 0.0);
    problem.lowerBound.assign(variables, 0.0);
    problem.upperBound.assign(variables, 0.0);
    problem.maxIterations = std::clamp(config.solverMaxIterations, 10, 1000);
    problem.tolerance = std::max(1.0e-9, config.solverTolerance);

    std::vector<double> positionMap(horizon);
    std::vector<double> velocityMap(horizon, dt);
    for (int k = 0; k < horizon; ++k) {
        positionMap[k] = (static_cast<double>(horizon - k) - 0.5) * dt * dt;
    }
    const double targetVelocity[3]{0.0, 0.0, 2.0};
    const double maximumUpwardThrustAcceleration = config.vehicle.maxThrustN
        * config.vehicle.maxThrottle / std::max(config.vehicle.dryMassKg, massKg);
    const double minimumUpwardThrustAcceleration = config.vehicle.maxThrustN
        * config.vehicle.minThrottle / std::max(config.vehicle.dryMassKg, massKg);

    for (int axis = 0; axis < 3; ++axis) {
        const int offset = axis * horizon;
        const double terminalPositionConstant = component(position, axis)
            + horizon * dt * component(velocity, axis);
        const double terminalVelocityConstant = component(velocity, axis) - targetVelocity[axis];
        for (int i = 0; i < horizon; ++i) {
            const int row = offset + i;
            problem.gradient[row] += 2.0 * config.weightTerminalPosition
                * terminalPositionConstant * positionMap[i]
                + 2.0 * config.weightTerminalVelocity * terminalVelocityConstant * velocityMap[i];
            for (int j = 0; j < horizon; ++j) {
                const int column = offset + j;
                problem.hessian[static_cast<std::size_t>(row) * variables + column]
                    += 2.0 * config.weightTerminalPosition * positionMap[i] * positionMap[j]
                     + 2.0 * config.weightTerminalVelocity * velocityMap[i] * velocityMap[j];
            }
            problem.hessian[static_cast<std::size_t>(row) * variables + row]
                += 2.0 * config.weightControlEffort;
            if (axis < 2) {
                problem.lowerBound[row] = -config.maxHorizontalAccelerationMps2;
                problem.upperBound[row] = config.maxHorizontalAccelerationMps2;
            } else {
                problem.lowerBound[row] = kGravity - maximumUpwardThrustAcceleration;
                problem.upperBound[row] = kGravity - minimumUpwardThrustAcceleration;
            }
        }

        const double smooth = config.weightControlSmoothness;
        for (int k = 0; k < horizon; ++k) {
            const int index = offset + k;
            if (k == 0) {
                problem.hessian[static_cast<std::size_t>(index) * variables + index] += 2.0 * smooth;
                problem.gradient[index] -= 2.0 * smooth * component(previousAcceleration, axis);
            } else {
                const int previous = index - 1;
                problem.hessian[static_cast<std::size_t>(index) * variables + index] += 2.0 * smooth;
                problem.hessian[static_cast<std::size_t>(previous) * variables + previous] += 2.0 * smooth;
                problem.hessian[static_cast<std::size_t>(index) * variables + previous] -= 2.0 * smooth;
                problem.hessian[static_cast<std::size_t>(previous) * variables + index] -= 2.0 * smooth;
            }
        }
    }
    return problem;
}

LandingGuidance::LandingGuidance() : solver_(std::make_unique<ProjectedGradientQpSolver>()) {}

void LandingGuidance::reset() {
    warmStart_.clear();
    previousAccelerationNedMps2_ = {};
}

Vec3 LandingGuidance::fallback(const Vec3& position, const Vec3& velocity, double massKg,
                               const RecoveryConfiguration& config) const {
    Vec3 acceleration{};
    acceleration.x = clamp(-0.012 * position.x - 0.30 * velocity.x,
                           -config.maxHorizontalAccelerationMps2, config.maxHorizontalAccelerationMps2);
    acceleration.y = clamp(-0.012 * position.y - 0.30 * velocity.y,
                           -config.maxHorizontalAccelerationMps2, config.maxHorizontalAccelerationMps2);
    const double lower = kGravity - config.vehicle.maxThrustN * config.vehicle.maxThrottle
                                    / std::max(config.vehicle.dryMassKg, massKg);
    const double upper = kGravity - config.vehicle.maxThrustN * config.vehicle.minThrottle
                                    / std::max(config.vehicle.dryMassKg, massKg);
    acceleration.z = clamp(-0.010 * position.z - 0.50 * (velocity.z - 1.5), lower, upper);
    return acceleration;
}

LandingGuidanceResult LandingGuidance::update(const RecoveryEstimatedState& state,
                                              const NedFrame& frame,
                                              const RecoveryConfiguration& config) {
    const Vec3 position = ecefPositionToNed(state.positionEcefM, frame);
    const Vec3 velocity = ecefVectorToNed(state.velocityEcefMps, frame);
    RecoveryConfiguration rollingConfig = config;
    const int horizon = std::clamp(config.landingQpHorizon, 4, 80);
    const double altitude = std::max(0.0, -position.z);
    const double maximumNetDeceleration = std::max(0.5,
        config.vehicle.maxThrustN * config.vehicle.maxThrottle
            / std::max(config.vehicle.dryMassKg, state.massKg) - kGravity);
    const double kinematicTime = altitude / std::max(2.0, velocity.z);
    const double brakingTime = std::max(0.0, velocity.z) / maximumNetDeceleration * 1.2;
    const double timeToGo = clamp(std::max(kinematicTime, brakingTime),
                                  2.0, horizon * config.landingQpDtSec);
    rollingConfig.landingQpDtSec = timeToGo / static_cast<double>(horizon);
    const QpProblem problem = problemBuilder_.build(position, velocity,
        previousAccelerationNedMps2_, state.massKg, rollingConfig);
    QpResult solved = solver_->solve(problem, warmStart_);
    LandingGuidanceResult output{};
    output.solverStatus = solved.status;
    output.solveTimeMs = solved.solveTimeMs;
    output.iterations = solved.iterations;
    const bool timedOut = solved.solveTimeMs > 1000.0 / std::max(1.0, config.guidanceFrequencyHz);
    if (timedOut) output.solverStatus = RecoverySolverStatus::Timeout;
    const bool usable = !timedOut && (solved.status == RecoverySolverStatus::Solved
                      || solved.status == RecoverySolverStatus::MaxIterations)
                     && solved.solution.size() == static_cast<std::size_t>(3 * horizon);
    if (usable) {
        output.netAccelerationNedMps2 = {solved.solution[0], solved.solution[horizon],
                                         solved.solution[2 * horizon]};
        warmStart_.assign(solved.solution.size(), 0.0);
        for (int axis = 0; axis < 3; ++axis) {
            for (int k = 0; k + 1 < horizon; ++k) {
                warmStart_[axis * horizon + k] = solved.solution[axis * horizon + k + 1];
            }
            warmStart_[axis * horizon + horizon - 1] = solved.solution[axis * horizon + horizon - 1];
        }
    } else {
        output.netAccelerationNedMps2 = fallback(position, velocity, state.massKg, config);
        if (!timedOut) output.solverStatus = RecoverySolverStatus::Fallback;
        output.fallbackUsed = true;
        warmStart_.clear();
    }
    previousAccelerationNedMps2_ = output.netAccelerationNedMps2;
    return output;
}

} // namespace gnc
