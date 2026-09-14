#include "gnc/recovery.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace gnc {

QpResult ProjectedGradientQpSolver::solve(const QpProblem& problem,
                                          const std::vector<double>& warmStart) {
    const auto start = std::chrono::steady_clock::now();
    QpResult result{};
    const int n = problem.variableCount;
    const std::size_t matrixSize = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    if (n <= 0 || problem.hessian.size() != matrixSize
        || problem.gradient.size() != static_cast<std::size_t>(n)
        || problem.lowerBound.size() != static_cast<std::size_t>(n)
        || problem.upperBound.size() != static_cast<std::size_t>(n)) {
        result.status = RecoverySolverStatus::NumericalError;
        return result;
    }
    result.solution.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(problem.lowerBound[i]) || !std::isfinite(problem.upperBound[i])
            || problem.lowerBound[i] > problem.upperBound[i]) {
            result.status = RecoverySolverStatus::Infeasible;
            return result;
        }
        const double initial = warmStart.size() == static_cast<std::size_t>(n) ? warmStart[i] : 0.0;
        result.solution[i] = clamp(initial, problem.lowerBound[i], problem.upperBound[i]);
    }

    bool converged = false;
    for (int iteration = 0; iteration < std::max(1, problem.maxIterations); ++iteration) {
        double maximumChange{};
        for (int i = 0; i < n; ++i) {
            const double diagonal = problem.hessian[static_cast<std::size_t>(i) * n + i];
            if (!std::isfinite(diagonal) || diagonal <= 1.0e-14) {
                result.status = RecoverySolverStatus::NumericalError;
                return result;
            }
            double gradient = problem.gradient[i];
            for (int j = 0; j < n; ++j) {
                gradient += problem.hessian[static_cast<std::size_t>(i) * n + j] * result.solution[j];
            }
            const double updated = clamp(result.solution[i] - gradient / diagonal,
                                         problem.lowerBound[i], problem.upperBound[i]);
            maximumChange = std::max(maximumChange, std::abs(updated - result.solution[i]));
            result.solution[i] = updated;
        }
        result.iterations = iteration + 1;
        if (maximumChange <= problem.tolerance) {
            converged = true;
            break;
        }
    }
    for (double value : result.solution) {
        if (!std::isfinite(value)) {
            result.status = RecoverySolverStatus::NumericalError;
            return result;
        }
    }
    result.status = converged ? RecoverySolverStatus::Solved : RecoverySolverStatus::MaxIterations;
    result.solveTimeMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
}

} // namespace gnc
