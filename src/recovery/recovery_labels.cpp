#include "gnc/recovery.hpp"
namespace gnc {
std::wstring recoveryPhaseBilingual(RecoveryPhase phase) {
    switch (phase) {
    case RecoveryPhase::Initialize: return L"初始化 / INITIALIZE";
    case RecoveryPhase::Flip: return L"翻转 / FLIP";
    case RecoveryPhase::Boostback: return L"返场点火 / BOOSTBACK";
    case RecoveryPhase::Coast: return L"滑行 / COAST";
    case RecoveryPhase::EntryBurn: return L"再入点火 / ENTRY BURN";
    case RecoveryPhase::AeroDescent: return L"气动下降 / AERO DESCENT";
    case RecoveryPhase::Landing: return L"着陆点火 / LANDING";
    case RecoveryPhase::Touchdown: return L"着陆 / TOUCHDOWN";
    default: return L"失败 / FAILED";
    }
}
std::wstring recoverySolverStatusBilingual(RecoverySolverStatus status) {
    switch (status) {
    case RecoverySolverStatus::Solved: return L"已求解 / Solved";
    case RecoverySolverStatus::MaxIterations: return L"达到迭代上限 / Max iterations";
    case RecoverySolverStatus::Infeasible: return L"不可行 / Infeasible";
    case RecoverySolverStatus::NumericalError: return L"数值错误 / Numerical error";
    case RecoverySolverStatus::Timeout: return L"超时 / Timeout";
    case RecoverySolverStatus::Fallback: return L"安全回退 / Fallback";
    default: return L"未运行 / Not run";
    }
}
} // namespace gnc
