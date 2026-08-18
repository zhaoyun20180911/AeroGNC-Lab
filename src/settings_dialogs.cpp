#include "gnc/settings_dialogs.hpp"

#include <algorithm>
#include <array>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace gnc::gui {
namespace {

constexpr COLORREF kDialogBackground = RGB(9, 18, 28);
constexpr COLORREF kDialogPanel = RGB(15, 29, 42);
constexpr COLORREF kDialogText = RGB(228, 237, 242);
constexpr COLORREF kDialogMuted = RGB(132, 155, 169);
constexpr COLORREF kDialogAccent = RGB(48, 213, 176);

constexpr int kApplyId = 9001;
constexpr int kCancelId = 9002;
constexpr int kRecommendId = 9003;
constexpr int kRestoreId = 9004;

std::wstring numberText(double value, int precision = 6) {
    std::wostringstream stream;
    stream << std::setprecision(precision) << value;
    return stream.str();
}

double readNumber(HWND control, double fallback) {
    wchar_t buffer[96]{};
    GetWindowTextW(control, buffer, 95);
    try { return std::stod(buffer); }
    catch (...) { return fallback; }
}

void setNumber(HWND control, double value) {
    SetWindowTextW(control, numberText(value).c_str());
}

double component(const Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

void setComponent(Vec3& value, int axis, double number) {
    if (axis == 0) value.x = number;
    else if (axis == 1) value.y = number;
    else value.z = number;
}

class ModalWindow {
public:
    ModalWindow(HWND owner, const wchar_t* title, int width, int height)
        : owner_(owner), title_(title), width_(width), height_(height) {}

    virtual ~ModalWindow() {
        if (font_) DeleteObject(font_);
        if (smallFont_) DeleteObject(smallFont_);
        if (brush_) DeleteObject(brush_);
        if (editBrush_) DeleteObject(editBrush_);
    }

    bool run() {
        registerClass();
        RECT ownerRect{};
        GetWindowRect(owner_, &ownerRect);
        const int ownerWidth = static_cast<int>(ownerRect.right - ownerRect.left);
        const int ownerHeight = static_cast<int>(ownerRect.bottom - ownerRect.top);
        const int x = static_cast<int>(ownerRect.left) + std::max(0, (ownerWidth - width_) / 2);
        const int y = static_cast<int>(ownerRect.top) + std::max(0, (ownerHeight - height_) / 2);
        hwnd_ = CreateWindowExW(WS_EX_DLGMODALFRAME, className(), title_,
            WS_POPUP | WS_CAPTION | WS_SYSMENU, x, y, width_, height_, owner_, nullptr,
            GetModuleHandleW(nullptr), this);
        if (!hwnd_) return false;
        EnableWindow(owner_, FALSE);
        ShowWindow(hwnd_, SW_SHOW);
        UpdateWindow(hwnd_);
        MSG message{};
        while (IsWindow(hwnd_) && GetMessageW(&message, nullptr, 0, 0) > 0) {
            if (!IsDialogMessageW(hwnd_, &message)) {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }
        }
        EnableWindow(owner_, TRUE);
        SetForegroundWindow(owner_);
        return accepted_;
    }

protected:
    virtual void build() = 0;
    virtual void apply() = 0;
    virtual void command(int id) { (void)id; }

    HWND label(const std::wstring& value, int x, int y, int width, int height = 22,
               bool muted = false) {
        HWND control = CreateWindowExW(0, L"STATIC", value.c_str(), WS_CHILD | WS_VISIBLE,
            x, y, width, height, hwnd_, nullptr, GetModuleHandleW(nullptr), nullptr);
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(muted ? smallFont_ : font_), TRUE);
        return control;
    }

    HWND edit(double value, int x, int y, int width, int id) {
        HWND control = CreateWindowExW(0, L"EDIT", numberText(value).c_str(),
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
            x, y, width, 27, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleW(nullptr), nullptr);
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font_), TRUE);
        return control;
    }

    HWND checkbox(const std::wstring& value, int x, int y, int width, int id, bool checked,
                  bool radio = false) {
        const DWORD style = WS_CHILD | WS_VISIBLE | WS_TABSTOP
            | (radio ? BS_AUTORADIOBUTTON : BS_AUTOCHECKBOX);
        HWND control = CreateWindowExW(0, L"BUTTON", value.c_str(), style,
            x, y, width, 25, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleW(nullptr), nullptr);
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font_), TRUE);
        SendMessageW(control, BM_SETCHECK, checked ? BST_CHECKED : BST_UNCHECKED, 0);
        return control;
    }

    HWND button(const std::wstring& value, int x, int y, int width, int id) {
        HWND control = CreateWindowExW(0, L"BUTTON", value.c_str(),
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_PUSHBUTTON,
            x, y, width, 32, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleW(nullptr), nullptr);
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font_), TRUE);
        return control;
    }

    HWND comboAxis(int x, int y, int width, int id, int selection) {
        HWND control = CreateWindowExW(0, L"COMBOBOX", L"",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | CBS_DROPDOWNLIST,
            x, y, width, 160, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)),
            GetModuleHandleW(nullptr), nullptr);
        SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(font_), TRUE);
        for (const wchar_t* item : {L"X 轴 / X axis", L"Y 轴 / Y axis", L"Z 轴 / Z axis"}) {
            SendMessageW(control, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item));
        }
        SendMessageW(control, CB_SETCURSEL, selection, 0);
        return control;
    }

    void standardButtons(int y) {
        button(L"应用 / Apply", width_ - 285, y, 120, kApplyId);
        button(L"取消 / Cancel", width_ - 155, y, 110, kCancelId);
    }

    static int axisSelection(const Vec3& axis) {
        const double values[3]{std::abs(axis.x), std::abs(axis.y), std::abs(axis.z)};
        return values[1] > values[0] && values[1] >= values[2] ? 1 : (values[2] > values[0] ? 2 : 0);
    }

    static Vec3 selectedAxis(HWND combo) {
        const int selection = static_cast<int>(SendMessageW(combo, CB_GETCURSEL, 0, 0));
        return selection == 1 ? Vec3{0.0, 1.0, 0.0}
             : (selection == 2 ? Vec3{0.0, 0.0, 1.0} : Vec3{1.0, 0.0, 0.0});
    }

    HWND hwnd_{};
    HWND owner_{};
    int width_{};
    int height_{};
    HFONT font_{};
    HFONT smallFont_{};
    HBRUSH brush_{};
    HBRUSH editBrush_{};
    bool accepted_{};

private:
    static const wchar_t* className() { return L"AerospaceGNCSettingsWindow"; }

    static void registerClass() {
        static bool registered = false;
        if (registered) return;
        WNDCLASSEXW windowClass{sizeof(windowClass)};
        windowClass.lpfnWndProc = &ModalWindow::windowProc;
        windowClass.hInstance = GetModuleHandleW(nullptr);
        windowClass.hCursor = LoadCursor(nullptr, IDC_ARROW);
        windowClass.hbrBackground = nullptr;
        windowClass.lpszClassName = className();
        RegisterClassExW(&windowClass);
        registered = true;
    }

    static LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        ModalWindow* dialog = reinterpret_cast<ModalWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(lParam);
            dialog = static_cast<ModalWindow*>(create->lpCreateParams);
            dialog->hwnd_ = hwnd;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(dialog));
        }
        if (!dialog) return DefWindowProcW(hwnd, message, wParam, lParam);
        switch (message) {
        case WM_CREATE:
            dialog->font_ = CreateFontW(-15, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                DEFAULT_PITCH, L"Segoe UI");
            dialog->smallFont_ = CreateFontW(-13, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
                DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                DEFAULT_PITCH, L"Segoe UI");
            dialog->brush_ = CreateSolidBrush(kDialogBackground);
            dialog->editBrush_ = CreateSolidBrush(RGB(12, 25, 37));
            dialog->build();
            return 0;
        case WM_COMMAND: {
            const int id = LOWORD(wParam);
            if (id == kApplyId) {
                dialog->apply();
                dialog->accepted_ = true;
                DestroyWindow(hwnd);
            } else if (id == kCancelId) {
                DestroyWindow(hwnd);
            } else {
                dialog->command(id);
            }
            return 0;
        }
        case WM_CTLCOLORSTATIC:
        case WM_CTLCOLORBTN: {
            HDC dc = reinterpret_cast<HDC>(wParam);
            SetBkMode(dc, TRANSPARENT);
            SetTextColor(dc, kDialogText);
            return reinterpret_cast<LRESULT>(dialog->brush_);
        }
        case WM_CTLCOLOREDIT:
        case WM_CTLCOLORLISTBOX: {
            HDC dc = reinterpret_cast<HDC>(wParam);
            SetTextColor(dc, kDialogText);
            SetBkColor(dc, RGB(12, 25, 37));
            return reinterpret_cast<LRESULT>(dialog->editBrush_);
        }
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT: {
            PAINTSTRUCT paint{};
            HDC dc = BeginPaint(hwnd, &paint);
            RECT client{};
            GetClientRect(hwnd, &client);
            FillRect(dc, &client, dialog->brush_);
            HPEN pen = CreatePen(PS_SOLID, 1, RGB(34, 54, 70));
            const HGDIOBJ old = SelectObject(dc, pen);
            MoveToEx(dc, 20, 58, nullptr); LineTo(dc, client.right - 20, 58);
            SelectObject(dc, old); DeleteObject(pen);
            EndPaint(hwnd, &paint);
            return 0;
        }
        case WM_CLOSE:
            DestroyWindow(hwnd);
            return 0;
        }
        return DefWindowProcW(hwnd, message, wParam, lParam);
    }

    const wchar_t* title_{};
};

class ControlDialog final : public ModalWindow {
public:
    ControlDialog(HWND owner, ScenarioKind scenario, ControlConfig config,
                  const Vec3& inertia, const Vec3& authority)
        : ModalWindow(owner, L"AeroGNC Lab v3.1｜姿态控制设置 / Attitude Control Settings", 1080, 650), scenario_(scenario),
          config_(config), inertia_(inertia), authority_(authority) {
        recommendation_ = recommendControlGains(
            scenario == ScenarioKind::Satellite ? ControlVehicle::Satellite : ControlVehicle::LaunchVehicle,
            inertia, authority);
    }

    const ControlConfig& result() const { return config_; }

protected:
    void build() override {
        label(L"姿态控制设置 / ATTITUDE CONTROL SETTINGS", 22, 16, 520, 28);
        label(scenario_ == ScenarioKind::Satellite ? L"卫星姿态控制 / Satellite attitude control"
                                                   : L"火箭姿态控制 / Launch vehicle attitude control",
              720, 18, 320, 24, true);
        mode_[0] = checkbox(L"不施加控制 / Control OFF", 28, 74, 220, 101,
                            config_.mode == ControlMode::Off, true);
        SetWindowLongPtrW(mode_[0], GWL_STYLE, GetWindowLongPtrW(mode_[0], GWL_STYLE) | WS_GROUP);
        mode_[1] = checkbox(L"默认配置 / Default", 310, 74, 200, 102,
                            config_.mode == ControlMode::Default, true);
        mode_[2] = checkbox(L"自定义配置 / Custom", 590, 74, 220, 103,
                            config_.mode == ControlMode::Custom, true);
        method_[0] = checkbox(L"PD", 36, 116, 90, 111, config_.customMethod == ControllerMethod::PD, true);
        SetWindowLongPtrW(method_[0], GWL_STYLE, GetWindowLongPtrW(method_[0], GWL_STYLE) | WS_GROUP);
        method_[1] = checkbox(L"PID", 136, 116, 90, 112, config_.customMethod == ControllerMethod::PID, true);
        method_[2] = checkbox(L"LQR", 236, 116, 90, 113, config_.customMethod == ControllerMethod::LQR, true);
        antiWindup_ = checkbox(L"抗积分饱和 / Anti-windup", 590, 116, 250, 114,
                               config_.antiWindup, false);

        label(L"参数 / Parameter", 28, 158, 170, 22, true);
        label(L"X 轴 / X", 230, 158, 130, 22, true);
        label(L"Y 轴 / Y", 380, 158, 130, 22, true);
        label(L"Z 轴 / Z", 530, 158, 130, 22, true);
        const std::array<std::wstring, 4> names{L"比例 Kp / Proportional", L"积分 Ki / Integral",
                                               L"微分 Kd / Derivative", L"积分限幅 / Integral limit"};
        for (int row = 0; row < 4; ++row) {
            gainLabels_[row] = label(names[row], 28, 190 + row * 48, 190, 25);
            const Vec3 source = row == 0 ? config_.proportionalGain : row == 1 ? config_.integralGain
                              : row == 2 ? config_.derivativeGain : config_.integralLimit;
            for (int axis = 0; axis < 3; ++axis) {
                gainEdits_[row][axis] = edit(component(source, axis), 228 + axis * 150,
                                             187 + row * 48, 130, 200 + row * 3 + axis);
            }
        }
        defaultHint_ = label(L"默认模式自动采用当前推荐增益。\r\nDefault mode automatically uses the current recommended gains.\r\n\r\n选择“自定义配置”后可编辑 PD、PID 或 LQR 参数。\r\nSelect Custom to edit PD, PID, or LQR parameters.",
                             28, 195, 410, 115, true);

        lqrLabels_[0] = label(L"姿态误差权重 / Attitude error weight", 28, 397, 285, 25);
        lqrLabels_[1] = label(L"角速度权重 / Angular-rate weight", 28, 440, 285, 25);
        lqrLabels_[2] = label(L"控制代价权重 / Control-effort weight", 28, 483, 285, 25);
        lqrEdits_[0] = edit(config_.attitudeErrorWeight, 325, 394, 130, 240);
        lqrEdits_[1] = edit(config_.angularRateWeight, 325, 437, 130, 241);
        lqrEdits_[2] = edit(config_.controlEffortWeight, 325, 480, 130, 242);

        recommendationLabel_ = label(L"", 720, 180, 330, 180, true);
        button(L"重新计算推荐值 / Recalculate", 720, 380, 320, kRecommendId);
        button(L"恢复推荐参数 / Restore", 720, 424, 320, kRestoreId);
        standardButtons(555);
        refreshRecommendationText();
        updateVisibility();
    }

    void command(int id) override {
        if ((id >= 101 && id <= 103) || (id >= 111 && id <= 113)) updateVisibility();
        if (id == kRecommendId || id == kRestoreId) restoreRecommendation();
    }

    void apply() override {
        config_.mode = SendMessageW(mode_[0], BM_GETCHECK, 0, 0) == BST_CHECKED ? ControlMode::Off
                     : (SendMessageW(mode_[2], BM_GETCHECK, 0, 0) == BST_CHECKED
                        ? ControlMode::Custom : ControlMode::Default);
        config_.customMethod = SendMessageW(method_[1], BM_GETCHECK, 0, 0) == BST_CHECKED
            ? ControllerMethod::PID : (SendMessageW(method_[2], BM_GETCHECK, 0, 0) == BST_CHECKED
                                       ? ControllerMethod::LQR : ControllerMethod::PD);
        for (int axis = 0; axis < 3; ++axis) {
            setComponent(config_.proportionalGain, axis,
                         readNumber(gainEdits_[0][axis], component(config_.proportionalGain, axis)));
            setComponent(config_.integralGain, axis,
                         readNumber(gainEdits_[1][axis], component(config_.integralGain, axis)));
            setComponent(config_.derivativeGain, axis,
                         readNumber(gainEdits_[2][axis], component(config_.derivativeGain, axis)));
            setComponent(config_.integralLimit, axis,
                         std::abs(readNumber(gainEdits_[3][axis], component(config_.integralLimit, axis))));
        }
        config_.antiWindup = SendMessageW(antiWindup_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.attitudeErrorWeight = std::max(1.0e-9, readNumber(lqrEdits_[0], config_.attitudeErrorWeight));
        config_.angularRateWeight = std::max(1.0e-9, readNumber(lqrEdits_[1], config_.angularRateWeight));
        config_.controlEffortWeight = std::max(1.0e-9, readNumber(lqrEdits_[2], config_.controlEffortWeight));
    }

private:
    void updateVisibility() {
        const bool custom = SendMessageW(mode_[2], BM_GETCHECK, 0, 0) == BST_CHECKED;
        const bool pid = SendMessageW(method_[1], BM_GETCHECK, 0, 0) == BST_CHECKED;
        const bool lqr = SendMessageW(method_[2], BM_GETCHECK, 0, 0) == BST_CHECKED;
        for (HWND method : method_) EnableWindow(method, custom);
        EnableWindow(antiWindup_, custom && pid);
        for (int row = 0; row < 4; ++row) {
            const bool visible = custom && !lqr && (row != 1 && row != 3 ? true : pid);
            ShowWindow(gainLabels_[row], visible ? SW_SHOW : SW_HIDE);
            for (HWND editControl : gainEdits_[row]) ShowWindow(editControl, visible ? SW_SHOW : SW_HIDE);
        }
        ShowWindow(defaultHint_, custom ? SW_HIDE : SW_SHOW);
        for (int index = 0; index < 3; ++index) {
            ShowWindow(lqrLabels_[index], custom && lqr ? SW_SHOW : SW_HIDE);
            ShowWindow(lqrEdits_[index], custom && lqr ? SW_SHOW : SW_HIDE);
        }
    }

    void restoreRecommendation() {
        for (int axis = 0; axis < 3; ++axis) {
            setNumber(gainEdits_[0][axis], component(recommendation_.proportionalGain, axis));
            setNumber(gainEdits_[1][axis], component(recommendation_.integralGain, axis));
            setNumber(gainEdits_[2][axis], component(recommendation_.derivativeGain, axis));
            setNumber(gainEdits_[3][axis], component(recommendation_.integralLimit, axis));
        }
        setNumber(lqrEdits_[0], recommendation_.attitudeErrorWeight);
        setNumber(lqrEdits_[1], recommendation_.angularRateWeight);
        setNumber(lqrEdits_[2], recommendation_.controlEffortWeight);
        refreshRecommendationText();
    }

    void refreshRecommendationText() {
        std::wostringstream stream;
        stream << L"当前飞行器推荐 / Current recommendation\r\n\r\n"
               << L"Kp  " << numberText(recommendation_.proportionalGain.x, 4) << L"  /  "
               << numberText(recommendation_.proportionalGain.y, 4) << L"  /  "
               << numberText(recommendation_.proportionalGain.z, 4) << L"\r\n"
               << L"Kd  " << numberText(recommendation_.derivativeGain.x, 4) << L"  /  "
               << numberText(recommendation_.derivativeGain.y, 4) << L"  /  "
               << numberText(recommendation_.derivativeGain.z, 4) << L"\r\n\r\n"
               << L"根据惯量和执行机构能力计算\r\nCalculated from inertia and actuator authority";
        SetWindowTextW(recommendationLabel_, stream.str().c_str());
    }

    ScenarioKind scenario_{};
    ControlConfig config_{};
    Vec3 inertia_{};
    Vec3 authority_{};
    ControlRecommendation recommendation_{};
    std::array<HWND, 3> mode_{};
    std::array<HWND, 3> method_{};
    HWND antiWindup_{};
    std::array<HWND, 4> gainLabels_{};
    std::array<std::array<HWND, 3>, 4> gainEdits_{};
    std::array<HWND, 3> lqrLabels_{};
    std::array<HWND, 3> lqrEdits_{};
    HWND recommendationLabel_{};
    HWND defaultHint_{};
};

class SatelliteOrbitControlDialog final : public ModalWindow {
public:
    SatelliteOrbitControlDialog(HWND owner, SatelliteOrbitControlConfig config)
        : ModalWindow(owner, L"AeroGNC Lab v3.1｜卫星轨道控制 / Satellite Orbit Control", 840, 700),
          config_(config) {}

    const SatelliteOrbitControlConfig& result() const { return config_; }

protected:
    void build() override {
        label(L"卫星轨道控制 / SATELLITE ORBIT CONTROL", 22, 16, 520, 28);
        label(L"名义六维状态反馈 + 三轴微推力器组 / Nominal six-state feedback with a three-axis thruster cluster",
              22, 70, 780, 24, true);
        enabled_ = checkbox(L"启用轨道控制 / Enable orbit control", 28, 105, 420, 801, config_.enabled);

        const std::array<std::wstring, 7> names{
            L"位置反馈增益 / Position gain (s⁻²)",
            L"速度反馈增益 / Velocity gain (s⁻¹)",
            L"最大合推力 / Maximum resultant thrust (N)",
            L"比冲 / Specific impulse (s)",
            L"初始推进剂质量 / Initial propellant mass (kg)",
            L"位置控制死区 / Position deadband (m)",
            L"速度控制死区 / Velocity deadband (m/s)"};
        const std::array<double, 7> values{
            config_.positionGainPerSec2, config_.velocityGainPerSec, config_.maxThrustN,
            config_.specificImpulseSec, config_.propellantMassKg, config_.positionDeadbandM,
            config_.velocityDeadbandMps};
        for (int index = 0; index < 7; ++index) {
            labels_[index] = label(names[index], 48, 154 + index * 50, 470, 24);
            edits_[index] = edit(values[index], 535, 151 + index * 50, 200, 820 + index);
        }
        label(L"控制器在 RTN/LVLH 误差基础上生成 ECI 修正力，再转换为本体系三轴推力器指令。\r\n"
              L"The controller generates an ECI correction force from RTN/LVLH errors and allocates it in body axes.",
              28, 518, 780, 48, true);
        label(L"推进剂耗尽后强制停止轨控；所有参数在下一次开始或重置时生效。\r\n"
              L"Orbit-control thrust stops at propellant depletion; settings apply on the next Start or Reset.",
              28, 572, 780, 44, true);
        standardButtons(620);
        updateEnabledState();
    }

    void command(int id) override {
        if (id == 801) updateEnabledState();
    }

    void apply() override {
        config_.enabled = SendMessageW(enabled_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.positionGainPerSec2 = clamp(readNumber(edits_[0], config_.positionGainPerSec2), 0.0, 1.0e-2);
        config_.velocityGainPerSec = clamp(readNumber(edits_[1], config_.velocityGainPerSec), 0.0, 1.0);
        config_.maxThrustN = clamp(readNumber(edits_[2], config_.maxThrustN), 0.0, 1000.0);
        config_.specificImpulseSec = clamp(readNumber(edits_[3], config_.specificImpulseSec), 1.0, 10000.0);
        config_.propellantMassKg = clamp(readNumber(edits_[4], config_.propellantMassKg), 0.0, 1.0e6);
        config_.positionDeadbandM = clamp(readNumber(edits_[5], config_.positionDeadbandM), 0.0, 1.0e6);
        config_.velocityDeadbandMps = clamp(readNumber(edits_[6], config_.velocityDeadbandMps), 0.0, 1.0e4);
    }

private:
    void updateEnabledState() {
        const bool enabled = SendMessageW(enabled_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        for (int index = 0; index < 7; ++index) {
            EnableWindow(labels_[index], enabled);
            EnableWindow(edits_[index], enabled);
        }
    }

    SatelliteOrbitControlConfig config_{};
    HWND enabled_{};
    std::array<HWND, 7> labels_{};
    std::array<HWND, 7> edits_{};
};

class GuidanceDialog final : public ModalWindow {
public:
    GuidanceDialog(HWND owner, RocketGuidanceConfig config)
        : ModalWindow(owner, L"AeroGNC Lab v3.1｜火箭轨迹制导 / Rocket Trajectory Guidance", 820, 720),
          config_(config) {}

    const RocketGuidanceConfig& result() const { return config_; }

protected:
    void build() override {
        label(L"火箭轨迹制导 / ROCKET TRAJECTORY GUIDANCE", 22, 16, 500, 28);
        label(L"位置/速度外环生成姿态指令，姿态控制器作为内环执行 / Position-velocity outer loop commands the attitude inner loop",
              22, 70, 760, 24, true);
        enabled_ = checkbox(L"启用轨迹制导外环 / Enable trajectory-guidance outer loop",
                            28, 104, 560, 701, config_.enabled);

        const std::array<std::wstring, 4> trackingLabels{
            L"位置反馈增益 / Position gain (s⁻²)",
            L"速度反馈增益 / Velocity gain (s⁻¹)",
            L"最大姿态修正角 / Max attitude correction (deg)",
            L"最大指令变化率 / Max command rate (deg/s)"};
        const std::array<double, 4> trackingValues{
            config_.positionGainPerSec2, config_.velocityGainPerSec,
            config_.maxCorrectionAngleDeg, config_.maxCommandRateDegPerSec};
        for (int index = 0; index < 4; ++index) {
            labels_[index] = label(trackingLabels[index], 44, 148 + index * 44, 430, 24);
            edits_[index] = edit(trackingValues[index], 500, 145 + index * 44, 190, 720 + index);
        }

        terminal_ = checkbox(L"启用终端轨道制导 / Enable terminal-orbit guidance",
                             28, 340, 480, 702, config_.terminalOrbitGuidance);
        labels_[4] = label(L"终端制导提前量 / Terminal lead time (s)", 44, 383, 430, 24);
        edits_[4] = edit(config_.terminalLeadTimeSec, 500, 380, 190, 724);
        adaptive_ = checkbox(L"自适应关机与有限补燃 / Adaptive cutoff and limited burn extension",
                             44, 426, 620, 703, config_.adaptiveCutoff);
        labels_[5] = label(L"比能量关机容差 / Specific-energy tolerance (J/kg)", 60, 469, 430, 24);
        edits_[5] = edit(config_.specificEnergyToleranceJPerKg, 500, 466, 190, 725);
        labels_[6] = label(L"最大补燃时间 / Maximum burn extension (s)", 60, 513, 430, 24);
        edits_[6] = edit(config_.maxBurnExtensionSec, 500, 510, 190, 726);
        label(L"说明 / Note: 本功能跟踪现有名义轨迹，不进行在线轨迹优化。推进剂耗尽后发动机必然关机。\r\n"
              L"Tracks the loaded nominal trajectory; it is not an online optimizer. Engine cutoff is mandatory at propellant depletion.",
              28, 558, 750, 48, true);
        standardButtons(630);
        updateEnabledState();
    }

    void command(int id) override {
        if (id == 701 || id == 702 || id == 703) updateEnabledState();
    }

    void apply() override {
        config_.enabled = SendMessageW(enabled_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.positionGainPerSec2 = clamp(readNumber(edits_[0], config_.positionGainPerSec2), 0.0, 1.0e-2);
        config_.velocityGainPerSec = clamp(readNumber(edits_[1], config_.velocityGainPerSec), 0.0, 1.0);
        config_.maxCorrectionAngleDeg = clamp(readNumber(edits_[2], config_.maxCorrectionAngleDeg), 0.0, 30.0);
        config_.maxCommandRateDegPerSec = clamp(readNumber(edits_[3], config_.maxCommandRateDegPerSec), 0.0, 20.0);
        config_.terminalOrbitGuidance = SendMessageW(terminal_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.terminalLeadTimeSec = clamp(readNumber(edits_[4], config_.terminalLeadTimeSec), 0.0, 300.0);
        config_.adaptiveCutoff = SendMessageW(adaptive_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.specificEnergyToleranceJPerKg = clamp(
            readNumber(edits_[5], config_.specificEnergyToleranceJPerKg), 0.0, 1.0e6);
        config_.maxBurnExtensionSec = clamp(readNumber(edits_[6], config_.maxBurnExtensionSec), 0.0, 300.0);
    }

private:
    void updateEnabledState() {
        const bool enabled = SendMessageW(enabled_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        const bool terminal = enabled && SendMessageW(terminal_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        const bool adaptive = terminal && SendMessageW(adaptive_, BM_GETCHECK, 0, 0) == BST_CHECKED;
        for (int index = 0; index < 4; ++index) {
            EnableWindow(labels_[index], enabled);
            EnableWindow(edits_[index], enabled);
        }
        EnableWindow(terminal_, enabled);
        EnableWindow(labels_[4], terminal);
        EnableWindow(edits_[4], terminal);
        EnableWindow(adaptive_, terminal);
        for (int index = 5; index < 7; ++index) {
            EnableWindow(labels_[index], adaptive);
            EnableWindow(edits_[index], adaptive);
        }
    }

    RocketGuidanceConfig config_{};
    HWND enabled_{};
    HWND terminal_{};
    HWND adaptive_{};
    std::array<HWND, 7> labels_{};
    std::array<HWND, 7> edits_{};
};

class DisturbanceDialog final : public ModalWindow {
public:
    DisturbanceDialog(HWND owner, ScenarioKind scenario, SatelliteDisturbanceConfig satellite,
                      RocketDisturbanceConfig rocket)
        : ModalWindow(owner, L"AeroGNC Lab v3.1｜扰动设置 / Disturbance Settings", 940, 780),
          scenario_(scenario), satellite_(satellite), rocket_(rocket) {}
    const SatelliteDisturbanceConfig& satelliteResult() const { return satellite_; }
    const RocketDisturbanceConfig& rocketResult() const { return rocket_; }

protected:
    void build() override {
        label(L"扰动设置 / DISTURBANCE SETTINGS", 22, 16, 460, 28);
        label(scenario_ == ScenarioKind::Satellite ? L"可组合卫星扰动 / Composable satellite disturbances"
                                                   : L"可组合火箭外扰 / Composable launch-vehicle disturbances",
              510, 18, 390, 24, true);
        if (scenario_ == ScenarioKind::Satellite) buildSatellite(); else buildRocket();
        standardButtons(685);
    }

    void apply() override {
        if (scenario_ == ScenarioKind::Satellite) applySatellite(); else applyRocket();
    }

private:
    void rowLabels(int y, const std::vector<std::wstring>& values, int startX = 260, int width = 130) {
        for (std::size_t index = 0; index < values.size(); ++index) {
            label(values[index], startX + static_cast<int>(index) * (width + 10), y, width, 20, true);
        }
    }

    void buildSatellite() {
        checks_[0] = checkbox(L"恒定扰动力矩 / Constant torque", 28, 80, 265, 401,
                              satellite_.constantEnabled);
        rowLabels(82, {L"X  N·m", L"Y  N·m", L"Z  N·m"}, 300, 130);
        for (int axis = 0; axis < 3; ++axis) edits_[axis] = edit(component(satellite_.constantTorqueNm, axis),
            300 + axis * 140, 106, 130, 410 + axis);

        checks_[1] = checkbox(L"脉冲扰动力矩 / Pulse torque", 28, 170, 265, 402,
                              satellite_.pulseEnabled);
        rowLabels(172, {L"轴 / Axis", L"幅值 / Magnitude", L"开始 / Start s", L"持续 / Duration s"}, 300, 130);
        combos_[0] = comboAxis(300, 196, 130, 420, axisSelection(satellite_.pulseAxis));
        edits_[3] = edit(satellite_.pulseMagnitudeNm, 440, 196, 130, 421);
        edits_[4] = edit(satellite_.pulseStartSec, 580, 196, 130, 422);
        edits_[5] = edit(satellite_.pulseDurationSec, 720, 196, 130, 423);

        checks_[2] = checkbox(L"正弦扰动力矩 / Sine torque", 28, 260, 265, 403,
                              satellite_.sineEnabled);
        rowLabels(262, {L"轴 / Axis", L"幅值 / Amplitude", L"频率 / Freq Hz", L"相位 / Phase deg"}, 300, 130);
        combos_[1] = comboAxis(300, 286, 130, 430, axisSelection(satellite_.sineAxis));
        edits_[6] = edit(satellite_.sineAmplitudeNm, 440, 286, 130, 431);
        edits_[7] = edit(satellite_.sineFrequencyHz, 580, 286, 130, 432);
        edits_[8] = edit(satellite_.sinePhaseDeg, 720, 286, 130, 433);

        checks_[3] = checkbox(L"随机扰动力矩 / Random torque", 28, 350, 265, 404,
                              satellite_.randomEnabled);
        rowLabels(352, {L"RMS  N·m", L"更新间隔 / Interval s", L"随机种子 / Seed"}, 300, 170);
        edits_[9] = edit(satellite_.randomRmsNm, 300, 376, 160, 440);
        edits_[10] = edit(satellite_.randomUpdateIntervalSec, 480, 376, 160, 441);
        edits_[11] = edit(static_cast<double>(satellite_.randomSeed), 660, 376, 160, 442);
        checks_[4] = checkbox(L"轨道 Δv 脉冲 / Orbital Δv impulse", 28, 455, 265, 405,
                              satellite_.deltaVImpulseEnabled);
        rowLabels(457, {L"径向 R / Radial", L"航向 T / Along-track",
                        L"法向 N / Normal", L"时刻 / Time s"}, 300, 130);
        edits_[12] = edit(satellite_.deltaVImpulseRtnMps.x, 300, 482, 130, 443);
        edits_[13] = edit(satellite_.deltaVImpulseRtnMps.y, 440, 482, 130, 444);
        edits_[14] = edit(satellite_.deltaVImpulseRtnMps.z, 580, 482, 130, 445);
        edits_[15] = edit(satellite_.deltaVImpulseTimeSec, 720, 482, 130, 446);
        label(L"Δv 分量采用瞬时 RTN/LVLH 坐标系，单位 m/s；每次仿真只施加一次。\r\n"
              L"Δv components use the instantaneous RTN/LVLH frame in m/s and are applied once per run.",
              28, 535, 850, 48, true);
        label(L"多项可同时启用；参数在下一次 Start/Reset 时进入动力学。\r\n"
              L"Multiple items may be active; values enter dynamics on the next Start/Reset.",
              28, 590, 850, 45, true);
    }

    void applySatellite() {
        satellite_.constantEnabled = SendMessageW(checks_[0], BM_GETCHECK, 0, 0) == BST_CHECKED;
        for (int axis = 0; axis < 3; ++axis) setComponent(satellite_.constantTorqueNm, axis,
            readNumber(edits_[axis], component(satellite_.constantTorqueNm, axis)));
        satellite_.pulseEnabled = SendMessageW(checks_[1], BM_GETCHECK, 0, 0) == BST_CHECKED;
        satellite_.pulseAxis = selectedAxis(combos_[0]);
        satellite_.pulseMagnitudeNm = readNumber(edits_[3], satellite_.pulseMagnitudeNm);
        satellite_.pulseStartSec = std::max(0.0, readNumber(edits_[4], satellite_.pulseStartSec));
        satellite_.pulseDurationSec = std::max(0.0, readNumber(edits_[5], satellite_.pulseDurationSec));
        satellite_.sineEnabled = SendMessageW(checks_[2], BM_GETCHECK, 0, 0) == BST_CHECKED;
        satellite_.sineAxis = selectedAxis(combos_[1]);
        satellite_.sineAmplitudeNm = readNumber(edits_[6], satellite_.sineAmplitudeNm);
        satellite_.sineFrequencyHz = std::max(0.0, readNumber(edits_[7], satellite_.sineFrequencyHz));
        satellite_.sinePhaseDeg = readNumber(edits_[8], satellite_.sinePhaseDeg);
        satellite_.randomEnabled = SendMessageW(checks_[3], BM_GETCHECK, 0, 0) == BST_CHECKED;
        satellite_.randomRmsNm = std::abs(readNumber(edits_[9], satellite_.randomRmsNm));
        satellite_.randomUpdateIntervalSec = std::max(1.0e-4,
            readNumber(edits_[10], satellite_.randomUpdateIntervalSec));
        satellite_.randomSeed = static_cast<std::uint32_t>(std::max(0.0,
            readNumber(edits_[11], static_cast<double>(satellite_.randomSeed))));
        satellite_.deltaVImpulseEnabled = SendMessageW(checks_[4], BM_GETCHECK, 0, 0) == BST_CHECKED;
        satellite_.deltaVImpulseRtnMps = {
            readNumber(edits_[12], satellite_.deltaVImpulseRtnMps.x),
            readNumber(edits_[13], satellite_.deltaVImpulseRtnMps.y),
            readNumber(edits_[14], satellite_.deltaVImpulseRtnMps.z)};
        satellite_.deltaVImpulseTimeSec = std::max(0.0,
            readNumber(edits_[15], satellite_.deltaVImpulseTimeSec));
    }

    void buildRocket() {
        checks_[0] = checkbox(L"稳态横风 / Steady crosswind", 28, 76, 250, 451,
                              rocket_.steadyCrosswindEnabled);
        rowLabels(78, {L"风速 / Speed m/s", L"方向 / Direction deg"}, 300, 180);
        edits_[0] = edit(rocket_.crosswindSpeedMps, 300, 102, 170, 460);
        edits_[1] = edit(rocket_.crosswindDirectionDeg, 490, 102, 170, 461);

        checks_[1] = checkbox(L"阵风 / Gust", 28, 158, 250, 452, rocket_.gustEnabled);
        rowLabels(160, {L"风速 / Speed", L"方向 / Direction", L"开始 / Start s", L"持续 / Duration s"}, 300, 130);
        edits_[2] = edit(rocket_.gustSpeedMps, 300, 184, 120, 462);
        edits_[3] = edit(rocket_.gustDirectionDeg, 440, 184, 120, 463);
        edits_[4] = edit(rocket_.gustStartSec, 580, 184, 120, 464);
        edits_[5] = edit(rocket_.gustDurationSec, 720, 184, 120, 465);

        checks_[2] = checkbox(L"脉冲外力 / Pulse force", 28, 240, 250, 453,
                              rocket_.pulseForceEnabled);
        rowLabels(242, {L"方向 / Direction", L"幅值 / Magnitude N", L"开始 / Start s", L"持续 / Duration s"}, 300, 130);
        combos_[0] = comboAxis(300, 266, 130, 470, axisSelection(rocket_.pulseForceDirectionEci));
        edits_[6] = edit(rocket_.pulseForceN, 440, 266, 130, 471);
        edits_[7] = edit(rocket_.pulseForceStartSec, 580, 266, 130, 472);
        edits_[8] = edit(rocket_.pulseForceDurationSec, 720, 266, 130, 473);

        checks_[3] = checkbox(L"脉冲外力矩 / Pulse torque", 28, 322, 250, 454,
                              rocket_.pulseTorqueEnabled);
        rowLabels(324, {L"轴 / Axis", L"幅值 / Magnitude N·m", L"开始 / Start s", L"持续 / Duration s"}, 300, 130);
        combos_[1] = comboAxis(300, 348, 130, 480, axisSelection(rocket_.pulseTorqueAxisBody));
        edits_[9] = edit(rocket_.pulseTorqueNm, 440, 348, 130, 481);
        edits_[10] = edit(rocket_.pulseTorqueStartSec, 580, 348, 130, 482);
        edits_[11] = edit(rocket_.pulseTorqueDurationSec, 720, 348, 130, 483);

        checks_[4] = checkbox(L"随机外扰 / Random disturbance", 28, 404, 260, 455,
                              rocket_.randomEnabled);
        rowLabels(406, {L"力 RMS / Force N", L"力矩 RMS / Torque N·m", L"更新 / Interval s", L"种子 / Seed"}, 300, 130);
        edits_[12] = edit(rocket_.randomForceRmsN, 300, 430, 120, 490);
        edits_[13] = edit(rocket_.randomTorqueRmsNm, 440, 430, 120, 491);
        edits_[14] = edit(rocket_.randomUpdateIntervalSec, 580, 430, 120, 492);
        edits_[15] = edit(static_cast<double>(rocket_.randomSeed), 720, 430, 120, 493);
        label(L"质量、推力、Isp 等模型偏差不属于扰动，在主页面设置。\r\n"
              L"Mass, thrust and Isp deviations belong to the truth model on the main page.",
              28, 500, 850, 55, true);
    }

    void applyRocket() {
        rocket_.steadyCrosswindEnabled = SendMessageW(checks_[0], BM_GETCHECK, 0, 0) == BST_CHECKED;
        rocket_.crosswindSpeedMps = readNumber(edits_[0], rocket_.crosswindSpeedMps);
        rocket_.crosswindDirectionDeg = readNumber(edits_[1], rocket_.crosswindDirectionDeg);
        rocket_.gustEnabled = SendMessageW(checks_[1], BM_GETCHECK, 0, 0) == BST_CHECKED;
        rocket_.gustSpeedMps = readNumber(edits_[2], rocket_.gustSpeedMps);
        rocket_.gustDirectionDeg = readNumber(edits_[3], rocket_.gustDirectionDeg);
        rocket_.gustStartSec = std::max(0.0, readNumber(edits_[4], rocket_.gustStartSec));
        rocket_.gustDurationSec = std::max(0.0, readNumber(edits_[5], rocket_.gustDurationSec));
        rocket_.pulseForceEnabled = SendMessageW(checks_[2], BM_GETCHECK, 0, 0) == BST_CHECKED;
        rocket_.pulseForceDirectionEci = selectedAxis(combos_[0]);
        rocket_.pulseForceN = readNumber(edits_[6], rocket_.pulseForceN);
        rocket_.pulseForceStartSec = std::max(0.0, readNumber(edits_[7], rocket_.pulseForceStartSec));
        rocket_.pulseForceDurationSec = std::max(0.0, readNumber(edits_[8], rocket_.pulseForceDurationSec));
        rocket_.pulseTorqueEnabled = SendMessageW(checks_[3], BM_GETCHECK, 0, 0) == BST_CHECKED;
        rocket_.pulseTorqueAxisBody = selectedAxis(combos_[1]);
        rocket_.pulseTorqueNm = readNumber(edits_[9], rocket_.pulseTorqueNm);
        rocket_.pulseTorqueStartSec = std::max(0.0, readNumber(edits_[10], rocket_.pulseTorqueStartSec));
        rocket_.pulseTorqueDurationSec = std::max(0.0, readNumber(edits_[11], rocket_.pulseTorqueDurationSec));
        rocket_.randomEnabled = SendMessageW(checks_[4], BM_GETCHECK, 0, 0) == BST_CHECKED;
        rocket_.randomForceRmsN = std::abs(readNumber(edits_[12], rocket_.randomForceRmsN));
        rocket_.randomTorqueRmsNm = std::abs(readNumber(edits_[13], rocket_.randomTorqueRmsNm));
        rocket_.randomUpdateIntervalSec = std::max(1.0e-4,
            readNumber(edits_[14], rocket_.randomUpdateIntervalSec));
        rocket_.randomSeed = static_cast<std::uint32_t>(std::max(0.0,
            readNumber(edits_[15], static_cast<double>(rocket_.randomSeed))));
    }

    ScenarioKind scenario_{};
    SatelliteDisturbanceConfig satellite_{};
    RocketDisturbanceConfig rocket_{};
    std::array<HWND, 5> checks_{};
    std::array<HWND, 2> combos_{};
    std::array<HWND, 16> edits_{};
};

class PerturbationDialog final : public ModalWindow {
public:
    PerturbationDialog(HWND owner, PerturbationConfig config)
        : ModalWindow(owner, L"AeroGNC Lab v3.1｜摄动设置 / Perturbation Settings", 690, 520), config_(config) {}
    const PerturbationConfig& result() const { return config_; }

protected:
    void build() override {
        label(L"卫星摄动占位 / SATELLITE PERTURBATION PLACEHOLDERS", 22, 16, 600, 28);
        const std::array<std::wstring, 5> names{
            L"J2 摄动 / J2", L"大气阻力 / Atmospheric drag",
            L"月球第三体引力 / Lunar third-body gravity",
            L"太阳第三体引力 / Solar third-body gravity",
            L"太阳光压 / Solar radiation pressure"};
        const bool values[5]{config_.j2Selected, config_.atmosphericDragSelected,
                             config_.moonThirdBodySelected, config_.sunThirdBodySelected,
                             config_.solarRadiationPressureSelected};
        for (int index = 0; index < 5; ++index) {
            checks_[index] = checkbox(names[index], 34, 84 + index * 58, 430, 601 + index, values[index]);
            label(L"未启用 / Not enabled", 480, 86 + index * 58, 170, 23, true);
        }
        standardButtons(405);
    }

    void apply() override {
        config_.j2Selected = SendMessageW(checks_[0], BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.atmosphericDragSelected = SendMessageW(checks_[1], BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.moonThirdBodySelected = SendMessageW(checks_[2], BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.sunThirdBodySelected = SendMessageW(checks_[3], BM_GETCHECK, 0, 0) == BST_CHECKED;
        config_.solarRadiationPressureSelected = SendMessageW(checks_[4], BM_GETCHECK, 0, 0) == BST_CHECKED;
    }

private:
    PerturbationConfig config_{};
    std::array<HWND, 5> checks_{};
};

} // namespace

bool showControlSettings(HWND owner, ScenarioKind scenario, ControlConfig& config,
                         const Vec3& inertiaDiagonalKgM2, const Vec3& actuatorAuthorityNm) {
    ControlDialog dialog(owner, scenario, config, inertiaDiagonalKgM2, actuatorAuthorityNm);
    if (!dialog.run()) return false;
    config = dialog.result();
    return true;
}

bool showDisturbanceSettings(HWND owner, ScenarioKind scenario,
                             SatelliteDisturbanceConfig& satellite,
                             RocketDisturbanceConfig& rocket) {
    DisturbanceDialog dialog(owner, scenario, satellite, rocket);
    if (!dialog.run()) return false;
    satellite = dialog.satelliteResult();
    rocket = dialog.rocketResult();
    return true;
}

bool showRocketGuidanceSettings(HWND owner, RocketGuidanceConfig& config) {
    GuidanceDialog dialog(owner, config);
    if (!dialog.run()) return false;
    config = dialog.result();
    return true;
}

bool showSatelliteOrbitControlSettings(HWND owner, SatelliteOrbitControlConfig& config) {
    SatelliteOrbitControlDialog dialog(owner, config);
    if (!dialog.run()) return false;
    config = dialog.result();
    return true;
}

bool showPerturbationSettings(HWND owner, PerturbationConfig& config) {
    PerturbationDialog dialog(owner, config);
    if (!dialog.run()) return false;
    config = dialog.result();
    return true;
}

} // namespace gnc::gui
