#include "gnc/orbit.hpp"
#include "gnc/settings_dialogs.hpp"
#include "gnc/simulation.hpp"
#include "resource.h"

#include <windows.h>
#include <commctrl.h>
#include <commdlg.h>
#include <dwmapi.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using gnc::RunState;
using gnc::ScenarioKind;
using gnc::SimulationSample;

constexpr COLORREF kBackground = RGB(5, 11, 18);
constexpr COLORREF kPanel = RGB(11, 22, 33);
constexpr COLORREF kPanelRaised = RGB(16, 31, 44);
constexpr COLORREF kBorder = RGB(32, 55, 71);
constexpr COLORREF kGrid = RGB(27, 47, 61);
constexpr COLORREF kText = RGB(229, 238, 243);
constexpr COLORREF kMuted = RGB(126, 150, 164);
constexpr COLORREF kAccent = RGB(49, 214, 177);
constexpr COLORREF kCyan = RGB(70, 184, 255);
constexpr COLORREF kAmber = RGB(255, 186, 78);
constexpr COLORREF kRed = RGB(255, 103, 108);
constexpr COLORREF kPurple = RGB(181, 132, 255);

constexpr int kIdSatellite = 100;
constexpr int kIdRocket = 101;
constexpr int kIdControl = 104;
constexpr int kIdDisturbance = 105;
constexpr int kIdPerturbation = 106;
constexpr int kIdGuidance = 107;
constexpr int kIdOrbitControl = 108;
constexpr int kIdStart = 110;
constexpr int kIdPause = 111;
constexpr int kIdReset = 112;
constexpr int kIdExport = 113;
constexpr int kIdEnlarge = 115;
constexpr int kIdCameraFirst = 120;
constexpr int kIdFirstEdit = 300;
constexpr int kIdObjective = 500;
constexpr int kIdSite = 501;
constexpr int kIdOrbit = 502;
constexpr int kIdPlayback = 503;
constexpr int kIdPlotFirst = 510;
constexpr int kFieldCount = 24;

constexpr double kRecommendedSatelliteMassMinKg = 20.0;
constexpr double kRecommendedSatelliteMassMaxKg = 2000.0;
constexpr double kRecommendedSatelliteInertiaMin = 0.1;
constexpr double kRecommendedSatelliteInertiaMax = 5000.0;
constexpr std::array<double, 7> kRecommendedRocketDeviationAbs{
    5.0, 10.0, 5.0, 5.0, 20.0, 20.0, 1.0};

struct FieldSpec {
    std::wstring label;
    std::wstring value;
};

struct SeriesPoint {
    double first{};
    double second{};
    bool dual{};
};

struct CameraState {
    double yaw{-0.65};
    double pitch{0.48};
    double zoom{1.0};
    double panX{};
    double panY{};
};

std::wstring number(double value, int precision = 2) {
    std::wostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

std::wstring compact(double value, int precision = 2) {
    std::wostringstream stream;
    if (std::abs(value) >= 1.0e6 || (std::abs(value) > 0.0 && std::abs(value) < 0.001)) {
        stream << std::scientific << std::setprecision(precision) << value;
    } else {
        stream << std::fixed << std::setprecision(precision) << value;
    }
    return stream.str();
}

void fillSolid(HDC dc, const RECT& rect, COLORREF color) {
    HBRUSH brush = CreateSolidBrush(color);
    FillRect(dc, &rect, brush);
    DeleteObject(brush);
}

void fillRounded(HDC dc, const RECT& rect, COLORREF color, int radius = 12,
                 COLORREF border = kBorder) {
    HBRUSH brush = CreateSolidBrush(color);
    HPEN pen = CreatePen(PS_SOLID, 1, border);
    const HGDIOBJ oldBrush = SelectObject(dc, brush);
    const HGDIOBJ oldPen = SelectObject(dc, pen);
    RoundRect(dc, rect.left, rect.top, rect.right, rect.bottom, radius, radius);
    SelectObject(dc, oldPen);
    SelectObject(dc, oldBrush);
    DeleteObject(pen);
    DeleteObject(brush);
}

void line(HDC dc, int x1, int y1, int x2, int y2, COLORREF color,
          int width = 1, int style = PS_SOLID) {
    HPEN pen = CreatePen(style, width, color);
    const HGDIOBJ old = SelectObject(dc, pen);
    MoveToEx(dc, x1, y1, nullptr);
    LineTo(dc, x2, y2);
    SelectObject(dc, old);
    DeleteObject(pen);
}

void drawText(HDC dc, const std::wstring& value, RECT rect, HFONT font, COLORREF color,
              UINT flags = DT_LEFT | DT_VCENTER | DT_SINGLELINE) {
    const HGDIOBJ old = SelectObject(dc, font);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, color);
    DrawTextW(dc, value.c_str(), static_cast<int>(value.size()), &rect, flags | DT_NOPREFIX);
    SelectObject(dc, old);
}

HFONT makeFont(int height, int weight, const wchar_t* face = L"Microsoft YaHei UI") {
    return CreateFontW(-height, 0, 0, 0, weight, FALSE, FALSE, FALSE, DEFAULT_CHARSET,
                       OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY,
                       DEFAULT_PITCH | FF_DONTCARE, face);
}

double vecMaxAbs(const gnc::Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

double attitudeErrorDeg(const SimulationSample& sample) {
    return gnc::quaternionAngularDistance(sample.attitude, sample.referenceAttitude)
         * gnc::kRadToDeg;
}

std::filesystem::path executableDirectory() {
    std::array<wchar_t, 32768> path{};
    const DWORD length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    return std::filesystem::path(std::wstring(path.data(), length)).parent_path();
}

bool loadEmbeddedMissionFiles(HINSTANCE instance,
                              std::unordered_map<std::string, std::string>& files,
                              std::string& error) {
    struct ResourceSpec {
        int id;
        const char* fileName;
    };
    constexpr std::array<ResourceSpec, 7> resources{{
        {IDR_MISSION_SUMMARY, "mission_summary.csv"},
        {IDR_WENCHANG_300, "wenchang_300km_28p5deg.csv"},
        {IDR_WENCHANG_500, "wenchang_500km_51p6deg.csv"},
        {IDR_XICHANG_300, "xichang_300km_28p5deg.csv"},
        {IDR_XICHANG_500, "xichang_500km_51p6deg.csv"},
        {IDR_CAPE_300, "cape_300km_28p5deg.csv"},
        {IDR_CAPE_500, "cape_500km_51p6deg.csv"}}};

    files.clear();
    files.reserve(resources.size());
    for (const ResourceSpec& resource : resources) {
        const HRSRC handle = FindResourceW(instance, MAKEINTRESOURCEW(resource.id),
                                           MAKEINTRESOURCEW(10)); // RT_RCDATA
        if (!handle) {
            error = std::string("Embedded resource is missing: ") + resource.fileName;
            files.clear();
            return false;
        }
        const DWORD size = SizeofResource(instance, handle);
        const HGLOBAL loaded = LoadResource(instance, handle);
        const void* bytes = loaded ? LockResource(loaded) : nullptr;
        if (!bytes || size == 0) {
            error = std::string("Unable to read embedded resource: ") + resource.fileName;
            files.clear();
            return false;
        }
        files.emplace(resource.fileName,
                      std::string(static_cast<const char*>(bytes), static_cast<std::size_t>(size)));
    }
    error.clear();
    return true;
}

class Application {
public:
    explicit Application(HINSTANCE instance)
        : instance_(instance), satellite_(satelliteConfig_), rocket_(rocketConfig_) {}

    ~Application() {
        for (HFONT font : {fontTiny_, fontSmall_, fontNormal_, fontMedium_, fontLarge_, fontMono_}) {
            if (font) DeleteObject(font);
        }
        if (editBrush_) DeleteObject(editBrush_);
        if (warningBrush_) DeleteObject(warningBrush_);
        if (invalidBrush_) DeleteObject(invalidBrush_);
        if (listBrush_) DeleteObject(listBrush_);
    }

    bool create(int showCommand) {
        std::string loadError;
        const auto exe = executableDirectory();
        const std::array<std::filesystem::path, 3> candidates{
            exe / L"data" / L"rocket_missions",
            exe.parent_path().parent_path() / L"data" / L"rocket_missions",
            std::filesystem::current_path() / L"data" / L"rocket_missions"};
        for (const auto& candidate : candidates) {
            if (missions_.load(candidate, loadError)) break;
        }
        if (!missions_.loaded()) {
            std::unordered_map<std::string, std::string> embeddedFiles;
            if (loadEmbeddedMissionFiles(instance_, embeddedFiles, loadError)) {
                missions_.loadEmbedded(embeddedFiles, loadError);
            }
        }
        if (!missions_.loaded()) {
            const std::wstring message = L"无法读取六组火箭任务数据。\nUnable to load the six rocket mission datasets.\n\n"
                + std::wstring(loadError.begin(), loadError.end());
            MessageBoxW(nullptr, message.c_str(), L"AeroGNC Lab v3.2", MB_OK | MB_ICONERROR);
            return false;
        }
        selectMission(false);

        WNDCLASSEXW wc{sizeof(wc)};
        wc.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;
        wc.lpfnWndProc = &Application::windowProc;
        wc.hInstance = instance_;
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wc.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
        wc.hbrBackground = nullptr;
        wc.lpszClassName = L"AeroGNCLab31Window";
        if (!RegisterClassExW(&wc)) return false;

        hwnd_ = CreateWindowExW(0, wc.lpszClassName,
            L"AeroGNC Lab v3.2｜航天器GNC仿真实验平台",
            WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
            CW_USEDEFAULT, CW_USEDEFAULT, 1580, 970, nullptr, nullptr, instance_, this);
        if (!hwnd_) return false;
        const BOOL dark = TRUE;
        DwmSetWindowAttribute(hwnd_, 20, &dark, sizeof(dark));
        ShowWindow(hwnd_, showCommand);
        UpdateWindow(hwnd_);
        return true;
    }

private:
    static LRESULT CALLBACK windowProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
        Application* app = reinterpret_cast<Application*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
        if (message == WM_NCCREATE) {
            const auto* create = reinterpret_cast<CREATESTRUCTW*>(lParam);
            app = static_cast<Application*>(create->lpCreateParams);
            app->hwnd_ = hwnd;
            SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(app));
        }
        return app ? app->handle(message, wParam, lParam)
                   : DefWindowProcW(hwnd, message, wParam, lParam);
    }

    LRESULT handle(UINT message, WPARAM wParam, LPARAM lParam) {
        switch (message) {
        case WM_CREATE:
            createControls();
            SetTimer(hwnd_, 1, 16, nullptr);
            return 0;
        case WM_SIZE:
            layoutControls(LOWORD(lParam), HIWORD(lParam));
            InvalidateRect(hwnd_, nullptr, FALSE);
            return 0;
        case WM_GETMINMAXINFO:
            reinterpret_cast<MINMAXINFO*>(lParam)->ptMinTrackSize = {1280, 820};
            return 0;
        case WM_TIMER:
            onTimer();
            return 0;
        case WM_COMMAND:
            onCommand(LOWORD(wParam), HIWORD(wParam));
            return 0;
        case WM_LBUTTONDOWN:
            beginCameraDrag(lParam, false);
            return 0;
        case WM_RBUTTONDOWN:
            beginCameraDrag(lParam, true);
            return 0;
        case WM_MOUSEMOVE:
            moveCamera(lParam);
            return 0;
        case WM_LBUTTONUP:
        case WM_RBUTTONUP:
            endCameraDrag();
            return 0;
        case WM_MOUSEWHEEL:
            zoomCamera(wParam, lParam);
            return 0;
        case WM_LBUTTONDBLCLK:
            resetCamera();
            InvalidateRect(hwnd_, nullptr, FALSE);
            return 0;
        case WM_DRAWITEM:
            drawButton(*reinterpret_cast<DRAWITEMSTRUCT*>(lParam));
            return TRUE;
        case WM_CTLCOLOREDIT: {
            const HWND control = reinterpret_cast<HWND>(lParam);
            const int index = GetDlgCtrlID(control) - kIdFirstEdit;
            HDC dc = reinterpret_cast<HDC>(wParam);
            SetTextColor(dc, index >= 0 && index < kFieldCount && invalid_[index] ? kRed
                            : (index >= 0 && index < kFieldCount && warning_[index] ? kAmber : kText));
            SetBkColor(dc, RGB(9, 19, 28));
            if (index >= 0 && index < kFieldCount && invalid_[index]) {
                return reinterpret_cast<LRESULT>(invalidBrush_);
            }
            if (index >= 0 && index < kFieldCount && warning_[index]) {
                return reinterpret_cast<LRESULT>(warningBrush_);
            }
            return reinterpret_cast<LRESULT>(editBrush_);
        }
        case WM_CTLCOLORLISTBOX: {
            HDC dc = reinterpret_cast<HDC>(wParam);
            SetTextColor(dc, kText);
            SetBkColor(dc, kPanelRaised);
            return reinterpret_cast<LRESULT>(listBrush_);
        }
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT:
            paint();
            return 0;
        case WM_DESTROY:
            KillTimer(hwnd_, 1);
            PostQuitMessage(0);
            return 0;
        default:
            return DefWindowProcW(hwnd_, message, wParam, lParam);
        }
    }

    static POINT clientPoint(LPARAM lParam) {
        return {static_cast<short>(LOWORD(lParam)), static_cast<short>(HIWORD(lParam))};
    }

    void beginCameraDrag(LPARAM lParam, bool pan) {
        const POINT point = clientPoint(lParam);
        if (!PtInRect(&visualViewport_, point)) return;
        cameraDragging_ = true;
        cameraPanning_ = pan;
        cameraLastPoint_ = point;
        SetCapture(hwnd_);
    }

    void moveCamera(LPARAM lParam) {
        if (!cameraDragging_) return;
        const POINT point = clientPoint(lParam);
        const int dx = point.x - cameraLastPoint_.x;
        const int dy = point.y - cameraLastPoint_.y;
        cameraLastPoint_ = point;
        if (cameraPanning_) {
            camera_.panX += dx;
            camera_.panY += dy;
        } else {
            camera_.yaw += dx * 0.008;
            camera_.pitch = gnc::clamp(camera_.pitch + dy * 0.008, -1.48, 1.48);
        }
        cameraMode_ = -1;
        InvalidateRect(hwnd_, &visualViewport_, FALSE);
    }

    void endCameraDrag() {
        if (!cameraDragging_) return;
        cameraDragging_ = false;
        if (GetCapture() == hwnd_) ReleaseCapture();
    }

    void zoomCamera(WPARAM wParam, LPARAM lParam) {
        POINT point{static_cast<short>(LOWORD(lParam)), static_cast<short>(HIWORD(lParam))};
        ScreenToClient(hwnd_, &point);
        if (!PtInRect(&visualViewport_, point)) return;
        const int delta = GET_WHEEL_DELTA_WPARAM(wParam);
        camera_.zoom = gnc::clamp(camera_.zoom * std::pow(1.12, delta / 120.0), 0.18, 12.0);
        cameraMode_ = -1;
        InvalidateRect(hwnd_, &visualViewport_, FALSE);
    }

    void resetCamera() {
        camera_ = {};
        if (cameraMode_ == 1) {
            if (scenario_ == ScenarioKind::Satellite) {
                const gnc::Vec3 orbitNormal = sample().position.cross(sample().velocity).normalized();
                camera_.yaw = std::atan2(orbitNormal.x, orbitNormal.y);
                camera_.pitch = -std::asin(gnc::clamp(orbitNormal.z, -1.0, 1.0));
            }
        } else if (cameraMode_ == 2) {
            camera_.yaw = -0.4;
            camera_.pitch = 0.28;
            camera_.zoom = 1.6;
        } else if (cameraMode_ == 3) {
            camera_.yaw = 0.0;
            camera_.pitch = 1.47;
        }
    }

    void createControls() {
        fontTiny_ = makeFont(11, FW_NORMAL);
        fontSmall_ = makeFont(12, FW_NORMAL);
        fontNormal_ = makeFont(14, FW_NORMAL);
        fontMedium_ = makeFont(14, FW_SEMIBOLD);
        fontLarge_ = makeFont(23, FW_SEMIBOLD);
        fontMono_ = makeFont(12, FW_NORMAL, L"Cascadia Mono");
        editBrush_ = CreateSolidBrush(RGB(9, 19, 28));
        warningBrush_ = CreateSolidBrush(RGB(38, 31, 15));
        invalidBrush_ = CreateSolidBrush(RGB(43, 19, 22));
        listBrush_ = CreateSolidBrush(kPanelRaised);

        createButton(kIdSatellite, L"卫星 / Satellite");
        createButton(kIdRocket, L"运载火箭 / Launch Vehicle");
        createButton(kIdControl, L"姿态控制 / Attitude Control");
        createButton(kIdDisturbance, L"扰动设置 / Disturbance");
        createButton(kIdPerturbation, L"摄动设置 / Perturbation");
        createButton(kIdGuidance, L"轨迹制导 / Guidance");
        createButton(kIdOrbitControl, L"轨道控制 / Orbit Control");
        createButton(kIdStart, L"开始 / Start");
        createButton(kIdPause, L"暂停 / Pause");
        createButton(kIdReset, L"重置 / Reset");
        createButton(kIdExport, L"导出 / Export CSV");
        createButton(kIdEnlarge, L"放大 / Enlarge");
        createButton(kIdCameraFirst + 0, L"三维 / 3D");
        createButton(kIdCameraFirst + 1, L"轨道 / Orbit");
        createButton(kIdCameraFirst + 2, L"跟随 / Follow");
        createButton(kIdCameraFirst + 3, L"俯视 / Top");

        for (int index = 0; index < kFieldCount; ++index) {
            edits_[index] = CreateWindowExW(0, L"EDIT", L"", WS_CHILD | WS_VISIBLE
                | WS_TABSTOP | ES_AUTOHSCROLL | WS_BORDER,
                0, 0, 100, 28, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(kIdFirstEdit + index)),
                instance_, nullptr);
            SendMessageW(edits_[index], WM_SETFONT, reinterpret_cast<WPARAM>(fontMono_), TRUE);
            SendMessageW(edits_[index], EM_SETLIMITTEXT, 24, 0);
        }
        createTooltips();
        objectiveCombo_ = createCombo(kIdObjective);
        siteCombo_ = createCombo(kIdSite);
        orbitCombo_ = createCombo(kIdOrbit);
        playbackCombo_ = createCombo(kIdPlayback);
        for (int index = 0; index < 4; ++index) plotCombos_[index] = createCombo(kIdPlotFirst + index);

        const std::array<const wchar_t*, 4> objectives{
            L"对地定向 / Nadir Pointing", L"惯性定向 / Inertial Pointing",
            L"目标跟踪 / Target Tracking", L"姿态机动 / Slew Maneuver"};
        for (const auto* item : objectives) SendMessageW(objectiveCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item));
        SendMessageW(objectiveCombo_, CB_SETCURSEL, 0, 0);

        for (const auto site : {gnc::LaunchSite::Wenchang, gnc::LaunchSite::Xichang,
                                gnc::LaunchSite::CapeCanaveral}) {
            const std::wstring item = gnc::launchSiteBilingual(site);
            SendMessageW(siteCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item.c_str()));
        }
        SendMessageW(siteCombo_, CB_SETCURSEL, 0, 0);
        for (const auto orbit : {gnc::TargetOrbit::Leo300Km28_5Deg,
                                 gnc::TargetOrbit::Leo500Km51_6Deg}) {
            const std::wstring item = gnc::targetOrbitBilingual(orbit);
            SendMessageW(orbitCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item.c_str()));
        }
        SendMessageW(orbitCombo_, CB_SETCURSEL, 0, 0);

        const std::array<const wchar_t*, 10> speeds{
            L"0.25×", L"0.5×", L"1×", L"2×", L"5×", L"10×", L"20×", L"50×", L"100×", L"最快 / Max"};
        for (const auto* item : speeds) SendMessageW(playbackCombo_, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(item));
        SendMessageW(playbackCombo_, CB_SETCURSEL, 6, 0);

        fillPlotCombos();
        setFieldValues();
        updateControlState();
        lastTick_ = std::chrono::steady_clock::now();
    }

    void createButton(int id, const wchar_t* label) {
        HWND button = CreateWindowExW(0, L"BUTTON", label,
            WS_CHILD | WS_VISIBLE | WS_TABSTOP | BS_OWNERDRAW,
            0, 0, 120, 34, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), instance_, nullptr);
        SendMessageW(button, WM_SETFONT, reinterpret_cast<WPARAM>(fontMedium_), TRUE);
        buttons_[id] = button;
    }

    HWND createCombo(int id) {
        HWND combo = CreateWindowExW(0, L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | WS_TABSTOP
            | CBS_DROPDOWNLIST | WS_VSCROLL,
            0, 0, 160, 240, hwnd_, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), instance_, nullptr);
        SendMessageW(combo, WM_SETFONT, reinterpret_cast<WPARAM>(fontSmall_), TRUE);
        return combo;
    }

    void createTooltips() {
        tooltip_ = CreateWindowExW(WS_EX_TOPMOST, TOOLTIPS_CLASSW, nullptr,
            WS_POPUP | TTS_ALWAYSTIP | TTS_NOPREFIX,
            CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT, CW_USEDEFAULT,
            hwnd_, nullptr, instance_, nullptr);
        SendMessageW(tooltip_, TTM_SETMAXTIPWIDTH, 0, 430);
        SendMessageW(tooltip_, TTM_SETDELAYTIME, TTDT_INITIAL, 350);
        updateTooltipTexts(false);
        for (int index = 0; index < kFieldCount; ++index) {
            TOOLINFOW info{sizeof(info)};
            info.uFlags = TTF_IDISHWND | TTF_SUBCLASS;
            info.hwnd = hwnd_;
            info.uId = reinterpret_cast<UINT_PTR>(edits_[index]);
            info.lpszText = tooltipTexts_[index].data();
            SendMessageW(tooltip_, TTM_ADDTOOLW, 0, reinterpret_cast<LPARAM>(&info));
        }
    }

    void updateTooltipTexts(bool updateControls = true) {
        if (scenario_ == ScenarioKind::Satellite) {
            tooltipTexts_ = {
                L"合法 / Valid: a > 地球半径 / Earth radius. 建议 LEO / Suggested LEO: 6578–7378 km.",
                L"合法 / Valid: 0 ≤ e < 1. 建议实验 / Suggested experiment: 0–0.2.",
                L"合法范围 / Valid range: 0–180 deg.", L"角度输入 / Angular input: deg.",
                L"角度输入 / Angular input: deg.", L"角度输入 / Angular input: deg.",
                L"中小型卫星建议 / Small-to-medium satellite suggestion: 20–2000 kg.",
                L"建议主惯量 / Suggested principal inertia: 0.1–5000 kg·m².",
                L"建议主惯量 / Suggested principal inertia: 0.1–5000 kg·m².",
                L"建议主惯量 / Suggested principal inertia: 0.1–5000 kg·m².",
                L"建议飞轮惯量 / Suggested wheel inertia: 0.001–10 kg·m².",
                L"建议飞轮力矩 / Suggested wheel torque: 0.01–2 N·m.",
                L"建议飞轮转速 / Suggested wheel speed: 500–10000 rpm.",
                L"必须大于 0；物理步长固定 0.02 s / Must be positive; physics step stays 0.02 s.",
                L"任务相关参数 / Mission-specific parameter.", L"任务相关参数 / Mission-specific parameter.",
                L"任务相关参数 / Mission-specific parameter.", L"任务相关参数 / Mission-specific parameter.",
                L"实际初始姿态相对任务参考姿态的滚转偏差 / Initial roll offset from the mission reference attitude.",
                L"实际初始姿态相对任务参考姿态的俯仰偏差 / Initial pitch offset from the mission reference attitude.",
                L"实际初始姿态相对任务参考姿态的偏航偏差 / Initial yaw offset from the mission reference attitude.",
                L"实际初始机体系滚转角速度偏差 / Initial body-frame roll-rate error.",
                L"实际初始机体系俯仰角速度偏差 / Initial body-frame pitch-rate error.",
                L"实际初始机体系偏航角速度偏差 / Initial body-frame yaw-rate error."};
        } else {
            tooltipTexts_ = {
                L"建议实验范围 / Suggested experiment: ±5%.", L"建议实验范围 / Suggested experiment: ±10%.",
                L"建议实验范围 / Suggested experiment: ±5%.", L"建议实验范围 / Suggested experiment: ±5%.",
                L"建议实验范围 / Suggested experiment: ±20%.", L"建议实验范围 / Suggested experiment: ±20%.",
                L"建议小角度范围 / Suggested small-angle range: ±1 deg.",
                L"实际初始姿态相对标称参考 / Actual initial attitude relative to nominal reference.",
                L"实际初始姿态相对标称参考 / Actual initial attitude relative to nominal reference.",
                L"实际初始姿态相对标称参考 / Actual initial attitude relative to nominal reference.",
                L"实际初始机体系角速率误差 / Actual initial body-rate error.",
                L"实际初始机体系角速率误差 / Actual initial body-rate error.",
                L"实际初始机体系角速率误差 / Actual initial body-rate error.",
                L"超过标称文件末端将自动进入轨道滑行 / Beyond nominal file end, orbital coast starts automatically.",
                L"", L"", L"", L"", L"", L"", L"", L"", L"", L""};
        }
        if (!updateControls || !tooltip_) return;
        for (int index = 0; index < kFieldCount; ++index) {
            TOOLINFOW info{sizeof(info)};
            info.uFlags = TTF_IDISHWND;
            info.hwnd = hwnd_;
            info.uId = reinterpret_cast<UINT_PTR>(edits_[index]);
            info.lpszText = tooltipTexts_[index].data();
            SendMessageW(tooltip_, TTM_UPDATETIPTEXTW, 0, reinterpret_cast<LPARAM>(&info));
        }
    }

    int bottomTop(int height) const {
        const bool satellite = scenario_ == ScenarioKind::Satellite;
        return std::max(satellite ? 460 : 518, height - (satellite ? 360 : 302));
    }

    RECT contentRect(int width, int height) const {
        return {22, 160, width - 22, bottomTop(height) - 12};
    }

    std::array<RECT, 4> plotRects(int width, int height) const {
        const RECT content = contentRect(width, height);
        const int gap = 12;
        const int visualWidth = enlarged_ ? content.right - content.left
            : static_cast<int>((content.right - content.left - gap) * 0.37);
        const int left = content.left + visualWidth + gap;
        const int totalWidth = content.right - left;
        const int halfWidth = (totalWidth - gap) / 2;
        const int halfHeight = (content.bottom - content.top - gap) / 2;
        return {{{left, content.top, left + halfWidth, content.top + halfHeight},
                 {left + halfWidth + gap, content.top, content.right, content.top + halfHeight},
                 {left, content.top + halfHeight + gap, left + halfWidth, content.bottom},
                 {left + halfWidth + gap, content.top + halfHeight + gap, content.right, content.bottom}}};
    }

    void layoutControls(int width, int height) {
        if (width <= 0 || height <= 0) return;
        MoveWindow(buttons_[kIdSatellite], 22, 58, 166, 34, TRUE);
        MoveWindow(buttons_[kIdRocket], 195, 58, 216, 34, TRUE);

        int x = width - 22;
        const std::array<std::pair<int, int>, 4> actions{{{kIdExport, 145}, {kIdReset, 112},
                                                          {kIdPause, 118}, {kIdStart, 122}}};
        for (const auto [id, buttonWidth] : actions) {
            x -= buttonWidth;
            MoveWindow(buttons_[id], x, 18, buttonWidth, 36, TRUE);
            x -= 7;
        }
        MoveWindow(buttons_[kIdControl], 22, 100, 205, 34, TRUE);
        MoveWindow(buttons_[kIdGuidance], 234, 100, 205, 34, TRUE);
        MoveWindow(buttons_[kIdOrbitControl], 234, 100, 205, 34, TRUE);
        MoveWindow(buttons_[kIdDisturbance], 446, 100, 190, 34, TRUE);
        MoveWindow(buttons_[kIdPerturbation], 643, 100, 190, 34, TRUE);
        MoveWindow(buttons_[kIdEnlarge], width - 178, 100, 156, 34, TRUE);

        const RECT content = contentRect(width, height);
        MoveWindow(buttons_[kIdCameraFirst + 0], content.left + 14, content.top + 45, 102, 29, TRUE);
        MoveWindow(buttons_[kIdCameraFirst + 1], content.left + 121, content.top + 45, 112, 29, TRUE);
        MoveWindow(buttons_[kIdCameraFirst + 2], content.left + 238, content.top + 45, 112, 29, TRUE);
        MoveWindow(buttons_[kIdCameraFirst + 3], content.left + 355, content.top + 45, 105, 29, TRUE);

        const auto cards = plotRects(width, height);
        for (int index = 0; index < 4; ++index) {
            const RECT& card = cards[index];
            MoveWindow(plotCombos_[index], card.left + 12, card.top + 9,
                       std::max(80L, card.right - card.left - 24), 210, TRUE);
            ShowWindow(plotCombos_[index], enlarged_ ? SW_HIDE : SW_SHOW);
        }

        const int top = bottomTop(height);
        ShowWindow(objectiveCombo_, scenario_ == ScenarioKind::Satellite ? SW_SHOW : SW_HIDE);
        ShowWindow(siteCombo_, scenario_ == ScenarioKind::Rocket ? SW_SHOW : SW_HIDE);
        ShowWindow(orbitCombo_, scenario_ == ScenarioKind::Rocket ? SW_SHOW : SW_HIDE);
        if (scenario_ == ScenarioKind::Satellite) {
            MoveWindow(objectiveCombo_, 225, top + 42, 260, 220, TRUE);
        } else {
            MoveWindow(siteCombo_, 185, top + 42, 240, 220, TRUE);
            MoveWindow(orbitCombo_, 535, top + 42, 255, 180, TRUE);
        }
        MoveWindow(playbackCombo_, width - 205, top + 42, 173, 220, TRUE);

        const int margin = 24;
        const int gap = 9;
        const int columns = 6;
        const int columnWidth = (width - 2 * margin - gap * (columns - 1)) / columns;
        for (int index = 0; index < kFieldCount; ++index) {
            const int row = index / columns;
            const int column = index % columns;
            MoveWindow(edits_[index], margin + column * (columnWidth + gap), top + 101 + row * 57,
                       columnWidth, 28, TRUE);
        }
    }

    std::array<FieldSpec, kFieldCount> fieldSpecs() const {
        std::array<FieldSpec, kFieldCount> fields{};
        if (scenario_ == ScenarioKind::Satellite) {
            const auto& c = satelliteConfig_;
            fields = {{{L"半长轴 / Semi-major axis (km)", number(c.orbit.semiMajorAxisKm, 3)},
                       {L"偏心率 / Eccentricity", number(c.orbit.eccentricity, 5)},
                       {L"倾角 / Inclination (deg)", number(c.orbit.inclinationDeg, 2)},
                       {L"升交点赤经 / RAAN (deg)", number(c.orbit.raanDeg, 2)},
                       {L"近地点幅角 / Arg. perigee (deg)", number(c.orbit.argumentOfPerigeeDeg, 2)},
                       {L"真近点角 / True anomaly (deg)", number(c.orbit.trueAnomalyDeg, 2)},
                       {L"质量 / Mass (kg)", number(c.massKg, 1)},
                       {L"转动惯量 / Inertia Ix (kg·m²)", number(c.inertiaKgM2.x, 2)},
                       {L"转动惯量 / Inertia Iy (kg·m²)", number(c.inertiaKgM2.y, 2)},
                       {L"转动惯量 / Inertia Iz (kg·m²)", number(c.inertiaKgM2.z, 2)},
                       {L"飞轮惯量 / Wheel inertia (kg·m²)", number(c.wheelInertiaKgM2, 3)},
                       {L"飞轮最大力矩 / Max torque (N·m)", number(c.wheelMaxTorqueNm, 3)},
                       {L"飞轮最大转速 / Max speed (rpm)", number(c.wheelMaxSpeedRpm, 0)},
                       {L"仿真时长 / Duration (s)", number(c.durationSec, 0)},
                       {}, {}, {}, {}}};
            switch (c.mission.objective) {
            case gnc::SatelliteObjective::NadirPointing:
                break;
            case gnc::SatelliteObjective::InertialPointing:
                fields[14] = {L"目标滚转 / Target roll (deg)", number(c.mission.targetEulerDeg.x, 2)};
                fields[15] = {L"目标俯仰 / Target pitch (deg)", number(c.mission.targetEulerDeg.y, 2)};
                fields[16] = {L"目标偏航 / Target yaw (deg)", number(c.mission.targetEulerDeg.z, 2)};
                break;
            case gnc::SatelliteObjective::TargetTracking:
                fields[14] = {L"目标纬度 / Target latitude (deg)", number(c.mission.targetLatitudeDeg, 3)};
                fields[15] = {L"目标经度 / Target longitude (deg)", number(c.mission.targetLongitudeDeg, 3)};
                break;
            case gnc::SatelliteObjective::SlewManeuver:
                fields[14] = {L"机动目标滚转 / Slew roll (deg)", number(c.mission.slewTargetEulerDeg.x, 2)};
                fields[15] = {L"机动目标俯仰 / Slew pitch (deg)", number(c.mission.slewTargetEulerDeg.y, 2)};
                fields[16] = {L"机动目标偏航 / Slew yaw (deg)", number(c.mission.slewTargetEulerDeg.z, 2)};
                fields[17] = {L"机动开始 / Slew start (s)", number(c.mission.slewStartSec, 1)};
                break;
            }
            fields[18] = {L"初始滚转偏差 / Initial roll error (deg)", number(c.initialErrorDeg.x, 2)};
            fields[19] = {L"初始俯仰偏差 / Initial pitch error (deg)", number(c.initialErrorDeg.y, 2)};
            fields[20] = {L"初始偏航偏差 / Initial yaw error (deg)", number(c.initialErrorDeg.z, 2)};
            fields[21] = {L"初始滚转角速度 / Initial roll rate (deg/s)", number(c.initialRateDegPerSec.x, 3)};
            fields[22] = {L"初始俯仰角速度 / Initial pitch rate (deg/s)", number(c.initialRateDegPerSec.y, 3)};
            fields[23] = {L"初始偏航角速度 / Initial yaw rate (deg/s)", number(c.initialRateDegPerSec.z, 3)};
            return fields;
        }

        const auto& d = rocketConfig_.deviations;
        return {{{L"质量偏差 / Mass deviation (%)", number(d.massPercent, 2)},
                 {L"惯量偏差 / Inertia deviation (%)", number(d.inertiaPercent, 2)},
                 {L"推力偏差 / Thrust deviation (%)", number(d.thrustPercent, 2)},
                 {L"比冲偏差 / Isp deviation (%)", number(d.specificImpulsePercent, 2)},
                 {L"阻力系数偏差 / Cd deviation (%)", number(d.dragCoefficientPercent, 2)},
                 {L"大气密度偏差 / Density deviation (%)", number(d.atmosphericDensityPercent, 2)},
                 {L"TVC 零偏 / TVC zero bias (deg)", number(d.tvcZeroBiasDeg, 3)},
                 {L"初始滚转误差 / Initial roll error (deg)", number(rocketConfig_.initialAttitudeErrorDeg.x, 2)},
                 {L"初始俯仰误差 / Initial pitch error (deg)", number(rocketConfig_.initialAttitudeErrorDeg.y, 2)},
                 {L"初始偏航误差 / Initial yaw error (deg)", number(rocketConfig_.initialAttitudeErrorDeg.z, 2)},
                 {L"初始滚转角速率 / Roll rate error (deg/s)", number(rocketConfig_.initialRateErrorDegPerSec.x, 3)},
                 {L"初始俯仰角速率 / Pitch rate error (deg/s)", number(rocketConfig_.initialRateErrorDegPerSec.y, 3)},
                 {L"初始偏航角速率 / Yaw rate error (deg/s)", number(rocketConfig_.initialRateErrorDegPerSec.z, 3)},
                 {L"仿真时长 / Duration (s)", number(rocketConfig_.durationSec, 0)},
                 {}, {}, {}, {}}};
    }

    void setFieldValues() {
        const auto fields = fieldSpecs();
        internalEdit_ = true;
        for (int index = 0; index < kFieldCount; ++index) {
            SetWindowTextW(edits_[index], fields[index].value.c_str());
            ShowWindow(edits_[index], fields[index].label.empty() ? SW_HIDE : SW_SHOW);
        }
        internalEdit_ = false;
        validateInputs(false);
        updateButtonText();
    }

    double readField(int index, double fallback, bool* parsed = nullptr) const {
        wchar_t buffer[64]{};
        GetWindowTextW(edits_[index], buffer, 63);
        try {
            std::size_t used{};
            const double value = std::stod(buffer, &used);
            if (used != std::wstring(buffer).size() || !std::isfinite(value)) throw std::invalid_argument("field");
            if (parsed) *parsed = true;
            return value;
        } catch (...) {
            if (parsed) *parsed = false;
            return fallback;
        }
    }

    bool readConfig(bool report) {
        if (!validateInputs(report)) return false;
        if (scenario_ == ScenarioKind::Satellite) {
            auto& c = satelliteConfig_;
            c.orbit.semiMajorAxisKm = readField(0, c.orbit.semiMajorAxisKm);
            c.orbit.eccentricity = readField(1, c.orbit.eccentricity);
            c.orbit.inclinationDeg = readField(2, c.orbit.inclinationDeg);
            c.orbit.raanDeg = readField(3, c.orbit.raanDeg);
            c.orbit.argumentOfPerigeeDeg = readField(4, c.orbit.argumentOfPerigeeDeg);
            c.orbit.trueAnomalyDeg = readField(5, c.orbit.trueAnomalyDeg);
            c.orbitAltitudeKm = c.orbit.semiMajorAxisKm - gnc::kEarthEquatorialRadiusM / 1000.0;
            c.massKg = readField(6, c.massKg);
            c.inertiaKgM2 = {readField(7, c.inertiaKgM2.x), readField(8, c.inertiaKgM2.y),
                             readField(9, c.inertiaKgM2.z)};
            c.wheelInertiaKgM2 = readField(10, c.wheelInertiaKgM2);
            c.wheelMaxTorqueNm = readField(11, c.wheelMaxTorqueNm);
            c.wheelMaxSpeedRpm = readField(12, c.wheelMaxSpeedRpm);
            c.durationSec = readField(13, c.durationSec);
            c.mission.objective = static_cast<gnc::SatelliteObjective>(
                std::max<LRESULT>(0, SendMessageW(objectiveCombo_, CB_GETCURSEL, 0, 0)));
            if (c.mission.objective == gnc::SatelliteObjective::InertialPointing) {
                c.mission.targetEulerDeg = {readField(14, c.mission.targetEulerDeg.x),
                                            readField(15, c.mission.targetEulerDeg.y),
                                            readField(16, c.mission.targetEulerDeg.z)};
            } else if (c.mission.objective == gnc::SatelliteObjective::TargetTracking) {
                c.mission.targetLatitudeDeg = readField(14, c.mission.targetLatitudeDeg);
                c.mission.targetLongitudeDeg = readField(15, c.mission.targetLongitudeDeg);
            } else {
                c.mission.slewTargetEulerDeg = {readField(14, c.mission.slewTargetEulerDeg.x),
                                                readField(15, c.mission.slewTargetEulerDeg.y),
                                                readField(16, c.mission.slewTargetEulerDeg.z)};
                c.mission.slewStartSec = readField(17, c.mission.slewStartSec);
            }
            c.initialErrorDeg = {readField(18, c.initialErrorDeg.x),
                                 readField(19, c.initialErrorDeg.y),
                                 readField(20, c.initialErrorDeg.z)};
            c.initialRateDegPerSec = {readField(21, c.initialRateDegPerSec.x),
                                      readField(22, c.initialRateDegPerSec.y),
                                      readField(23, c.initialRateDegPerSec.z)};
        } else {
            auto& c = rocketConfig_;
            c.deviations = {readField(0, c.deviations.massPercent),
                            readField(1, c.deviations.inertiaPercent),
                            readField(2, c.deviations.thrustPercent),
                            readField(3, c.deviations.specificImpulsePercent),
                            readField(4, c.deviations.dragCoefficientPercent),
                            readField(5, c.deviations.atmosphericDensityPercent),
                            readField(6, c.deviations.tvcZeroBiasDeg)};
            c.initialAttitudeErrorDeg = {readField(7, c.initialAttitudeErrorDeg.x),
                                         readField(8, c.initialAttitudeErrorDeg.y),
                                         readField(9, c.initialAttitudeErrorDeg.z)};
            c.initialRateErrorDegPerSec = {readField(10, c.initialRateErrorDegPerSec.x),
                                           readField(11, c.initialRateErrorDegPerSec.y),
                                           readField(12, c.initialRateErrorDegPerSec.z)};
            c.durationSec = readField(13, c.durationSec);
            selectMission(false);
        }
        const int speedIndex = static_cast<int>(SendMessageW(playbackCombo_, CB_GETCURSEL, 0, 0));
        const std::array<double, 9> speeds{0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0};
        const double speed = speedIndex >= 0 && speedIndex < 9 ? speeds[speedIndex] : 1.0;
        satelliteConfig_.playbackSpeed = speed;
        rocketConfig_.playbackSpeed = speed;
        rocketConfig_.maxPlayback = speedIndex == 9;
        return true;
    }

    bool validateInputs(bool report) {
        warning_.fill(false);
        invalid_.fill(false);
        const auto specs = fieldSpecs();
        for (int i = 0; i < kFieldCount; ++i) {
            if (specs[i].label.empty()) continue;
            bool parsed{};
            readField(i, 0.0, &parsed);
            invalid_[i] = !parsed;
        }
        if (scenario_ == ScenarioKind::Satellite) {
            gnc::ClassicalOrbitElements e{readField(0, 0.0), readField(1, 0.0), readField(2, 0.0),
                                          readField(3, 0.0), readField(4, 0.0), readField(5, 0.0)};
            if (!gnc::validOrbitElements(e)) {
                for (int i = 0; i < 6; ++i) invalid_[i] = true;
            }
            for (int i : {6, 7, 8, 9, 10, 11, 12, 13}) if (readField(i, 0.0) <= 0.0) invalid_[i] = true;
            warning_[0] = e.semiMajorAxisKm < 6578.0 || e.semiMajorAxisKm > 42164.0;
            warning_[1] = e.eccentricity > 0.2;
            warning_[6] = readField(6, 0.0) < kRecommendedSatelliteMassMinKg
                       || readField(6, 0.0) > kRecommendedSatelliteMassMaxKg;
            for (int i : {7, 8, 9}) warning_[i] = readField(i, 0.0) < kRecommendedSatelliteInertiaMin
                                                 || readField(i, 0.0) > kRecommendedSatelliteInertiaMax;
            for (int i : {18, 19, 20}) warning_[i] = std::abs(readField(i, 0.0)) > 45.0;
            for (int i : {21, 22, 23}) warning_[i] = std::abs(readField(i, 0.0)) > 5.0;
            if (static_cast<gnc::SatelliteObjective>(SendMessageW(objectiveCombo_, CB_GETCURSEL, 0, 0))
                == gnc::SatelliteObjective::TargetTracking) {
                if (std::abs(readField(14, 0.0)) > 90.0) invalid_[14] = true;
                if (std::abs(readField(15, 0.0)) > 180.0) invalid_[15] = true;
            }
        } else {
            for (int i = 0; i < 6; ++i) if (readField(i, 0.0) <= -99.0) invalid_[i] = true;
            if (readField(13, 0.0) <= 0.0) invalid_[13] = true;
            for (int i = 0; i < 7; ++i) {
                warning_[i] = std::abs(readField(i, 0.0)) > kRecommendedRocketDeviationAbs[i];
            }
        }
        const bool ok = std::none_of(invalid_.begin(), invalid_.end(), [](bool value) { return value; });
        const bool hasWarning = std::any_of(warning_.begin(), warning_.end(), [](bool value) { return value; });
        if (!ok) validationText_ = L"参数无效，红色字段必须修正 / Invalid parameters: correct the red fields";
        else if (hasWarning) validationText_ = L"黄色字段超出建议范围，但仍可仿真 / Yellow fields exceed recommended ranges";
        else validationText_ = L"参数检查通过 / Parameters valid";
        for (HWND edit : edits_) InvalidateRect(edit, nullptr, TRUE);
        if (report && !ok) MessageBeep(MB_ICONWARNING);
        InvalidateRect(hwnd_, nullptr, FALSE);
        return ok;
    }

    void selectMission(bool updateDuration) {
        int siteIndex = siteCombo_ ? static_cast<int>(SendMessageW(siteCombo_, CB_GETCURSEL, 0, 0)) : 0;
        int orbitIndex = orbitCombo_ ? static_cast<int>(SendMessageW(orbitCombo_, CB_GETCURSEL, 0, 0)) : 0;
        if (siteIndex < 0) siteIndex = 0;
        if (orbitIndex < 0) orbitIndex = 0;
        const auto site = static_cast<gnc::LaunchSite>(siteIndex);
        const auto target = static_cast<gnc::TargetOrbit>(orbitIndex);
        rocketConfig_.mission = missions_.find(site, target);
        if (rocketConfig_.mission && updateDuration) {
            rocketConfig_.durationSec = rocketConfig_.mission->endTimeSec() + 120.0;
        } else if (rocketConfig_.mission && rocketConfig_.durationSec < 1.0) {
            rocketConfig_.durationSec = rocketConfig_.mission->endTimeSec() + 120.0;
        }
    }

    void resetSimulation(bool read = true) {
        if (read && !readConfig(true)) return;
        if (scenario_ == ScenarioKind::Satellite) satellite_.reset(satelliteConfig_);
        else rocket_.reset(rocketConfig_);
        runState_ = RunState::Ready;
        accumulator_ = 0.0;
        coastAutoSwitched_ = false;
        if (scenario_ == ScenarioKind::Rocket) {
            rocketGlobalView_ = false;
            followCraft_ = false;
            cameraMode_ = 0;
            resetCamera();
        }
        updateButtonText();
        InvalidateRect(hwnd_, nullptr, FALSE);
    }

    void onCommand(int id, int notification) {
        if (id >= kIdFirstEdit && id < kIdFirstEdit + kFieldCount && notification == EN_CHANGE) {
            if (!internalEdit_) validateInputs(false);
            return;
        }
        if (id == kIdObjective && notification == CBN_SELCHANGE) {
            satelliteConfig_.mission.objective = static_cast<gnc::SatelliteObjective>(
                SendMessageW(objectiveCombo_, CB_GETCURSEL, 0, 0));
            setFieldValues();
            return;
        }
        if ((id == kIdSite || id == kIdOrbit) && notification == CBN_SELCHANGE) {
            selectMission(true);
            setFieldValues();
            resetSimulation(false);
            return;
        }
        if (id == kIdSatellite || id == kIdRocket) {
            const ScenarioKind next = id == kIdSatellite ? ScenarioKind::Satellite : ScenarioKind::Rocket;
            if (next != scenario_) {
                scenario_ = next;
                runState_ = RunState::Ready;
                enlarged_ = false;
                cameraMode_ = 0;
                rocketGlobalView_ = false;
                followCraft_ = false;
                resetCamera();
                fillPlotCombos();
                updateTooltipTexts();
                setFieldValues();
                updateControlState();
                RECT client{}; GetClientRect(hwnd_, &client);
                layoutControls(client.right, client.bottom);
                InvalidateRect(hwnd_, nullptr, FALSE);
            }
            return;
        }
        if (id == kIdControl) {
            if (!readConfig(false)) return;
            if (scenario_ == ScenarioKind::Satellite) {
                gnc::gui::showControlSettings(hwnd_, scenario_, satelliteConfig_.control,
                    satelliteConfig_.inertiaKgM2,
                    {satelliteConfig_.wheelMaxTorqueNm, satelliteConfig_.wheelMaxTorqueNm,
                     satelliteConfig_.wheelMaxTorqueNm});
            } else if (rocketConfig_.mission) {
                const auto& s = rocketConfig_.mission->summary;
                const double inertiaScale = std::max(0.01, 1.0 + rocketConfig_.deviations.inertiaPercent / 100.0);
                const double thrustScale = std::max(0.01, 1.0 + rocketConfig_.deviations.thrustPercent / 100.0);
                const double torque = s.stage1ThrustN * thrustScale * s.thrustLeverArmM
                    * std::sin(s.maxTvcAngleDeg * gnc::kDegToRad);
                gnc::gui::showControlSettings(hwnd_, scenario_, rocketConfig_.control,
                    s.nominalWetInertiaKgM2 * inertiaScale,
                    {s.maxRollRcsTorqueNm, torque, torque});
            }
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id == kIdDisturbance) {
            readConfig(false);
            gnc::gui::showDisturbanceSettings(hwnd_, scenario_, satelliteConfig_.disturbances,
                                               rocketConfig_.disturbances);
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id == kIdGuidance && scenario_ == ScenarioKind::Rocket) {
            if (!readConfig(false)) return;
            gnc::gui::showRocketGuidanceSettings(hwnd_, rocketConfig_.guidance);
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id == kIdOrbitControl && scenario_ == ScenarioKind::Satellite) {
            if (!readConfig(false)) return;
            gnc::gui::showSatelliteOrbitControlSettings(hwnd_, satelliteConfig_.orbitControl);
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id == kIdPerturbation && scenario_ == ScenarioKind::Satellite) {
            gnc::gui::showPerturbationSettings(hwnd_, satelliteConfig_.perturbations);
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id == kIdStart) {
            resetSimulation();
            if (scenario_ == ScenarioKind::Satellite || rocket_.valid()) {
                runState_ = RunState::Running;
                lastTick_ = std::chrono::steady_clock::now();
                updateButtonText();
            }
            return;
        }
        if (id == kIdPause) {
            if (runState_ == RunState::Running) runState_ = RunState::Paused;
            else if (runState_ == RunState::Paused) {
                runState_ = RunState::Running;
                lastTick_ = std::chrono::steady_clock::now();
            }
            updateButtonText();
            return;
        }
        if (id == kIdReset) { resetSimulation(); return; }
        if (id == kIdExport) { exportData(); return; }
        if (id == kIdEnlarge) {
            enlarged_ = !enlarged_;
            updateButtonText();
            RECT client{}; GetClientRect(hwnd_, &client);
            layoutControls(client.right, client.bottom);
            InvalidateRect(hwnd_, nullptr, FALSE);
            return;
        }
        if (id >= kIdCameraFirst && id < kIdCameraFirst + 4) {
            cameraMode_ = id - kIdCameraFirst;
            if (scenario_ == ScenarioKind::Rocket) {
                if (cameraMode_ == 0) {
                    rocketGlobalView_ = false;
                    followCraft_ = false;
                } else if (cameraMode_ == 1) {
                    rocketGlobalView_ = true;
                    followCraft_ = false;
                } else if (cameraMode_ == 2) {
                    followCraft_ = !followCraft_;
                }
            } else {
                followCraft_ = cameraMode_ == 2;
            }
            resetCamera();
            updateButtonText();
            InvalidateRect(hwnd_, nullptr, FALSE);
        }
    }

    void updateControlState() {
        ShowWindow(buttons_[kIdGuidance], scenario_ == ScenarioKind::Rocket ? SW_SHOW : SW_HIDE);
        ShowWindow(buttons_[kIdOrbitControl], scenario_ == ScenarioKind::Satellite ? SW_SHOW : SW_HIDE);
        ShowWindow(buttons_[kIdPerturbation], scenario_ == ScenarioKind::Satellite ? SW_SHOW : SW_HIDE);
        EnableWindow(buttons_[kIdGuidance], scenario_ == ScenarioKind::Rocket);
        EnableWindow(buttons_[kIdOrbitControl], scenario_ == ScenarioKind::Satellite);
        EnableWindow(buttons_[kIdPerturbation], scenario_ == ScenarioKind::Satellite);
        SetWindowTextW(buttons_[kIdCameraFirst + 0], scenario_ == ScenarioKind::Satellite
            ? L"三维 / 3D" : L"局部 / Ascent");
        SetWindowTextW(buttons_[kIdCameraFirst + 1], scenario_ == ScenarioKind::Satellite
            ? L"轨道面 / Orbit" : L"全球 / Global");
        SetWindowTextW(buttons_[kIdCameraFirst + 2], L"跟随 / Follow");
        SetWindowTextW(buttons_[kIdCameraFirst + 3], scenario_ == ScenarioKind::Satellite
            ? L"俯视 / Top" : L"俯视 / Top");
    }

    void updateButtonText() {
        SetWindowTextW(buttons_[kIdPause], runState_ == RunState::Paused ? L"继续 / Resume" : L"暂停 / Pause");
        SetWindowTextW(buttons_[kIdStart], runState_ == RunState::Completed ? L"重新开始 / Restart" : L"开始 / Start");
        SetWindowTextW(buttons_[kIdEnlarge], enlarged_ ? L"还原 / Restore" : L"放大 / Enlarge");
        for (const auto& [id, button] : buttons_) {
            (void)id;
            InvalidateRect(button, nullptr, TRUE);
        }
    }

    void fillPlotCombos() {
        const std::vector<std::wstring> satelliteNames{
            L"参考/实际俯仰 / Ref vs actual pitch", L"姿态误差 / Attitude error",
            L"角速度 / Angular rate", L"控制力矩 / Control torque",
            L"飞轮转速 / Wheel speed", L"执行器饱和 / Saturation",
            L"指向误差 / Pointing error", L"轨道高度 / Orbit altitude",
            L"标称/实际轨道高度 / Nominal vs actual altitude",
            L"轨道位置误差 / Orbit position error",
            L"径向/航向误差 / Radial vs along-track error",
            L"法向位置误差 / Cross-track position error",
            L"轨道速度误差 / Orbit velocity error",
            L"轨控推力 / Orbit-control thrust",
            L"剩余推进剂 / Propellant remaining",
            L"累计速度增量 / Cumulative delta-v",
            L"轨道比能量误差 / Specific-energy error",
            L"半长轴误差 / Semi-major-axis error",
            L"偏心率误差 / Eccentricity error",
            L"倾角误差 / Inclination error"};
        const std::vector<std::wstring> rocketNames{
            L"标称/实际高度 / Nominal vs actual altitude",
            L"标称/实际速度 / Nominal vs actual velocity",
            L"标称/实际俯仰 / Nominal vs actual pitch",
            L"姿态误差 / Attitude error", L"角速度 / Angular rate",
            L"控制力矩 / Control torque", L"TVC 摆角 / TVC deflection",
            L"质量 / Mass", L"动压 / Dynamic pressure",
            L"位置误差 / Position error", L"横向误差 / Cross-track error",
            L"实际轨道高度 / Actual orbital altitude",
            L"实际半长轴 / Actual semi-major axis",
            L"实际偏心率 / Actual eccentricity",
            L"实际倾角 / Actual inclination",
            L"径向/航向位置误差 / Radial vs along-track error",
            L"制导俯仰/偏航修正 / Guidance pitch vs yaw correction",
            L"剩余推进剂 / Propellant remaining",
            L"轨道比能量误差 / Specific-energy error"};
        const auto& names = scenario_ == ScenarioKind::Satellite ? satelliteNames : rocketNames;
        const std::array<int, 4> satelliteDefaults{1, 9, 13, 8};
        const std::array<int, 4> rocketDefaults{2, 9, 6, 0};
        for (int plot = 0; plot < 4; ++plot) {
            if (!plotCombos_[plot]) continue;
            SendMessageW(plotCombos_[plot], CB_RESETCONTENT, 0, 0);
            for (const auto& name : names) {
                SendMessageW(plotCombos_[plot], CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(name.c_str()));
            }
            SendMessageW(plotCombos_[plot], CB_SETCURSEL,
                scenario_ == ScenarioKind::Satellite ? satelliteDefaults[plot] : rocketDefaults[plot], 0);
        }
    }

    void onTimer() {
        const auto now = std::chrono::steady_clock::now();
        const double wall = std::chrono::duration<double>(now - lastTick_).count();
        lastTick_ = now;
        if (runState_ != RunState::Running) return;
        constexpr double dt = 0.02;
        int steps{};
        if (rocketConfig_.maxPlayback) {
            steps = 5000;
        } else {
            const double playback = scenario_ == ScenarioKind::Satellite
                ? satellite_.config().playbackSpeed : rocket_.config().playbackSpeed;
            accumulator_ += std::min(0.10, wall) * playback;
            steps = std::min(5000, static_cast<int>(accumulator_ / dt));
            accumulator_ -= steps * dt;
        }
        for (int i = 0; i < steps; ++i) {
            if (scenario_ == ScenarioKind::Satellite) {
                satellite_.step(dt);
            } else {
                const bool wasCoast = rocket_.currentSample().orbitalCoast;
                rocket_.step(dt);
                if (!wasCoast && rocket_.currentSample().orbitalCoast && !coastAutoSwitched_) {
                    coastAutoSwitched_ = true;
                    rocketGlobalView_ = true;
                    followCraft_ = false;
                    cameraMode_ = 1;
                    resetCamera();
                    updateButtonText();
                }
            }
            if ((scenario_ == ScenarioKind::Satellite && satellite_.complete())
                || (scenario_ == ScenarioKind::Rocket && rocket_.complete())) {
                runState_ = RunState::Completed;
                accumulator_ = 0.0;
                updateButtonText();
                break;
            }
        }
        InvalidateRect(hwnd_, nullptr, FALSE);
    }

    const SimulationSample& sample() const {
        return scenario_ == ScenarioKind::Satellite ? satellite_.currentSample() : rocket_.currentSample();
    }

    const std::vector<SimulationSample>& history() const {
        return scenario_ == ScenarioKind::Satellite ? satellite_.history() : rocket_.history();
    }

    double duration() const {
        return scenario_ == ScenarioKind::Satellite ? satellite_.config().durationSec : rocket_.config().durationSec;
    }

    gnc::PerformanceMetrics metrics() const {
        return scenario_ == ScenarioKind::Satellite ? satellite_.metrics() : rocket_.metrics();
    }

    void exportData() {
        wchar_t path[MAX_PATH]{};
        wcscpy_s(path, scenario_ == ScenarioKind::Satellite
            ? L"SatelliteTelemetry_卫星遥测.csv" : L"RocketTelemetry_火箭遥测.csv");
        OPENFILENAMEW dialog{sizeof(dialog)};
        dialog.hwndOwner = hwnd_;
        dialog.lpstrFilter = L"CSV 数据文件 / CSV data (*.csv)\0*.csv\0所有文件 / All files (*.*)\0*.*\0";
        dialog.lpstrFile = path;
        dialog.nMaxFile = MAX_PATH;
        dialog.lpstrDefExt = L"csv";
        dialog.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST;
        if (GetSaveFileNameW(&dialog)) {
            const bool ok = gnc::exportCsv(path, scenario_, history());
            MessageBoxW(hwnd_, ok ? L"仿真数据已导出。\nSimulation data exported."
                                  : L"无法写入所选文件。\nUnable to write the selected file.",
                        L"AeroGNC Lab v3.2", MB_OK | (ok ? MB_ICONINFORMATION : MB_ICONERROR));
        }
    }

    std::wstring statusText() const {
        switch (runState_) {
        case RunState::Running: return L"● 运行中 / Running";
        case RunState::Paused: return L"Ⅱ 已暂停 / Paused";
        case RunState::Completed: return L"✓ 已完成 / Completed";
        default: return L"○ 就绪 / Ready";
        }
    }

    void drawButton(const DRAWITEMSTRUCT& item) {
        const int id = static_cast<int>(item.CtlID);
        const bool pressed = (item.itemState & ODS_SELECTED) != 0;
        const bool disabled = (item.itemState & ODS_DISABLED) != 0;
        const bool active = (id == kIdSatellite && scenario_ == ScenarioKind::Satellite)
                         || (id == kIdRocket && scenario_ == ScenarioKind::Rocket)
                         || (id == kIdPause && runState_ == RunState::Paused)
                         || (id >= kIdCameraFirst && id < kIdCameraFirst + 4
                             && id - kIdCameraFirst == cameraMode_);
        COLORREF background = active ? RGB(22, 88, 80) : kPanelRaised;
        COLORREF border = active ? kAccent : kBorder;
        if (id == kIdStart) { background = RGB(23, 103, 84); border = kAccent; }
        if (pressed) background = RGB(29, 65, 75);
        if (disabled) { background = RGB(14, 23, 31); border = RGB(28, 39, 47); }
        fillRounded(item.hDC, item.rcItem, background, 8, border);
        wchar_t label[96]{};
        GetWindowTextW(item.hwndItem, label, 95);
        drawText(item.hDC, label, item.rcItem, id >= kIdCameraFirst ? fontSmall_ : fontMedium_,
                 disabled ? RGB(72, 87, 97) : kText, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
        if (item.itemState & ODS_FOCUS) {
            RECT focus = item.rcItem; InflateRect(&focus, -4, -4); DrawFocusRect(item.hDC, &focus);
        }
    }

    void paint() {
        PAINTSTRUCT ps{};
        HDC windowDc = BeginPaint(hwnd_, &ps);
        RECT client{}; GetClientRect(hwnd_, &client);
        HDC dc = CreateCompatibleDC(windowDc);
        HBITMAP bitmap = CreateCompatibleBitmap(windowDc, std::max(1L, client.right), std::max(1L, client.bottom));
        const HGDIOBJ oldBitmap = SelectObject(dc, bitmap);
        fillSolid(dc, client, kBackground);
        drawHeader(dc, client);
        drawContent(dc, client);
        drawParameterPanel(dc, client);
        BitBlt(windowDc, 0, 0, client.right, client.bottom, dc, 0, 0, SRCCOPY);
        SelectObject(dc, oldBitmap);
        DeleteObject(bitmap);
        DeleteDC(dc);
        EndPaint(hwnd_, &ps);
    }

    void drawHeader(HDC dc, const RECT& client) {
        fillSolid(dc, {0, 0, client.right, 150}, RGB(7, 15, 23));
        line(dc, 0, 149, client.right, 149, kBorder);
        drawText(dc, L"AeroGNC Lab v3.2｜航天器GNC仿真实验平台",
                 {22, 8, 720, 58}, fontLarge_, kText,
                 DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
        const double progress = duration() > 0.0 ? gnc::clamp(sample().time / duration(), 0.0, 1.0) : 0.0;
        fillSolid(dc, {0, 146, client.right, 150}, RGB(12, 27, 37));
        fillSolid(dc, {0, 146, static_cast<LONG>(client.right * progress), 150}, kAccent);
    }

    void drawContent(HDC dc, const RECT& client) {
        const RECT content = contentRect(client.right, client.bottom);
        const int gap = 12;
        const int visualWidth = enlarged_ ? content.right - content.left
            : static_cast<int>((content.right - content.left - gap) * 0.37);
        RECT visual{content.left, content.top, content.left + visualWidth, content.bottom};
        drawVisualization(dc, visual);
        if (!enlarged_) {
            const auto cards = plotRects(client.right, client.bottom);
            for (int index = 0; index < 4; ++index) drawPlot(dc, cards[index], index);
        }
    }

    void drawVisualization(HDC dc, RECT rect) {
        fillRounded(dc, rect, kPanel, 13);
        drawText(dc, scenario_ == ScenarioKind::Satellite
            ? L"轨道与姿态三维视图 / Orbit & Attitude 3D View"
            : (rocketGlobalView_ ? L"上升与入轨全球视图 / Ascent & Orbit Global View"
                                 : L"标称与实际上升轨迹 / Nominal & Actual Ascent"),
            {rect.left + 15, rect.top + 8, rect.right - 150, rect.top + 38}, fontMedium_, kText);
        drawText(dc, L"T+ " + number(sample().time, 1) + L" s",
            {rect.right - 150, rect.top + 8, rect.right - 15, rect.top + 38}, fontMono_, kAccent,
            DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
        visualViewport_ = {rect.left + 9, rect.top + 81, rect.right - 9,
                           rect.bottom - 183};
        fillRounded(dc, visualViewport_, RGB(4, 12, 20), 9, RGB(19, 39, 52));
        const int savedDc = SaveDC(dc);
        IntersectClipRect(dc, visualViewport_.left + 1, visualViewport_.top + 1,
                         visualViewport_.right - 1, visualViewport_.bottom - 1);
        if (scenario_ == ScenarioKind::Satellite) drawSatellite(dc, visualViewport_);
        else drawRocket(dc, visualViewport_);
        RestoreDC(dc, savedDc);
        drawMetrics(dc, rect);
    }

    POINT project3d(const gnc::Vec3& p, const RECT& viewport, double scale, POINT center) const {
        (void)viewport;
        const double cy = std::cos(camera_.yaw);
        const double sy = std::sin(camera_.yaw);
        const double cp = std::cos(camera_.pitch);
        const double sp = std::sin(camera_.pitch);
        const double x = cy * p.x - sy * p.y;
        const double y = sy * p.x + cy * p.y;
        const double screenY = -(sp * y + cp * p.z);
        const double displayScale = scale * camera_.zoom;
        return {center.x + static_cast<LONG>(camera_.panX + x * displayScale),
                center.y + static_cast<LONG>(camera_.panY + screenY * displayScale)};
    }

    void drawEarth(HDC dc, POINT center, int radius) const {
        radius = std::max(2, radius);
        for (int current = radius; current > 0; current -= std::max(1, radius / 30)) {
            const double t = static_cast<double>(current) / radius;
            const COLORREF color = RGB(static_cast<int>(8 + 13 * (1.0 - t)),
                                       static_cast<int>(42 + 64 * (1.0 - t)),
                                       static_cast<int>(82 + 88 * (1.0 - t)));
            HBRUSH brush = CreateSolidBrush(color);
            HPEN noOutline = CreatePen(PS_NULL, 0, color);
            const HGDIOBJ oldBrush = SelectObject(dc, brush);
            const HGDIOBJ oldPen = SelectObject(dc, noOutline);
            Ellipse(dc, center.x - current, center.y - current,
                    center.x + current, center.y + current);
            SelectObject(dc, oldPen);
            SelectObject(dc, oldBrush);
            DeleteObject(noOutline);
            DeleteObject(brush);
        }
    }

    void drawStars(HDC dc, const RECT& viewport) const {
        const int width = std::max(20L, viewport.right - viewport.left);
        const int height = std::max(20L, viewport.bottom - viewport.top);
        for (int i = 0; i < 58; ++i) {
            const int x = viewport.left + 7 + (i * 97 + i * i * 13) % (width - 14);
            const int y = viewport.top + 7 + (i * 53 + i * i * 7) % (height - 14);
            SetPixel(dc, x, y, i % 7 == 0 ? RGB(105, 154, 177) : RGB(43, 70, 85));
        }
    }

    void drawSatellite(HDC dc, const RECT& viewport) {
        drawStars(dc, viewport);
        const int width = viewport.right - viewport.left;
        const int height = viewport.bottom - viewport.top;
        POINT center{(viewport.left + viewport.right) / 2, (viewport.top + viewport.bottom) / 2};
        gnc::ClassicalOrbitElements actualElements = gnc::cartesianToClassicalElements(
            sample().position, sample().velocity);
        if (!gnc::validOrbitElements(actualElements)) actualElements = satelliteConfig_.orbit;
        const double referenceApogee = satelliteConfig_.orbit.semiMajorAxisKm * 1000.0
                                     * (1.0 + satelliteConfig_.orbit.eccentricity);
        const double actualApogee = actualElements.semiMajorAxisKm * 1000.0
                                  * (1.0 + actualElements.eccentricity);
        double scale = (std::min(width, height) * 0.43)
                     / std::max({1.0, referenceApogee, actualApogee});
        const gnc::Vec3 origin = followCraft_ ? sample().position : gnc::Vec3{};
        if (followCraft_) scale *= 2.2;

        std::vector<POINT> referenceOrbit;
        std::vector<POINT> actualOrbit;
        referenceOrbit.reserve(181);
        actualOrbit.reserve(181);
        for (int i = 0; i <= 160; ++i) {
            auto reference = satelliteConfig_.orbit;
            auto actual = actualElements;
            reference.trueAnomalyDeg = actual.trueAnomalyDeg = 360.0 * i / 160.0;
            referenceOrbit.push_back(project3d(
                gnc::classicalElementsToCartesian(reference).positionEciM - origin,
                viewport, scale, center));
            actualOrbit.push_back(project3d(
                gnc::classicalElementsToCartesian(actual).positionEciM - origin,
                viewport, scale, center));
        }
        HPEN orbitPen = CreatePen(PS_DASH, 1, RGB(58, 99, 122));
        const HGDIOBJ oldPen = SelectObject(dc, orbitPen);
        Polyline(dc, referenceOrbit.data(), static_cast<int>(referenceOrbit.size()));
        SelectObject(dc, oldPen);
        DeleteObject(orbitPen);
        orbitPen = CreatePen(PS_SOLID, 2, kAccent);
        const HGDIOBJ oldActualPen = SelectObject(dc, orbitPen);
        Polyline(dc, actualOrbit.data(), static_cast<int>(actualOrbit.size()));
        SelectObject(dc, oldActualPen);
        DeleteObject(orbitPen);

        const POINT earthCenter = project3d(-origin, viewport, scale, center);
        drawEarth(dc, earthCenter, static_cast<int>(gnc::kEarthEquatorialRadiusM * scale * camera_.zoom));
        const POINT craft = project3d(sample().position - origin, viewport, scale, center);
        const auto axisPoint = [&](const gnc::Vec3& axis, int length) {
            const gnc::Vec3 end = sample().position + sample().attitude.rotate(axis) / scale * length;
            return project3d(end - origin, viewport, scale, center);
        };
        const auto directionPoint = [&](const gnc::Vec3& directionEci, int length) {
            const gnc::Vec3 end = sample().position + directionEci.normalized() / scale * length;
            return project3d(end - origin, viewport, scale, center);
        };
        const POINT nominalCraft = project3d(sample().nominalPosition - origin, viewport, scale, center);
        line(dc, craft.x, craft.y, nominalCraft.x, nominalCraft.y, kAmber, 1, PS_DOT);
        HBRUSH nominalBrush = CreateSolidBrush(kCyan);
        HGDIOBJ oldBrush = SelectObject(dc, nominalBrush);
        Ellipse(dc, nominalCraft.x - 3, nominalCraft.y - 3, nominalCraft.x + 4, nominalCraft.y + 4);
        SelectObject(dc, oldBrush);
        DeleteObject(nominalBrush);
        const POINT xAxis = axisPoint({1, 0, 0}, 24);
        const POINT yAxis = axisPoint({0, 1, 0}, 28);
        const POINT zAxis = axisPoint({0, 0, 1}, 38);
        const POINT targetAxis = directionPoint(sample().referenceAttitude.rotate({0, 0, 1}), 48);
        const POINT orbitDirection = directionPoint(sample().velocity, 42);
        line(dc, craft.x - (yAxis.x - craft.x), craft.y - (yAxis.y - craft.y), yAxis.x, yAxis.y, kCyan, 6);
        RECT bus{craft.x - 7, craft.y - 7, craft.x + 7, craft.y + 7};
        fillRounded(dc, bus, RGB(203, 217, 221), 3, RGB(244, 248, 249));
        line(dc, craft.x, craft.y, xAxis.x, xAxis.y, kRed, 2);
        line(dc, craft.x, craft.y, zAxis.x, zAxis.y, kAccent, 2);
        line(dc, craft.x, craft.y, targetAxis.x, targetAxis.y, kCyan, 1, PS_DOT);
        line(dc, craft.x, craft.y, orbitDirection.x, orbitDirection.y, kAmber, 2);
        if (sample().thrust > 1.0e-12) {
            const gnc::Vec3 forceEci = sample().attitude.rotate(sample().orbitControlForceBodyN);
            const POINT thrustDirection = directionPoint(forceEci, 36);
            line(dc, craft.x, craft.y, thrustDirection.x, thrustDirection.y, kPurple, 3);
        }
        drawText(dc, L"左键旋转 · 滚轮缩放 · 右键平移 · 双击重置 / L-drag rotate · Wheel zoom · R-drag pan · Double-click reset",
                 {viewport.left + 12, viewport.top + 7, viewport.right - 12, viewport.top + 25}, fontTiny_, kMuted);
        drawText(dc, L"绿色实轨 / Green actual · 灰虚线名义轨道 / Dashed nominal · 紫色轨控推力 / Purple orbit thrust",
                 {viewport.left + 12, viewport.top + 24, viewport.right - 12, viewport.top + 43}, fontTiny_, kMuted);
        const double perigeeKm = actualElements.semiMajorAxisKm * (1.0 - actualElements.eccentricity)
                               - gnc::kEarthEquatorialRadiusM / 1000.0;
        const double apogeeKm = actualElements.semiMajorAxisKm * (1.0 + actualElements.eccentricity)
                              - gnc::kEarthEquatorialRadiusM / 1000.0;
        drawText(dc, L"高度 / ALT " + number(sample().altitude / 1000.0, 2) + L" km · 轨差 / Δr "
                 + number(sample().positionError, 2) + L" m · 指向 / Δatt "
                 + number(attitudeErrorDeg(sample()), 3) + L"°",
                 {viewport.left + 12, viewport.bottom - 44, viewport.right - 12, viewport.bottom - 25},
                 fontMono_, kText);
        drawText(dc, L"实轨 / Actual: a " + number(actualElements.semiMajorAxisKm, 1) + L" km · e "
                 + compact(actualElements.eccentricity, 4) + L" · 近/远地点 Pe/Ap "
                 + number(perigeeKm, 1) + L" / " + number(apogeeKm, 1) + L" km",
                 {viewport.left + 12, viewport.bottom - 25, viewport.right - 12, viewport.bottom - 5},
                 fontTiny_, kAccent);
    }

    gnc::Vec3 rocketLocal(const gnc::Vec3& eci) const {
        if (!rocketConfig_.mission || rocketConfig_.mission->trajectory.empty()) return {};
        const auto& trajectory = rocketConfig_.mission->trajectory;
        const gnc::Vec3 origin = trajectory.front().positionEciM;
        const gnc::Vec3 up = origin.normalized();
        gnc::Vec3 down = trajectory.back().positionEciM - origin;
        down -= up * down.dot(up);
        if (down.normSquared() < 1.0) {
            down = trajectory.front().velocityEciMps - up * trajectory.front().velocityEciMps.dot(up);
        }
        down = down.normalized();
        const gnc::Vec3 cross = up.cross(down).normalized();
        const gnc::Vec3 delta = eci - origin;
        return {delta.dot(down), delta.dot(cross), eci.norm() - origin.norm()};
    }

    void drawRocketGlobal(HDC dc, const RECT& viewport) {
        const auto& nominal = rocketConfig_.mission->trajectory;
        drawStars(dc, viewport);
        const auto& targetState = nominal.back();
        const gnc::ClassicalOrbitElements targetElements = gnc::cartesianToClassicalElements(
            targetState.positionEciM, targetState.velocityEciMps);
        const gnc::ClassicalOrbitElements actualElements = gnc::cartesianToClassicalElements(
            sample().position, sample().velocity);
        const bool actualOrbitValid = sample().orbitalCoast && gnc::validOrbitElements(actualElements);
        const double targetApogee = targetElements.semiMajorAxisKm * 1000.0
                                  * (1.0 + targetElements.eccentricity);
        const double actualApogee = actualOrbitValid
            ? actualElements.semiMajorAxisKm * 1000.0 * (1.0 + actualElements.eccentricity) : 0.0;
        const double radiusMax = std::max({gnc::kEarthEquatorialRadiusM, targetApogee, actualApogee});
        double scale = std::min(viewport.right - viewport.left, viewport.bottom - viewport.top) * 0.43
                     / std::max(1.0, radiusMax);
        const gnc::Vec3 origin = followCraft_ ? sample().position : gnc::Vec3{};
        if (followCraft_) scale *= 2.2;
        const POINT center{(viewport.left + viewport.right) / 2,
                           (viewport.top + viewport.bottom) / 2};
        const auto project = [&](const gnc::Vec3& eci) {
            return project3d(eci - origin, viewport, scale, center);
        };
        const auto drawPolyline = [&](const std::vector<POINT>& points, COLORREF color,
                                      int width, int style) {
            if (points.size() < 2) return;
            HPEN pen = CreatePen(style, width, color);
            const HGDIOBJ oldPen = SelectObject(dc, pen);
            Polyline(dc, points.data(), static_cast<int>(points.size()));
            SelectObject(dc, oldPen);
            DeleteObject(pen);
        };
        const auto elementOrbit = [&](gnc::ClassicalOrbitElements elements) {
            std::vector<POINT> points;
            points.reserve(181);
            for (int i = 0; i <= 180; ++i) {
                elements.trueAnomalyDeg = i * 2.0;
                points.push_back(project(gnc::classicalElementsToCartesian(elements).positionEciM));
            }
            return points;
        };

        if (gnc::validOrbitElements(targetElements)) {
            drawPolyline(elementOrbit(targetElements), RGB(77, 111, 131), 1, PS_DASH);
        }
        if (actualOrbitValid) {
            drawPolyline(elementOrbit(actualElements), RGB(28, 124, 111), 1, PS_DOT);
        }

        std::vector<POINT> nominalAscent;
        nominalAscent.reserve(nominal.size() / 2 + 1);
        for (std::size_t i = 0; i < nominal.size(); i += 2) {
            nominalAscent.push_back(project(nominal[i].positionEciM));
        }
        drawPolyline(nominalAscent, kPurple, 1, PS_DASH);

        const POINT earthCenter = project({});
        drawEarth(dc, earthCenter,
                  static_cast<int>(gnc::kEarthEquatorialRadiusM * scale * camera_.zoom));

        std::vector<POINT> actualHistory;
        std::vector<POINT> nominalCoast;
        const std::size_t stride = std::max<std::size_t>(1, history().size() / 1200);
        for (std::size_t i = 0; i < history().size(); i += stride) {
            actualHistory.push_back(project(history()[i].position));
            if (history()[i].orbitalCoast) nominalCoast.push_back(project(history()[i].nominalPosition));
        }
        if (!history().empty() && (history().size() - 1) % stride != 0) {
            actualHistory.push_back(project(history().back().position));
            if (history().back().orbitalCoast) nominalCoast.push_back(project(history().back().nominalPosition));
        }
        drawPolyline(nominalCoast, kCyan, 1, PS_DASH);
        drawPolyline(actualHistory, kAccent, 2, PS_SOLID);

        const POINT launch = project(nominal.front().positionEciM);
        const POINT targetInsertion = project(targetState.positionEciM);
        HBRUSH markerBrush = CreateSolidBrush(kAmber);
        HGDIOBJ oldBrush = SelectObject(dc, markerBrush);
        Ellipse(dc, launch.x - 3, launch.y - 3, launch.x + 4, launch.y + 4);
        SelectObject(dc, oldBrush);
        DeleteObject(markerBrush);
        markerBrush = CreateSolidBrush(kCyan);
        oldBrush = SelectObject(dc, markerBrush);
        Ellipse(dc, targetInsertion.x - 4, targetInsertion.y - 4,
                targetInsertion.x + 5, targetInsertion.y + 5);
        SelectObject(dc, oldBrush);
        DeleteObject(markerBrush);
        if (rocket_.hasInsertionSnapshot()) {
            const POINT insertion = project(rocket_.insertionSample().position);
            markerBrush = CreateSolidBrush(kPurple);
            oldBrush = SelectObject(dc, markerBrush);
            Ellipse(dc, insertion.x - 4, insertion.y - 4, insertion.x + 5, insertion.y + 5);
            SelectObject(dc, oldBrush);
            DeleteObject(markerBrush);
        }

        const POINT craft = project(sample().position);
        POINT body[3]{{craft.x + 8, craft.y}, {craft.x - 6, craft.y - 5}, {craft.x - 6, craft.y + 5}};
        HBRUSH bodyBrush = CreateSolidBrush(RGB(232, 239, 241));
        HPEN bodyPen = CreatePen(PS_SOLID, 1, kAccent);
        oldBrush = SelectObject(dc, bodyBrush);
        const HGDIOBJ oldPen = SelectObject(dc, bodyPen);
        Polygon(dc, body, 3);
        SelectObject(dc, oldPen);
        SelectObject(dc, oldBrush);
        DeleteObject(bodyPen);
        DeleteObject(bodyBrush);

        drawText(dc, L"左键旋转 · 滚轮缩放 · 右键平移 · 双击重置 / L-drag rotate · Wheel zoom · R-drag pan · Double-click reset",
                 {viewport.left + 12, viewport.top + 7, viewport.right - 12, viewport.top + 25},
                 fontTiny_, kMuted);
        drawText(dc, L"紫虚线上升标称 / Purple nominal ascent · 绿色实际 / Green actual · 灰虚线目标轨道 / Dashed target orbit",
                 {viewport.left + 12, viewport.top + 24, viewport.right - 12, viewport.top + 43},
                 fontTiny_, kMuted);
        const std::wstring phase = sample().orbitalCoast ? L"轨道滑行 / Orbital coast"
            : (sample().terminalGuidanceActive ? L"终端轨道制导 / Terminal orbit guidance"
            : (sample().stageId == 1 ? L"一级动力 / Stage 1 powered"
               : (sample().stageId == 3 ? L"二级动力 / Stage 2 powered" : L"级间滑行 / Interstage coast")));
        drawText(dc, phase,
                 {viewport.left + 12, viewport.bottom - 44, viewport.right - 12, viewport.bottom - 25},
                 fontMono_, sample().orbitalCoast ? kPurple : kText);
        if (actualOrbitValid) {
            const double perigee = actualElements.semiMajorAxisKm * (1.0 - actualElements.eccentricity)
                                 - gnc::kEarthEquatorialRadiusM / 1000.0;
            const double apogee = actualElements.semiMajorAxisKm * (1.0 + actualElements.eccentricity)
                                - gnc::kEarthEquatorialRadiusM / 1000.0;
            drawText(dc, L"实际轨道 / Actual: a " + number(actualElements.semiMajorAxisKm, 1)
                     + L" km · e " + compact(actualElements.eccentricity, 4) + L" · i "
                     + number(actualElements.inclinationDeg, 2) + L"° · Pe/Ap "
                     + number(perigee, 1) + L" / " + number(apogee, 1) + L" km",
                     {viewport.left + 12, viewport.bottom - 25, viewport.right - 12, viewport.bottom - 5},
                     fontTiny_, kAccent);
        } else {
            drawText(dc, L"高度 / ALT " + number(sample().altitude / 1000.0, 2)
                     + L" km · 速度 / V " + number(sample().speed / 1000.0, 3) + L" km/s",
                     {viewport.left + 12, viewport.bottom - 25, viewport.right - 12, viewport.bottom - 5},
                     fontTiny_, kText);
        }
    }

    void drawRocket(HDC dc, const RECT& viewport) {
        if (!rocketConfig_.mission) return;
        if (rocketGlobalView_) {
            drawRocketGlobal(dc, viewport);
            return;
        }
        const auto& nominal = rocketConfig_.mission->trajectory;
        double horizontalMax = 1000.0;
        double altitudeMax = 1000.0;
        for (std::size_t i = 0; i < nominal.size(); i += 8) {
            const gnc::Vec3 p = rocketLocal(nominal[i].positionEciM);
            horizontalMax = std::max(horizontalMax, std::hypot(p.x, p.y));
            altitudeMax = std::max(altitudeMax, std::abs(p.z));
        }
        const double sx = (viewport.right - viewport.left) * 0.39 / horizontalMax;
        const double sz = (viewport.bottom - viewport.top) * 0.36 / altitudeMax;
        (void)sx;
        (void)sz;
        double scale = std::min(viewport.right - viewport.left, viewport.bottom - viewport.top) * 0.42;
        POINT center{viewport.left + 35, viewport.bottom - 40};
        gnc::Vec3 offset{};
        if (followCraft_) {
            offset = rocketLocal(sample().position);
            scale *= 2.7;
            center = {(viewport.left + viewport.right) / 2, (viewport.top + viewport.bottom) / 2};
        }
        const auto project = [&](const gnc::Vec3& eci) {
            const gnc::Vec3 local = rocketLocal(eci) - offset;
            const gnc::Vec3 normalized{local.x / horizontalMax, local.y / horizontalMax,
                                       local.z / altitudeMax};
            return project3d(normalized, viewport, scale, center);
        };
        for (int i = 0; i < 7; ++i) {
            const int y = viewport.bottom - 30 - i * std::max<LONG>(8, (viewport.bottom - viewport.top - 70) / 7);
            line(dc, viewport.left + 8, y, viewport.right - 8, y, kGrid, 1, PS_DOT);
        }
        std::vector<POINT> nominalPoints;
        nominalPoints.reserve(nominal.size() / 2 + 1);
        for (std::size_t i = 0; i < nominal.size(); i += 2) nominalPoints.push_back(project(nominal[i].positionEciM));
        HPEN nominalPen = CreatePen(PS_DASH, 1, RGB(84, 117, 132));
        HGDIOBJ oldPen = SelectObject(dc, nominalPen);
        if (nominalPoints.size() > 1) Polyline(dc, nominalPoints.data(), static_cast<int>(nominalPoints.size()));
        SelectObject(dc, oldPen); DeleteObject(nominalPen);

        std::vector<POINT> actualPoints;
        actualPoints.reserve(history().size());
        for (const auto& point : history()) actualPoints.push_back(project(point.position));
        HPEN actualPen = CreatePen(PS_SOLID, 2, kAccent);
        oldPen = SelectObject(dc, actualPen);
        if (actualPoints.size() > 1) Polyline(dc, actualPoints.data(), static_cast<int>(actualPoints.size()));
        SelectObject(dc, oldPen); DeleteObject(actualPen);
        if (sample().orbitalCoast) {
            std::vector<POINT> nominalCoast;
            for (const auto& point : history()) {
                if (point.orbitalCoast) nominalCoast.push_back(project(point.nominalPosition));
            }
            if (nominalCoast.size() > 1) {
                HPEN coastPen = CreatePen(PS_SOLID, 2, kCyan);
                oldPen = SelectObject(dc, coastPen);
                Polyline(dc, nominalCoast.data(), static_cast<int>(nominalCoast.size()));
                SelectObject(dc, oldPen); DeleteObject(coastPen);
            }
        }

        const POINT actual = project(sample().position);
        const POINT reference = project(sample().nominalPosition);
        line(dc, actual.x, actual.y, reference.x, reference.y, kAmber, 1, PS_DOT);
        HBRUSH refBrush = CreateSolidBrush(kCyan);
        HGDIOBJ oldBrush = SelectObject(dc, refBrush);
        Ellipse(dc, reference.x - 3, reference.y - 3, reference.x + 4, reference.y + 4);
        SelectObject(dc, oldBrush); DeleteObject(refBrush);

        const double pitchDeg = std::isfinite(sample().actualPitchDeg) ? sample().actualPitchDeg
            : (rocket_.hasInsertionSnapshot() ? rocket_.insertionSample().actualPitchDeg : 0.0);
        const double pitch = pitchDeg * gnc::kDegToRad;
        double dx = std::cos(pitch);
        double dy = -std::sin(pitch);
        if (cameraMode_ == 3) dy = 0.0;
        const double length = std::max(1.0e-9, std::hypot(dx, dy)); dx /= length; dy /= length;
        const double px = -dy, py = dx;
        POINT body[5]{{actual.x + static_cast<LONG>(dx * 25), actual.y + static_cast<LONG>(dy * 25)},
                      {actual.x + static_cast<LONG>(dx * 13 + px * 5), actual.y + static_cast<LONG>(dy * 13 + py * 5)},
                      {actual.x - static_cast<LONG>(dx * 18 - px * 5), actual.y - static_cast<LONG>(dy * 18 - py * 5)},
                      {actual.x - static_cast<LONG>(dx * 18 + px * 5), actual.y - static_cast<LONG>(dy * 18 + py * 5)},
                      {actual.x + static_cast<LONG>(dx * 13 - px * 5), actual.y + static_cast<LONG>(dy * 13 - py * 5)}};
        HBRUSH bodyBrush = CreateSolidBrush(RGB(215, 224, 227));
        HPEN bodyPen = CreatePen(PS_SOLID, 1, RGB(246, 249, 250));
        oldBrush = SelectObject(dc, bodyBrush); oldPen = SelectObject(dc, bodyPen);
        Polygon(dc, body, 5);
        SelectObject(dc, oldPen); SelectObject(dc, oldBrush);
        DeleteObject(bodyPen); DeleteObject(bodyBrush);
        if (sample().thrust > 0.0) {
            const POINT tail{actual.x - static_cast<LONG>(dx * 18), actual.y - static_cast<LONG>(dy * 18)};
            line(dc, tail.x, tail.y, tail.x - static_cast<int>(dx * 32), tail.y - static_cast<int>(dy * 32), kAmber, 4);
        }
        drawText(dc, L"左键旋转 · 滚轮缩放 · 右键平移 · 双击重置 / L-drag rotate · Wheel zoom · R-drag pan · Double-click reset",
                 {viewport.left + 12, viewport.top + 7, viewport.right - 12, viewport.top + 25}, fontTiny_, kMuted);
        drawText(dc, L"虚线标称 / Dashed nominal · 绿色实际 / Green actual · 黄色误差 / Amber error",
                 {viewport.left + 12, viewport.top + 24, viewport.right - 12, viewport.top + 43}, fontTiny_, kMuted);
        const std::wstring phase = sample().orbitalCoast ? L"轨道滑行 / Orbital coast"
            : (sample().terminalGuidanceActive ? L"终端轨道制导 / Terminal orbit guidance"
            : (sample().stageId == 1 ? L"一级动力 / Stage 1 powered"
               : (sample().stageId == 3 ? L"二级动力 / Stage 2 powered" : L"级间滑行 / Interstage coast")));
        drawText(dc, phase,
                 {viewport.left + 12, viewport.bottom - 43, viewport.right - 12, viewport.bottom - 25},
                 fontMono_, sample().orbitalCoast ? kPurple : kText);
        const SimulationSample& tracking = sample().ascentTrackingActive || !rocket_.hasInsertionSnapshot()
            ? sample() : rocket_.insertionSample();
        drawText(dc, std::wstring(sample().orbitalCoast ? L"入轨冻结 / Frozen at insertion · " : L"")
                 + L"位置 / Δr " + number(tracking.positionError, 1) + L" m · 横向 / Cross "
                 + number(tracking.crossTrackError, 1) + L" m · 高度 / Δh " + number(tracking.altitudeError, 1)
                 + L" m · 姿态 / Δatt " + number(attitudeErrorDeg(tracking), 3) + L"°",
                 {viewport.left + 12, viewport.bottom - 25, viewport.right - 12, viewport.bottom - 5},
                 fontTiny_, kText);
    }

    SeriesPoint plotValue(const SimulationSample& s, int selection) const {
        if (scenario_ == ScenarioKind::Satellite) {
            switch (selection) {
            case 0: return {s.referenceEulerDeg.y, s.eulerDeg.y, true};
            case 1: return {attitudeErrorDeg(s), vecMaxAbs(s.attitudeErrorDeg), true};
            case 2: return {vecMaxAbs(s.angularRateDegPerSec), 0.0, false};
            case 3: return {vecMaxAbs(s.controlTorque), 0.0, false};
            case 4: return {vecMaxAbs(s.wheelSpeedRpm), 0.0, false};
            case 5: return {s.actuatorSaturated ? 1.0 : 0.0, 0.0, false};
            case 6: return {attitudeErrorDeg(s), 0.0, false};
            case 7: return {s.altitude / 1000.0, 0.0, false};
            case 8: return {s.nominalAltitude / 1000.0, s.altitude / 1000.0, true};
            case 9: return {s.positionError, 0.0, false};
            case 10: return {s.orbitPositionErrorRtnM.x, s.orbitPositionErrorRtnM.y, true};
            case 11: return {s.orbitPositionErrorRtnM.z, 0.0, false};
            case 12: return {s.velocityError, 0.0, false};
            case 13: return {s.thrust, 0.0, false};
            case 14: return {s.propellantRemaining, 0.0, false};
            case 15: return {s.cumulativeDeltaVMps, 0.0, false};
            case 16: return {s.specificEnergyError / 1000.0, 0.0, false};
            default: {
                const gnc::ClassicalOrbitElements actual = gnc::cartesianToClassicalElements(
                    s.position, s.velocity);
                const gnc::ClassicalOrbitElements nominal = gnc::cartesianToClassicalElements(
                    s.nominalPosition, s.nominalVelocity);
                if (!gnc::validOrbitElements(actual) || !gnc::validOrbitElements(nominal)) {
                    return {std::numeric_limits<double>::quiet_NaN(), 0.0, false};
                }
                if (selection == 17) return {actual.semiMajorAxisKm - nominal.semiMajorAxisKm, 0.0, false};
                if (selection == 18) return {actual.eccentricity - nominal.eccentricity, 0.0, false};
                return {actual.inclinationDeg - nominal.inclinationDeg, 0.0, false};
            }
            }
        }
        switch (selection) {
        case 0: return {s.nominalAltitude / 1000.0, s.altitude / 1000.0, true};
        case 1: return {s.nominalSpeed / 1000.0, s.speed / 1000.0, true};
        case 2: return {s.nominalPitchDeg, s.actualPitchDeg, true};
        case 3: return {attitudeErrorDeg(s), 0.0, false};
        case 4: return {vecMaxAbs(s.angularRateDegPerSec), 0.0, false};
        case 5: return {vecMaxAbs(s.controlTorque) / 1000.0, 0.0, false};
        case 6: return {s.tvcPitchDeg, s.tvcYawDeg, true};
        case 7: return {s.mass / 1000.0, 0.0, false};
        case 8: return {s.dynamicPressure / 1000.0, 0.0, false};
        case 9: return {s.positionError, 0.0, false};
        case 10: return {s.crossTrackError, 0.0, false};
        case 11: return {s.altitude / 1000.0, 0.0, false};
        case 15: return {s.radialPositionError, s.alongTrackPositionError, true};
        case 16: return {s.guidanceCorrectionDeg.y, s.guidanceCorrectionDeg.z, true};
        case 17: return {s.propellantRemaining / 1000.0, 0.0, false};
        case 18: return {s.specificEnergyError / 1000.0, 0.0, false};
        default: {
            const gnc::ClassicalOrbitElements elements = gnc::cartesianToClassicalElements(
                s.position, s.velocity);
            if (!gnc::validOrbitElements(elements)) {
                return {std::numeric_limits<double>::quiet_NaN(), 0.0, false};
            }
            if (selection == 12) return {elements.semiMajorAxisKm, 0.0, false};
            if (selection == 13) return {elements.eccentricity, 0.0, false};
            return {elements.inclinationDeg, 0.0, false};
        }
        }
    }

    bool plotApplicable(const SimulationSample& s, int selection) const {
        if (scenario_ == ScenarioKind::Satellite) return true;
        const bool ascentTracking = selection == 0 || selection == 1 || selection == 2
                                 || selection == 3 || selection == 9 || selection == 10
                                 || selection == 15 || selection == 16;
        if (ascentTracking && !s.ascentTrackingActive) return false;
        if (selection >= 12 && selection <= 14 && !s.orbitalCoast) return false;
        return true;
    }

    bool plotNonnegative(int selection) const {
        if (scenario_ == ScenarioKind::Satellite) {
            switch (selection) {
            case 0: case 10: case 11: case 16: case 17: case 18: case 19:
                return false;
            default:
                return true;
            }
        }
        switch (selection) {
        case 0: case 1: case 3: case 4: case 5: case 7: case 8: case 9:
        case 11: case 12: case 13: case 14: case 17:
            return true;
        default:
            return false;
        }
    }

    static double niceTickStep(double range, int targetTicks = 5) {
        const double raw = std::max(1.0e-12, range / std::max(1, targetTicks));
        const double magnitude = std::pow(10.0, std::floor(std::log10(raw)));
        const double fraction = raw / magnitude;
        const double nice = fraction <= 1.0 ? 1.0 : (fraction <= 2.0 ? 2.0
                             : (fraction <= 5.0 ? 5.0 : 10.0));
        return nice * magnitude;
    }

    std::wstring plotUnit(int selection) const {
        if (scenario_ == ScenarioKind::Satellite) {
            const std::array<const wchar_t*, 20> units{
                L"deg", L"deg", L"deg/s", L"N·m", L"rpm", L"0/1", L"deg", L"km",
                L"km", L"m", L"m", L"m", L"m/s", L"N", L"kg", L"m/s",
                L"kJ/kg", L"km", L"", L"deg"};
            return units[std::clamp(selection, 0, 19)];
        }
        const std::array<const wchar_t*, 19> units{L"km", L"km/s", L"deg", L"deg", L"deg/s", L"kN·m",
                                                   L"deg", L"t", L"kPa", L"m", L"m", L"km",
                                                   L"km", L"", L"deg", L"m", L"deg", L"t", L"kJ/kg"};
        return units[std::clamp(selection, 0, 18)];
    }

    void drawPlot(HDC dc, RECT rect, int index) {
        fillRounded(dc, rect, kPanel, 12);
        RECT chart{rect.left + 54, rect.top + 47, rect.right - 13, rect.bottom - 25};
        if (chart.right <= chart.left || chart.bottom <= chart.top) return;
        for (int i = 0; i <= 5; ++i) {
            const int x = chart.left + i * (chart.right - chart.left) / 5;
            line(dc, x, chart.top, x, chart.bottom, kGrid, 1, PS_DOT);
        }
        const int selection = std::max(0, static_cast<int>(SendMessageW(plotCombos_[index], CB_GETCURSEL, 0, 0)));
        const auto& samples = history();
        if (samples.empty()) return;
        double minimum = std::numeric_limits<double>::infinity();
        double maximum = -std::numeric_limits<double>::infinity();
        bool dual{};
        for (const auto& s : samples) {
            if (!plotApplicable(s, selection)) continue;
            const SeriesPoint value = plotValue(s, selection);
            if (std::isfinite(value.first)) {
                minimum = std::min(minimum, value.first);
                maximum = std::max(maximum, value.first);
            }
            if (value.dual && std::isfinite(value.second)) {
                minimum = std::min(minimum, value.second);
                maximum = std::max(maximum, value.second);
                dual = true;
            }
        }
        if (!std::isfinite(minimum) || !std::isfinite(maximum)) {
            drawText(dc, L"当前阶段无适用数据 / No applicable data in the current phase",
                     chart, fontSmall_, kMuted, DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            return;
        }
        const bool nonnegative = plotNonnegative(selection);
        if (nonnegative) minimum = 0.0;
        else {
            minimum = std::min(0.0, minimum);
            maximum = std::max(0.0, maximum);
        }
        if (maximum - minimum < 1.0e-12) {
            maximum = nonnegative ? std::max(1.0, maximum * 1.2) : minimum + 1.0;
        }
        double tickStep = niceTickStep(maximum - minimum);
        minimum = nonnegative ? 0.0 : std::floor(minimum / tickStep) * tickStep;
        maximum = std::ceil(maximum / tickStep) * tickStep;
        if (maximum <= minimum) maximum = minimum + tickStep;
        tickStep = niceTickStep(maximum - minimum);

        int tickGuard{};
        const double firstTick = std::ceil((minimum - tickStep * 1.0e-9) / tickStep) * tickStep;
        for (double value = firstTick; value <= maximum + tickStep * 1.0e-8 && tickGuard++ < 12;
             value += tickStep) {
            const int y = chart.bottom - static_cast<int>((value - minimum) / (maximum - minimum)
                                                        * (chart.bottom - chart.top));
            const bool zero = std::abs(value) <= tickStep * 1.0e-8;
            line(dc, chart.left, y, chart.right, y, zero ? RGB(91, 111, 122) : kGrid,
                 zero ? 2 : 1, zero ? PS_SOLID : PS_DOT);
            drawText(dc, compact(zero ? 0.0 : value, 2),
                     {rect.left + 4, y - 10, chart.left - 5, y + 10}, fontTiny_,
                     zero ? kText : kMuted, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
        }
        const double timeMax = std::max(1.0, duration());
        const std::size_t stride = std::max<std::size_t>(1, samples.size() / std::max(1L, chart.right - chart.left));
        const auto build = [&](bool second) {
            std::vector<POINT> points;
            for (std::size_t i = 0; i < samples.size(); i += stride) {
                if (!plotApplicable(samples[i], selection)) continue;
                const SeriesPoint value = plotValue(samples[i], selection);
                const double v = second ? value.second : value.first;
                if (!std::isfinite(v) || (second && !value.dual)) continue;
                points.push_back({chart.left + static_cast<LONG>(samples[i].time / timeMax * (chart.right - chart.left)),
                    chart.bottom - static_cast<LONG>((v - minimum) / (maximum - minimum) * (chart.bottom - chart.top))});
            }
            if ((samples.size() - 1) % stride != 0 && plotApplicable(samples.back(), selection)) {
                const auto& s = samples.back(); const SeriesPoint value = plotValue(s, selection);
                const double v = second ? value.second : value.first;
                if (std::isfinite(v) && (!second || value.dual)) {
                    points.push_back({chart.left + static_cast<LONG>(s.time / timeMax * (chart.right - chart.left)),
                        chart.bottom - static_cast<LONG>((v - minimum) / (maximum - minimum) * (chart.bottom - chart.top))});
                }
            }
            return points;
        };
        const auto first = build(false);
        HPEN pen = CreatePen(PS_SOLID, 2, kAccent); HGDIOBJ old = SelectObject(dc, pen);
        if (first.size() > 1) Polyline(dc, first.data(), static_cast<int>(first.size()));
        SelectObject(dc, old); DeleteObject(pen);
        if (dual) {
            const auto second = build(true);
            pen = CreatePen(PS_SOLID, 1, kCyan); old = SelectObject(dc, pen);
            if (second.size() > 1) Polyline(dc, second.data(), static_cast<int>(second.size()));
            SelectObject(dc, old); DeleteObject(pen);
        }
        const SimulationSample* latest = nullptr;
        for (auto it = samples.rbegin(); it != samples.rend(); ++it) {
            if (!plotApplicable(*it, selection)) continue;
            const SeriesPoint value = plotValue(*it, selection);
            if (std::isfinite(value.first)) { latest = &*it; break; }
        }
        if (!latest) return;
        const SeriesPoint current = plotValue(*latest, selection);
        const bool frozen = scenario_ == ScenarioKind::Rocket && sample().orbitalCoast
                         && !plotApplicable(sample(), selection);
        const std::wstring legend = (frozen ? L"入轨冻结 / Frozen at insertion  " : L"当前 / Now ")
            + compact(current.first, 3)
            + (current.dual && std::isfinite(current.second) ? L"  /  " + compact(current.second, 3) : L"")
            + (plotUnit(selection).empty() ? L"" : L" " + plotUnit(selection));
        drawText(dc, legend, {chart.left, chart.bottom + 2, chart.right, rect.bottom - 3}, fontTiny_, kMuted,
                 DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }

    void drawMetrics(HDC dc, const RECT& rect) {
        const auto m = metrics();
        const bool controlOff = (scenario_ == ScenarioKind::Satellite ? satelliteConfig_.control.mode
                                                                       : rocketConfig_.control.mode)
                              == gnc::ControlMode::Off;
        std::vector<std::pair<std::wstring, std::wstring>> values;
        if (scenario_ == ScenarioKind::Satellite) {
            const std::wstring orbitStatus = !satelliteConfig_.orbitControl.enabled ? L"关闭 / OFF"
                : (sample().propellantDepleted ? L"推进剂耗尽 / Depleted"
                   : (sample().guidanceActive ? L"正在修正 / Correcting" : L"死区待机 / Deadband"));
            values = {{L"均方根姿态误差 / RMS attitude error", number(m.rmsAttitudeErrorDeg, 3) + L"°"},
                       {L"最大姿态误差 / Max attitude error", number(m.maxAttitudeErrorDeg, 3) + L"°"},
                       {L"飞轮峰值 / Peak wheel speed", controlOff ? L"不适用 / N/A"
                           : number(m.maxActuatorValue, 1) + L" rpm"},
                       {L"执行机构饱和 / Actuator saturation", m.saturated ? L"是 / Yes" : L"否 / No"},
                       {L"当前指向误差 / Pointing error", number(attitudeErrorDeg(sample()), 4) + L"°"},
                       {L"最大轨道位置误差 / Max orbit position error", number(m.maxPositionError, 2) + L" m"},
                       {L"当前轨道位置误差 / Current position error", number(m.finalPositionError, 3) + L" m"},
                       {L"当前轨道速度误差 / Current velocity error", number(m.finalVelocityError, 5) + L" m/s"},
                       {L"累计速度增量 / Cumulative delta-v", number(m.cumulativeDeltaVMps, 4) + L" m/s"},
                       {L"剩余推进剂 / Propellant remaining", number(sample().propellantRemaining, 4) + L" kg"},
                       {L"轨道控制状态 / Orbit-control status", orbitStatus}};
        } else {
            const bool inserted = rocket_.hasInsertionSnapshot();
            const auto insertionValue = [&](double value, int precision, const wchar_t* unit) {
                return inserted ? number(value, precision) + unit : std::wstring(L"待入轨 / Pending");
            };
            values = {{L"均方根姿态误差 / RMS attitude error", number(m.rmsAttitudeErrorDeg, 3) + L"°"},
                       {L"最大姿态误差 / Max attitude error", number(m.maxAttitudeErrorDeg, 3) + L"°"},
                       {L"最大 TVC / Peak TVC", controlOff ? L"不适用 / N/A" : number(m.maxActuatorValue, 3) + L"°"},
                       {L"TVC 饱和 / TVC saturation", controlOff ? L"不适用 / N/A"
                           : (m.saturated ? L"是 / Yes" : L"否 / No")},
                       {L"上升段最大位置误差 / Max ascent position error", number(m.maxPositionError, 1) + L" m"},
                       {L"入轨高度误差 / Insertion altitude error",
                           insertionValue(m.terminalAltitudeError, 1, L" m")},
                       {L"入轨速度误差 / Insertion velocity error",
                           insertionValue(m.terminalVelocityError, 2, L" m/s")},
                       {L"入轨轨道根数误差 / Insertion element error",
                           inserted ? L"Δa " + number(m.terminalSemiMajorAxisErrorKm, 2) + L" km\nΔe "
                               + compact(m.terminalEccentricityError, 2) + L" · Δi "
                               + number(m.terminalInclinationErrorDeg, 3) + L"°"
                                    : L"待入轨 / Pending"},
                       {L"最大动压 / Max dynamic pressure", number(m.maxDynamicPressure / 1000.0, 2) + L" kPa"},
                       {L"末端质量 / Final mass", number(m.finalMass / 1000.0, 3) + L" t"},
                       {L"轨迹制导状态 / Trajectory guidance",
                           !rocketConfig_.guidance.enabled ? L"关闭 / OFF"
                           : (sample().terminalGuidanceActive ? L"终端制导 / Terminal"
                              : (sample().guidanceActive ? L"轨迹跟踪 / Tracking"
                                 : (sample().orbitalCoast ? L"入轨关机 / Cutoff" : L"待机 / Standby")))},
                       {L"剩余推进剂 / Propellant remaining",
                           number(sample().propellantRemaining / 1000.0, 3) + L" t"}};
        }
        const int width = (rect.right - rect.left - 30) / 4;
        const int rows = (static_cast<int>(values.size()) + 3) / 4;
        const int blockHeight = 55;
        const int firstTop = rect.bottom - 10 - rows * blockHeight;
        for (int i = 0; i < static_cast<int>(values.size()); ++i) {
            const int row = i / 4;
            const int column = i % 4;
            const int x = rect.left + 10 + column * width;
            const int y = firstTop + row * blockHeight;
            const std::size_t separator = values[i].first.find(L" / ");
            const std::wstring chinese = separator == std::wstring::npos
                ? values[i].first : values[i].first.substr(0, separator);
            const std::wstring english = separator == std::wstring::npos
                ? L"" : values[i].first.substr(separator + 3);
            drawText(dc, chinese, {x, y, x + width - 6, y + 14}, fontTiny_, kMuted,
                     DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            drawText(dc, english, {x, y + 12, x + width - 6, y + 27}, fontTiny_, kMuted,
                     DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            const std::size_t valueBreak = values[i].second.find(L'\n');
            if (valueBreak == std::wstring::npos) {
                drawText(dc, values[i].second, {x, y + 28, x + width - 6, y + 54}, fontMono_,
                         m.saturated && i == 3 ? kAmber : kText,
                         DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            } else {
                drawText(dc, values[i].second.substr(0, valueBreak),
                         {x, y + 27, x + width - 6, y + 41}, fontTiny_, kText,
                         DT_CENTER | DT_VCENTER | DT_SINGLELINE);
                drawText(dc, values[i].second.substr(valueBreak + 1),
                         {x, y + 40, x + width - 6, y + 55}, fontTiny_, kText,
                         DT_CENTER | DT_VCENTER | DT_SINGLELINE);
            }
        }
    }

    void drawParameterPanel(HDC dc, const RECT& client) {
        const int top = bottomTop(client.bottom);
        fillSolid(dc, {0, top, client.right, client.bottom}, RGB(7, 15, 23));
        line(dc, 0, top, client.right, top, kBorder);
        drawText(dc, scenario_ == ScenarioKind::Satellite
            ? L"卫星任务与模型参数 / Satellite Mission & Model Parameters"
            : L"火箭真值模型偏差 / Launch Vehicle Truth-model Deviations",
            {24, top + 7, 555, top + 34}, fontMedium_, kText);
        drawText(dc, statusText(), {client.right - 300, top + 7, client.right - 24, top + 34}, fontMedium_,
                 runState_ == RunState::Completed ? kAccent : (runState_ == RunState::Paused ? kAmber : kCyan),
                 DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
        if (scenario_ == ScenarioKind::Satellite) {
            drawText(dc, L"姿态目标 / Attitude objective", {24, top + 39, 215, top + 68}, fontSmall_, kMuted);
        } else {
            drawText(dc, L"发射场 / Launch site", {24, top + 39, 180, top + 68}, fontSmall_, kMuted);
            drawText(dc, L"固定目标轨道 / Fixed target orbit", {438, top + 39, 530, top + 68}, fontSmall_, kMuted,
                     DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
            if (rocketConfig_.mission) {
                const auto& s = rocketConfig_.mission->summary;
                const std::wstring nominal = L"标称只读 / Nominal read-only: 湿质量 Wet mass "
                    + number(s.nominalWetMassKg() / 1000.0, 1) + L" t · 一级推力 S1 thrust "
                    + number(s.stage1ThrustN / 1000.0, 0) + L" kN · 二级推力 S2 thrust "
                    + number(s.stage2ThrustN / 1000.0, 0) + L" kN · 最大 TVC Max TVC "
                    + number(s.maxTvcAngleDeg, 1) + L"° · 标称末端 Nominal end "
                    + number(rocketConfig_.mission->endTimeSec(), 1) + L" s";
                drawText(dc, nominal, {600, top + 7, client.right - 315, top + 34}, fontTiny_, kAccent,
                         DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
            }
        }
        drawText(dc, L"播放速度 / Playback", {client.right - 380, top + 39, client.right - 212, top + 68},
                 fontSmall_, kMuted, DT_RIGHT | DT_VCENTER | DT_SINGLELINE);

        const auto fields = fieldSpecs();
        const int margin = 24, gap = 9, columns = 6;
        const int columnWidth = (client.right - 2 * margin - gap * (columns - 1)) / columns;
        for (int index = 0; index < kFieldCount; ++index) {
            if (fields[index].label.empty()) continue;
            const int row = index / columns;
            const int column = index % columns;
            const int x = margin + column * (columnWidth + gap);
            drawText(dc, fields[index].label, {x, top + 78 + row * 57, x + columnWidth, top + 100 + row * 57},
                     fontTiny_, invalid_[index] ? kRed : (warning_[index] ? kAmber : kMuted),
                     DT_LEFT | DT_VCENTER | DT_SINGLELINE | DT_END_ELLIPSIS);
        }
        drawText(dc, validationText_, {24, client.bottom - 35, client.right / 2, client.bottom - 10},
                 fontSmall_, std::any_of(invalid_.begin(), invalid_.end(), [](bool v) { return v; }) ? kRed : kMuted);
        drawText(dc, L"固定物理步长 0.02 s / Fixed physics step · SI 内核 / SI core · RK4 · 参数在开始时生效 / Applied on Start",
                 {client.right / 2, client.bottom - 35, client.right - 24, client.bottom - 10}, fontTiny_, kMuted,
                 DT_RIGHT | DT_VCENTER | DT_SINGLELINE);
    }

private:
    HINSTANCE instance_{};
    HWND hwnd_{};
    std::array<HWND, kFieldCount> edits_{};
    std::array<HWND, 4> plotCombos_{};
    HWND objectiveCombo_{};
    HWND siteCombo_{};
    HWND orbitCombo_{};
    HWND playbackCombo_{};
    HWND tooltip_{};
    std::unordered_map<int, HWND> buttons_{};
    HFONT fontTiny_{};
    HFONT fontSmall_{};
    HFONT fontNormal_{};
    HFONT fontMedium_{};
    HFONT fontLarge_{};
    HFONT fontMono_{};
    HBRUSH editBrush_{};
    HBRUSH warningBrush_{};
    HBRUSH invalidBrush_{};
    HBRUSH listBrush_{};
    gnc::MissionRepository missions_{};
    gnc::SatelliteConfig satelliteConfig_{};
    gnc::RocketMissionConfig rocketConfig_{};
    gnc::SatelliteSimulation satellite_;
    gnc::RocketMissionSimulation rocket_;
    ScenarioKind scenario_{ScenarioKind::Satellite};
    RunState runState_{RunState::Ready};
    std::array<bool, kFieldCount> warning_{};
    std::array<bool, kFieldCount> invalid_{};
    std::array<std::wstring, kFieldCount> tooltipTexts_{};
    std::wstring validationText_{L"参数检查通过 / Parameters valid"};
    bool internalEdit_{};
    bool enlarged_{};
    bool rocketGlobalView_{};
    bool followCraft_{};
    bool coastAutoSwitched_{};
    bool cameraDragging_{};
    bool cameraPanning_{};
    int cameraMode_{};
    CameraState camera_{};
    POINT cameraLastPoint_{};
    RECT visualViewport_{};
    double accumulator_{};
    std::chrono::steady_clock::time_point lastTick_{};
};

} // namespace

int WINAPI wWinMain(HINSTANCE instance, HINSTANCE, PWSTR commandLine, int showCommand) {
    if (commandLine && std::wstring(commandLine).find(L"--verify-embedded-data") != std::wstring::npos) {
        std::unordered_map<std::string, std::string> embeddedFiles;
        std::string error;
        gnc::MissionRepository repository;
        return loadEmbeddedMissionFiles(instance, embeddedFiles, error)
            && repository.loadEmbedded(embeddedFiles, error) && repository.loaded() ? 0 : 2;
    }
    INITCOMMONCONTROLSEX controls{sizeof(controls), ICC_STANDARD_CLASSES};
    InitCommonControlsEx(&controls);
    Application application(instance);
    if (!application.create(showCommand)) return 1;
    MSG message{};
    while (GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }
    return static_cast<int>(message.wParam);
}
