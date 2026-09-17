#pragma once

#include <string>
#include <string_view>

namespace gnc::gui {

enum class UiLanguage { Chinese, English };

inline UiLanguage gUiLanguage = UiLanguage::Chinese;

inline UiLanguage uiLanguage() { return gUiLanguage; }
inline bool englishUi() { return gUiLanguage == UiLanguage::English; }
inline void setUiLanguage(UiLanguage language) { gUiLanguage = language; }

inline bool containsHan(std::wstring_view value) {
    for (const wchar_t ch : value) {
        if (ch >= 0x3400 && ch <= 0x9fff) return true;
    }
    return false;
}

inline bool containsLatin(std::wstring_view value) {
    for (const wchar_t ch : value) {
        if ((ch >= L'A' && ch <= L'Z') || (ch >= L'a' && ch <= L'z')) return true;
    }
    return false;
}

inline std::wstring tr(std::wstring_view chinese, std::wstring_view english) {
    return std::wstring(englishUi() ? english : chinese);
}

inline std::wstring_view trailingParenthesizedSuffix(std::wstring_view value) {
    while (!value.empty() && (value.back() == L' ' || value.back() == L'\t')) {
        value.remove_suffix(1);
    }
    if (value.empty() || value.back() != L')') return {};

    int depth = 0;
    for (std::size_t index = value.size(); index-- > 0;) {
        if (value[index] == L')') ++depth;
        else if (value[index] == L'(' && --depth == 0) return value.substr(index);
    }
    return {};
}

inline bool isDisplayUnitSuffix(std::wstring_view suffix) {
    return suffix == L"(km)" || suffix == L"(m)" || suffix == L"(kg)"
        || suffix == L"(s)" || suffix == L"(Hz)" || suffix == L"(rpm)"
        || suffix == L"(deg)" || suffix == L"(deg/s)" || suffix == L"(m/s)"
        || suffix == L"(kN)" || suffix == L"(N)" || suffix == L"(N·m)"
        || suffix == L"(kg·m²)" || suffix == L"(%)" || suffix == L"(J/kg)"
        || suffix == L"(km/s)" || suffix == L"(kPa)" || suffix == L"(MPa)"
        || suffix == L"(t)" || suffix == L"(ms)" || suffix == L"(kN·m)"
        || suffix == L"(kJ/kg)"
        || suffix == L"(s⁻¹)" || suffix == L"(s⁻²)" || suffix == L"(rad·s)"
        || suffix == L"(N·m/rad)" || suffix == L"(N·m/(rad·s))"
        || suffix == L"(N·m·s/rad)";
}

// Converts the project's conventional "中文 / English" label to the active
// language. Numeric separators such as "1.0 / 2.0" are intentionally retained.
inline std::wstring localizeBilingual(std::wstring_view value) {
    const std::size_t separator = value.find(L" / ");
    if (separator == std::wstring_view::npos) return std::wstring(value);
    const std::wstring_view left = value.substr(0, separator);
    const std::wstring_view right = value.substr(separator + 3);
    if (!containsHan(left) || !containsLatin(right)) return std::wstring(value);
    if (englishUi()) return std::wstring(right);

    std::wstring localized(left);
    const std::wstring_view suffix = trailingParenthesizedSuffix(right);
    if (isDisplayUnitSuffix(suffix) && left.find(suffix) == std::wstring_view::npos) {
        localized.push_back(L' ');
        localized += suffix;
    }
    return localized;
}

} // namespace gnc::gui
