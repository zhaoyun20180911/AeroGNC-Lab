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

// Converts the project's conventional "中文 / English" label to the active
// language. Numeric separators such as "1.0 / 2.0" are intentionally retained.
inline std::wstring localizeBilingual(std::wstring_view value) {
    const std::size_t separator = value.find(L" / ");
    if (separator == std::wstring_view::npos) return std::wstring(value);
    const std::wstring_view left = value.substr(0, separator);
    const std::wstring_view right = value.substr(separator + 3);
    if (!containsHan(left) || !containsLatin(right)) return std::wstring(value);
    return std::wstring(englishUi() ? right : left);
}

} // namespace gnc::gui
