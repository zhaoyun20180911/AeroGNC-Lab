#include "gnc/recovery.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace gnc {
namespace {

bool readValues(const std::string& line, std::vector<double>& values) {
    values.clear();
    std::stringstream stream(line);
    std::string item;
    while (std::getline(stream, item, ',')) {
        if (!item.empty() && item.back() == '\r') item.pop_back();
        try {
            std::size_t used{};
            const double value = std::stod(item, &used);
            if (used != item.size() || !std::isfinite(value)) return false;
            values.push_back(value);
        } catch (...) {
            return false;
        }
    }
    return true;
}

} // namespace

bool exportRecoveryInitialState(const std::wstring& path, const RecoveryInitialState& state,
                                const LandingSite& site) {
    std::ofstream stream(std::filesystem::path(path), std::ios::binary);
    if (!stream) return false;
    stream << "AEROSYS_RECOVERY_STATE_V1\r\n" << std::setprecision(17)
           << state.time << ',' << state.positionEcefM.x << ',' << state.positionEcefM.y << ','
           << state.positionEcefM.z << ',' << state.velocityEcefMps.x << ','
           << state.velocityEcefMps.y << ',' << state.velocityEcefMps.z << ','
           << state.bodyToEcef.w << ',' << state.bodyToEcef.x << ',' << state.bodyToEcef.y << ','
           << state.bodyToEcef.z << ',' << state.angularRateBodyRadPerSec.x << ','
           << state.angularRateBodyRadPerSec.y << ',' << state.angularRateBodyRadPerSec.z << ','
           << state.massTotalKg << ',' << state.propellantMassKg << "\r\n"
           << site.latitudeDeg << ',' << site.longitudeDeg << ',' << site.altitudeM << "\r\n";
    return static_cast<bool>(stream);
}

bool importRecoveryInitialState(const std::wstring& path, RecoveryInitialState& state,
                                LandingSite& site) {
    std::ifstream stream(std::filesystem::path(path), std::ios::binary);
    if (!stream) return false;
    std::string magic, stateLine, siteLine;
    if (!std::getline(stream, magic) || !std::getline(stream, stateLine)
        || !std::getline(stream, siteLine)) return false;
    if (!magic.empty() && magic.back() == '\r') magic.pop_back();
    if (magic != "AEROSYS_RECOVERY_STATE_V1") return false;
    std::vector<double> values;
    if (!readValues(stateLine, values) || values.size() != 16) return false;
    std::vector<double> siteValues;
    if (!readValues(siteLine, siteValues) || siteValues.size() != 3) return false;
    RecoveryInitialState parsed{};
    parsed.time = values[0];
    parsed.positionEcefM = {values[1], values[2], values[3]};
    parsed.velocityEcefMps = {values[4], values[5], values[6]};
    parsed.bodyToEcef = Quaternion{values[7], values[8], values[9], values[10]}.normalized();
    parsed.angularRateBodyRadPerSec = {values[11], values[12], values[13]};
    parsed.massTotalKg = values[14];
    parsed.propellantMassKg = values[15];
    LandingSite parsedSite{siteValues[0], siteValues[1], siteValues[2]};
    if (parsed.positionEcefM.norm() < kRecoveryEarthRadiusM * 0.5
        || parsed.massTotalKg <= 0.0 || parsed.propellantMassKg < 0.0
        || std::abs(parsedSite.latitudeDeg) > 90.0
        || std::abs(parsedSite.longitudeDeg) > 180.0) return false;
    state = parsed;
    site = parsedSite;
    return true;
}

} // namespace gnc
