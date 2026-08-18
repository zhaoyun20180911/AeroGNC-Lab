#include "gnc/mission.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace gnc {
namespace {

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> fields;
    std::size_t start = 0;
    while (start <= line.size()) {
        const std::size_t comma = line.find(',', start);
        fields.push_back(line.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    if (!fields.empty() && !fields.back().empty() && fields.back().back() == '\r') fields.back().pop_back();
    return fields;
}

using ColumnMap = std::unordered_map<std::string, std::size_t>;

ColumnMap columnsFromHeader(const std::string& header) {
    ColumnMap columns;
    const auto names = splitCsvLine(header);
    for (std::size_t index = 0; index < names.size(); ++index) columns[names[index]] = index;
    return columns;
}

const std::string& value(const std::vector<std::string>& row, const ColumnMap& columns,
                         const std::string& name) {
    const auto iterator = columns.find(name);
    if (iterator == columns.end() || iterator->second >= row.size()) {
        throw std::runtime_error("Missing CSV column: " + name);
    }
    return row[iterator->second];
}

double number(const std::vector<std::string>& row, const ColumnMap& columns,
              const std::string& name) {
    return std::stod(value(row, columns, name));
}

int integer(const std::vector<std::string>& row, const ColumnMap& columns,
            const std::string& name) {
    return std::stoi(value(row, columns, name));
}

LaunchSite parseSite(const std::string& site) {
    if (site == "Wenchang") return LaunchSite::Wenchang;
    if (site == "Xichang") return LaunchSite::Xichang;
    if (site == "Cape Canaveral") return LaunchSite::CapeCanaveral;
    throw std::runtime_error("Unsupported launch site: " + site);
}

TargetOrbit parseTarget(double altitudeKm, double inclinationDeg) {
    if (std::abs(altitudeKm - 300.0) < 1.0 && std::abs(inclinationDeg - 28.5) < 0.2) {
        return TargetOrbit::Leo300Km28_5Deg;
    }
    if (std::abs(altitudeKm - 500.0) < 1.0 && std::abs(inclinationDeg - 51.6) < 0.2) {
        return TargetOrbit::Leo500Km51_6Deg;
    }
    throw std::runtime_error("Unsupported target orbit in mission_summary.csv");
}

std::vector<RocketMissionSummary> parseSummaries(std::istream& stream) {
    std::string line;
    if (!std::getline(stream, line)) throw std::runtime_error("mission_summary.csv is empty");
    const ColumnMap columns = columnsFromHeader(line);
    std::vector<RocketMissionSummary> summaries;
    while (std::getline(stream, line)) {
        if (line.empty()) continue;
        const auto row = splitCsvLine(line);
        RocketMissionSummary summary{};
        summary.missionId = integer(row, columns, "mission_id");
        summary.site = parseSite(value(row, columns, "site"));
        summary.siteLatitudeDeg = number(row, columns, "site_latitude_deg");
        summary.siteLongitudeDeg = number(row, columns, "site_longitude_deg");
        summary.siteAltitudeM = number(row, columns, "site_altitude_m");
        summary.targetAltitudeKm = number(row, columns, "target_altitude_km");
        summary.targetInclinationDeg = number(row, columns, "target_inclination_deg");
        summary.payloadMassKg = number(row, columns, "payload_mass_kg");
        summary.stage1DryMassKg = number(row, columns, "stage1_dry_mass_kg");
        summary.stage1PropellantMassKg = number(row, columns, "stage1_prop_mass_kg");
        summary.stage1ThrustN = number(row, columns, "stage1_thrust_N");
        summary.stage1SpecificImpulseSec = number(row, columns, "stage1_Isp_s");
        summary.stage1DragCoefficient = number(row, columns, "stage1_Cd");
        summary.stage1ReferenceAreaM2 = number(row, columns, "stage1_ref_area_m2");
        summary.stage2DryMassKg = number(row, columns, "stage2_dry_mass_kg");
        summary.stage2PropellantMassKg = number(row, columns, "stage2_prop_mass_kg");
        summary.stage2ThrustN = number(row, columns, "stage2_thrust_N");
        summary.stage2SpecificImpulseSec = number(row, columns, "stage2_Isp_s");
        summary.stage2DragCoefficient = number(row, columns, "stage2_Cd");
        summary.stage2ReferenceAreaM2 = number(row, columns, "stage2_ref_area_m2");
        summary.nominalWetInertiaKgM2 = {
            number(row, columns, "nominal_Ix_wet_kgm2"), number(row, columns, "nominal_Iy_wet_kgm2"),
            number(row, columns, "nominal_Iz_wet_kgm2")};
        summary.nominalDryInertiaKgM2 = {
            number(row, columns, "nominal_Ix_dry_kgm2"), number(row, columns, "nominal_Iy_dry_kgm2"),
            number(row, columns, "nominal_Iz_dry_kgm2")};
        summary.maxTvcAngleDeg = number(row, columns, "max_tvc_angle_deg");
        summary.maxTvcRateDegPerSec = number(row, columns, "max_tvc_rate_degps");
        summary.thrustLeverArmM = number(row, columns, "thrust_lever_arm_m");
        summary.maxRollRcsTorqueNm = number(row, columns, "max_roll_rcs_torque_Nm");
        summary.stage1BurnSec = number(row, columns, "stage1_burn_s");
        summary.stage2ActualBurnSec = number(row, columns, "stage2_actual_burn_s");
        summary.finalAltitudeKm = number(row, columns, "final_altitude_km");
        summary.finalSpeedMps = number(row, columns, "final_speed_mps");
        summary.finalRadialVelocityMps = number(row, columns, "final_radial_velocity_mps");
        summary.finalTangentialVelocityMps = number(row, columns, "final_tangential_velocity_mps");
        summary.finalSemiMajorAxisKm = number(row, columns, "semi_major_axis_km");
        summary.finalEccentricity = number(row, columns, "eccentricity");
        summary.finalInclinationDeg = number(row, columns, "inclination_deg");
        summary.finalRaanDeg = number(row, columns, "raan_deg");
        summary.finalPerigeeAltitudeKm = number(row, columns, "perigee_altitude_km");
        summary.finalApogeeAltitudeKm = number(row, columns, "apogee_altitude_km");
        summary.maxDynamicPressureKPa = number(row, columns, "max_dynamic_pressure_kPa");
        summary.finalMassKg = number(row, columns, "final_mass_kg");
        summary.success = integer(row, columns, "success_flag") != 0;
        summaries.push_back(summary);
    }
    return summaries;
}

std::vector<RocketMissionSummary> loadSummaries(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("Unable to open mission_summary.csv");
    return parseSummaries(stream);
}

std::vector<NominalTrajectoryPoint> parseTrajectory(std::istream& stream) {
    std::string line;
    if (!std::getline(stream, line)) throw std::runtime_error("Trajectory CSV is empty");
    const ColumnMap columns = columnsFromHeader(line);
    std::vector<NominalTrajectoryPoint> points;
    points.reserve(5000);
    Quaternion previous{};
    bool hasPrevious = false;
    while (std::getline(stream, line)) {
        if (line.empty()) continue;
        const auto row = splitCsvLine(line);
        NominalTrajectoryPoint point{};
        point.timeSec = number(row, columns, "time_s");
        point.positionEciM = {number(row, columns, "r_eci_x_m"), number(row, columns, "r_eci_y_m"),
                              number(row, columns, "r_eci_z_m")};
        point.velocityEciMps = {number(row, columns, "v_eci_x_mps"), number(row, columns, "v_eci_y_mps"),
                                number(row, columns, "v_eci_z_mps")};
        point.latitudeDeg = number(row, columns, "latitude_deg");
        point.longitudeDeg = number(row, columns, "longitude_deg");
        point.altitudeM = number(row, columns, "altitude_m");
        point.downrangeM = number(row, columns, "downrange_m");
        point.crossrangeM = number(row, columns, "crossrange_m");
        point.speedMps = number(row, columns, "speed_mps");
        point.radialVelocityMps = number(row, columns, "radial_velocity_mps");
        point.tangentialVelocityMps = number(row, columns, "tangential_velocity_mps");
        point.massKg = number(row, columns, "mass_kg");
        point.stageId = integer(row, columns, "stage_id");
        point.thrustN = number(row, columns, "thrust_N");
        point.pitchReferenceDeg = number(row, columns, "pitch_ref_deg");
        point.azimuthReferenceDeg = number(row, columns, "azimuth_ref_deg");
        point.referenceAttitude = Quaternion{
            number(row, columns, "qw_ref"), number(row, columns, "qx_ref"),
            number(row, columns, "qy_ref"), number(row, columns, "qz_ref")}.normalized();
        if (hasPrevious) {
            const double dot = previous.w * point.referenceAttitude.w + previous.x * point.referenceAttitude.x
                             + previous.y * point.referenceAttitude.y + previous.z * point.referenceAttitude.z;
            if (dot < 0.0) {
                point.referenceAttitude = {-point.referenceAttitude.w, -point.referenceAttitude.x,
                                           -point.referenceAttitude.y, -point.referenceAttitude.z};
            }
        }
        previous = point.referenceAttitude;
        hasPrevious = true;
        point.referenceAngularRateRadPerSec = {
            number(row, columns, "omega_ref_x_radps"), number(row, columns, "omega_ref_y_radps"),
            number(row, columns, "omega_ref_z_radps")};
        point.densityKgPerM3 = number(row, columns, "rho_kgpm3");
        point.dynamicPressurePa = number(row, columns, "dynamic_pressure_Pa");
        point.dragEciN = {number(row, columns, "drag_x_N"), number(row, columns, "drag_y_N"),
                          number(row, columns, "drag_z_N")};
        points.push_back(point);
    }
    if (points.size() < 2) throw std::runtime_error("Trajectory requires at least two samples");
    return points;
}

std::vector<NominalTrajectoryPoint> loadTrajectory(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) throw std::runtime_error("Unable to open trajectory: " + path.string());
    return parseTrajectory(stream);
}

double interpolateScalar(double a, double b, double fraction) {
    return a + (b - a) * fraction;
}

Vec3 interpolateVector(const Vec3& a, const Vec3& b, double fraction) {
    return a + (b - a) * fraction;
}

} // namespace

double RocketMissionData::startTimeSec() const {
    return trajectory.empty() ? 0.0 : trajectory.front().timeSec;
}

double RocketMissionData::endTimeSec() const {
    return trajectory.empty() ? 0.0 : trajectory.back().timeSec;
}

NominalTrajectoryPoint RocketMissionData::interpolate(double timeSec) const {
    if (trajectory.empty()) return {};
    if (timeSec <= trajectory.front().timeSec) return trajectory.front();
    if (timeSec >= trajectory.back().timeSec) return trajectory.back();
    const auto upper = std::lower_bound(trajectory.begin(), trajectory.end(), timeSec,
        [](const NominalTrajectoryPoint& point, double time) { return point.timeSec < time; });
    const auto lower = upper - 1;
    const double fraction = (timeSec - lower->timeSec) / (upper->timeSec - lower->timeSec);
    NominalTrajectoryPoint result{};
    result.timeSec = timeSec;
    result.positionEciM = interpolateVector(lower->positionEciM, upper->positionEciM, fraction);
    result.velocityEciMps = interpolateVector(lower->velocityEciMps, upper->velocityEciMps, fraction);
    result.latitudeDeg = interpolateScalar(lower->latitudeDeg, upper->latitudeDeg, fraction);
    result.longitudeDeg = interpolateScalar(lower->longitudeDeg, upper->longitudeDeg, fraction);
    result.altitudeM = interpolateScalar(lower->altitudeM, upper->altitudeM, fraction);
    result.downrangeM = interpolateScalar(lower->downrangeM, upper->downrangeM, fraction);
    result.crossrangeM = interpolateScalar(lower->crossrangeM, upper->crossrangeM, fraction);
    result.speedMps = interpolateScalar(lower->speedMps, upper->speedMps, fraction);
    result.radialVelocityMps = interpolateScalar(lower->radialVelocityMps, upper->radialVelocityMps, fraction);
    result.tangentialVelocityMps = interpolateScalar(lower->tangentialVelocityMps, upper->tangentialVelocityMps, fraction);
    result.massKg = interpolateScalar(lower->massKg, upper->massKg, fraction);
    result.stageId = fraction < 0.5 ? lower->stageId : upper->stageId;
    result.thrustN = interpolateScalar(lower->thrustN, upper->thrustN, fraction);
    result.pitchReferenceDeg = interpolateScalar(lower->pitchReferenceDeg, upper->pitchReferenceDeg, fraction);
    result.azimuthReferenceDeg = interpolateScalar(lower->azimuthReferenceDeg, upper->azimuthReferenceDeg, fraction);
    result.referenceAttitude = slerp(lower->referenceAttitude, upper->referenceAttitude, fraction);
    result.referenceAngularRateRadPerSec = interpolateVector(
        lower->referenceAngularRateRadPerSec, upper->referenceAngularRateRadPerSec, fraction);
    result.densityKgPerM3 = interpolateScalar(lower->densityKgPerM3, upper->densityKgPerM3, fraction);
    result.dynamicPressurePa = interpolateScalar(lower->dynamicPressurePa, upper->dynamicPressurePa, fraction);
    result.dragEciN = interpolateVector(lower->dragEciN, upper->dragEciN, fraction);
    return result;
}

bool MissionRepository::load(const std::filesystem::path& directory, std::string& error) {
    try {
        const auto summaries = loadSummaries(directory / "mission_summary.csv");
        if (summaries.size() != 6) throw std::runtime_error("mission_summary.csv must contain exactly 6 missions");
        std::vector<RocketMissionData> loaded;
        loaded.reserve(6);
        for (const auto& summary : summaries) {
            RocketMissionData mission{};
            mission.summary = summary;
            mission.target = parseTarget(summary.targetAltitudeKm, summary.targetInclinationDeg);
            mission.missionKey = launchSiteKey(summary.site) + "_"
                + (mission.target == TargetOrbit::Leo300Km28_5Deg ? "300km_28p5deg" : "500km_51p6deg");
            mission.sourcePath = directory / trajectoryFileName(summary.site, mission.target);
            mission.trajectory = loadTrajectory(mission.sourcePath);
            loaded.push_back(std::move(mission));
        }
        directory_ = directory;
        missions_ = std::move(loaded);
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        missions_.clear();
        return false;
    }
}

bool MissionRepository::loadEmbedded(
    const std::unordered_map<std::string, std::string>& files, std::string& error) {
    try {
        const auto summaryFile = files.find("mission_summary.csv");
        if (summaryFile == files.end()) {
            throw std::runtime_error("Embedded mission_summary.csv is missing");
        }
        std::istringstream summaryStream(summaryFile->second);
        const auto summaries = parseSummaries(summaryStream);
        if (summaries.size() != 6) {
            throw std::runtime_error("Embedded mission_summary.csv must contain exactly 6 missions");
        }

        std::vector<RocketMissionData> loaded;
        loaded.reserve(6);
        for (const auto& summary : summaries) {
            RocketMissionData mission{};
            mission.summary = summary;
            mission.target = parseTarget(summary.targetAltitudeKm, summary.targetInclinationDeg);
            mission.missionKey = launchSiteKey(summary.site) + "_"
                + (mission.target == TargetOrbit::Leo300Km28_5Deg
                    ? "300km_28p5deg" : "500km_51p6deg");
            const std::string fileName = trajectoryFileName(summary.site, mission.target).string();
            const auto trajectoryFile = files.find(fileName);
            if (trajectoryFile == files.end()) {
                throw std::runtime_error("Embedded trajectory is missing: " + fileName);
            }
            std::istringstream trajectoryStream(trajectoryFile->second);
            mission.sourcePath = std::filesystem::path("embedded") / fileName;
            mission.trajectory = parseTrajectory(trajectoryStream);
            loaded.push_back(std::move(mission));
        }
        directory_ = std::filesystem::path("embedded");
        missions_ = std::move(loaded);
        error.clear();
        return true;
    } catch (const std::exception& exception) {
        error = exception.what();
        missions_.clear();
        return false;
    }
}

const RocketMissionData* MissionRepository::find(LaunchSite site, TargetOrbit target) const {
    const auto iterator = std::find_if(missions_.begin(), missions_.end(),
        [site, target](const RocketMissionData& mission) {
            return mission.summary.site == site && mission.target == target;
        });
    return iterator == missions_.end() ? nullptr : &*iterator;
}

std::string launchSiteKey(LaunchSite site) {
    switch (site) {
    case LaunchSite::Wenchang: return "wenchang";
    case LaunchSite::Xichang: return "xichang";
    default: return "cape";
    }
}

std::wstring launchSiteBilingual(LaunchSite site) {
    switch (site) {
    case LaunchSite::Wenchang: return L"文昌 / Wenchang";
    case LaunchSite::Xichang: return L"西昌 / Xichang";
    default: return L"卡纳维拉尔角 / Cape Canaveral";
    }
}

std::wstring targetOrbitBilingual(TargetOrbit target) {
    return target == TargetOrbit::Leo300Km28_5Deg
        ? L"300 km 圆轨道 / Circular LEO · 28.5°"
        : L"500 km 圆轨道 / Circular LEO · 51.6°";
}

std::filesystem::path trajectoryFileName(LaunchSite site, TargetOrbit target) {
    return launchSiteKey(site) + (target == TargetOrbit::Leo300Km28_5Deg
        ? "_300km_28p5deg.csv" : "_500km_51p6deg.csv");
}

} // namespace gnc
