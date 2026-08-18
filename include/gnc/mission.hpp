#pragma once

#include "gnc/math.hpp"

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace gnc {

enum class LaunchSite {
    Wenchang,
    Xichang,
    CapeCanaveral
};

enum class TargetOrbit {
    Leo300Km28_5Deg,
    Leo500Km51_6Deg
};

struct RocketMissionSummary {
    int missionId{};
    LaunchSite site{LaunchSite::Wenchang};
    double siteLatitudeDeg{};
    double siteLongitudeDeg{};
    double siteAltitudeM{};
    double targetAltitudeKm{};
    double targetInclinationDeg{};
    double payloadMassKg{};
    double stage1DryMassKg{};
    double stage1PropellantMassKg{};
    double stage1ThrustN{};
    double stage1SpecificImpulseSec{};
    double stage1DragCoefficient{};
    double stage1ReferenceAreaM2{};
    double stage2DryMassKg{};
    double stage2PropellantMassKg{};
    double stage2ThrustN{};
    double stage2SpecificImpulseSec{};
    double stage2DragCoefficient{};
    double stage2ReferenceAreaM2{};
    Vec3 nominalWetInertiaKgM2{};
    Vec3 nominalDryInertiaKgM2{};
    double maxTvcAngleDeg{};
    double maxTvcRateDegPerSec{};
    double thrustLeverArmM{};
    double maxRollRcsTorqueNm{};
    double stage1BurnSec{};
    double stage2ActualBurnSec{};
    double finalAltitudeKm{};
    double finalSpeedMps{};
    double finalRadialVelocityMps{};
    double finalTangentialVelocityMps{};
    double finalSemiMajorAxisKm{};
    double finalEccentricity{};
    double finalInclinationDeg{};
    double finalRaanDeg{};
    double finalPerigeeAltitudeKm{};
    double finalApogeeAltitudeKm{};
    double maxDynamicPressureKPa{};
    double finalMassKg{};
    bool success{};

    [[nodiscard]] double nominalWetMassKg() const {
        return payloadMassKg + stage1DryMassKg + stage1PropellantMassKg
             + stage2DryMassKg + stage2PropellantMassKg;
    }
};

struct NominalTrajectoryPoint {
    double timeSec{};
    Vec3 positionEciM{};
    Vec3 velocityEciMps{};
    double latitudeDeg{};
    double longitudeDeg{};
    double altitudeM{};
    double downrangeM{};
    double crossrangeM{};
    double speedMps{};
    double radialVelocityMps{};
    double tangentialVelocityMps{};
    double massKg{};
    int stageId{};
    double thrustN{};
    double pitchReferenceDeg{};
    double azimuthReferenceDeg{};
    Quaternion referenceAttitude{};
    Vec3 referenceAngularRateRadPerSec{};
    double densityKgPerM3{};
    double dynamicPressurePa{};
    Vec3 dragEciN{};
};

struct RocketMissionData {
    RocketMissionSummary summary{};
    TargetOrbit target{TargetOrbit::Leo300Km28_5Deg};
    std::string missionKey;
    std::filesystem::path sourcePath;
    std::vector<NominalTrajectoryPoint> trajectory;

    [[nodiscard]] double startTimeSec() const;
    [[nodiscard]] double endTimeSec() const;
    [[nodiscard]] NominalTrajectoryPoint interpolate(double timeSec) const;
};

class MissionRepository {
public:
    bool load(const std::filesystem::path& directory, std::string& error);
    bool loadEmbedded(const std::unordered_map<std::string, std::string>& files,
                      std::string& error);
    [[nodiscard]] bool loaded() const { return missions_.size() == 6; }
    [[nodiscard]] const std::filesystem::path& directory() const { return directory_; }
    [[nodiscard]] const std::vector<RocketMissionData>& missions() const { return missions_; }
    [[nodiscard]] const RocketMissionData* find(LaunchSite site, TargetOrbit target) const;

private:
    std::filesystem::path directory_;
    std::vector<RocketMissionData> missions_;
};

[[nodiscard]] std::string launchSiteKey(LaunchSite site);
[[nodiscard]] std::wstring launchSiteBilingual(LaunchSite site);
[[nodiscard]] std::wstring targetOrbitBilingual(TargetOrbit target);
[[nodiscard]] std::filesystem::path trajectoryFileName(LaunchSite site, TargetOrbit target);

} // namespace gnc
