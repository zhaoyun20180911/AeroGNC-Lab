#pragma once

#include "gnc/math.hpp"

#include <cstdint>

namespace gnc {

enum class ControlMode { Off, Default, Custom };
enum class ControllerMethod { PD, PID, LQR };

struct ControlConfig {
    ControlMode mode{ControlMode::Default};
    ControllerMethod customMethod{ControllerMethod::PD};
    Vec3 proportionalGain{};
    Vec3 integralGain{};
    Vec3 derivativeGain{};
    Vec3 integralLimit{0.1, 0.1, 0.1};
    bool antiWindup{true};
    double attitudeErrorWeight{10.0};
    double angularRateWeight{2.0};
    double controlEffortWeight{1.0};
};

struct SatelliteDisturbanceConfig {
    bool constantEnabled{true};
    Vec3 constantTorqueNm{0.00014, -0.00009, 0.00006};
    bool pulseEnabled{true};
    Vec3 pulseAxis{0.0, 0.0, 1.0};
    double pulseMagnitudeNm{0.001};
    double pulseStartSec{75.0};
    double pulseDurationSec{3.0};
    bool sineEnabled{true};
    Vec3 sineAxis{1.0, 0.0, 0.0};
    double sineAmplitudeNm{0.00005};
    double sineFrequencyHz{0.03};
    double sinePhaseDeg{};
    bool randomEnabled{false};
    double randomRmsNm{0.00002};
    double randomUpdateIntervalSec{0.2};
    std::uint32_t randomSeed{20260818u};
    bool deltaVImpulseEnabled{false};
    Vec3 deltaVImpulseRtnMps{0.0, 0.20, 0.0};
    double deltaVImpulseTimeSec{60.0};
};

struct SatelliteOrbitControlConfig {
    bool enabled{true};
    double positionGainPerSec2{3.6e-5};
    double velocityGainPerSec{1.2e-2};
    double maxThrustN{1.0};
    double specificImpulseSec{220.0};
    double propellantMassKg{12.0};
    double positionDeadbandM{2.0};
    double velocityDeadbandMps{0.002};
};

struct RocketDisturbanceConfig {
    bool steadyCrosswindEnabled{false};
    double crosswindSpeedMps{12.0};
    double crosswindDirectionDeg{90.0};
    bool gustEnabled{true};
    double gustSpeedMps{18.0};
    double gustDirectionDeg{90.0};
    double gustStartSec{55.0};
    double gustDurationSec{8.0};
    bool pulseForceEnabled{false};
    Vec3 pulseForceDirectionEci{0.0, 1.0, 0.0};
    double pulseForceN{8000.0};
    double pulseForceStartSec{80.0};
    double pulseForceDurationSec{2.0};
    bool pulseTorqueEnabled{true};
    Vec3 pulseTorqueAxisBody{0.0, 1.0, 0.0};
    double pulseTorqueNm{65000.0};
    double pulseTorqueStartSec{32.0};
    double pulseTorqueDurationSec{2.0};
    bool randomEnabled{false};
    double randomForceRmsN{1500.0};
    double randomTorqueRmsNm{3000.0};
    double randomUpdateIntervalSec{0.2};
    std::uint32_t randomSeed{20260818u};
};

struct PerturbationConfig {
    bool j2Selected{};
    bool atmosphericDragSelected{};
    bool moonThirdBodySelected{};
    bool sunThirdBodySelected{};
    bool solarRadiationPressureSelected{};
};

enum class SatelliteObjective { NadirPointing, InertialPointing, TargetTracking, SlewManeuver };

struct ClassicalOrbitElements {
    double semiMajorAxisKm{6878.137};
    double eccentricity{};
    double inclinationDeg{51.6};
    double raanDeg{};
    double argumentOfPerigeeDeg{};
    double trueAnomalyDeg{};
};

struct SatelliteMissionConfig {
    SatelliteObjective objective{SatelliteObjective::NadirPointing};
    Vec3 targetEulerDeg{};
    double targetLatitudeDeg{20.0};
    double targetLongitudeDeg{110.0};
    Vec3 slewTargetEulerDeg{0.0, 20.0, 35.0};
    double slewStartSec{20.0};
};

struct RocketModelDeviations {
    double massPercent{};
    double inertiaPercent{};
    double thrustPercent{};
    double specificImpulsePercent{};
    double dragCoefficientPercent{};
    double atmosphericDensityPercent{};
    double tvcZeroBiasDeg{};
};

struct RocketGuidanceConfig {
    bool enabled{true};
    double positionGainPerSec2{2.0e-5};
    double velocityGainPerSec{1.2e-2};
    double maxCorrectionAngleDeg{8.0};
    double maxCommandRateDegPerSec{1.5};
    bool terminalOrbitGuidance{true};
    double terminalLeadTimeSec{90.0};
    bool adaptiveCutoff{true};
    double specificEnergyToleranceJPerKg{2500.0};
    double maxBurnExtensionSec{90.0};
};

} // namespace gnc
