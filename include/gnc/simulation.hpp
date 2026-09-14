#pragma once

#include "gnc/math.hpp"
#include "gnc/config.hpp"
#include "gnc/control.hpp"
#include "gnc/mission.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace gnc {

enum class ScenarioKind { Satellite, Rocket, Recovery };
enum class RunState { Ready, Running, Paused, Completed };

struct SimulationSample {
    double time{};
    Vec3 position{};
    Vec3 velocity{};
    Vec3 nominalPosition{};
    Vec3 nominalVelocity{};
    Quaternion attitude{};
    Quaternion referenceAttitude{};
    Quaternion nominalReferenceAttitude{};
    Vec3 eulerDeg{};
    Vec3 referenceEulerDeg{};
    Vec3 attitudeErrorDeg{};
    Vec3 angularRateDegPerSec{};
    Vec3 controlTorque{};
    Vec3 wheelSpeedRpm{};
    double mass{};
    double altitude{};
    double speed{};
    double dynamicPressure{};
    double tvcPitchDeg{};
    double tvcYawDeg{};
    double thrust{};
    double drag{};
    double orbitRadius{};
    double nominalAltitude{};
    double nominalSpeed{};
    double nominalPitchDeg{};
    double actualPitchDeg{};
    double positionError{};
    double crossTrackError{};
    double radialPositionError{};
    double alongTrackPositionError{};
    double altitudeError{};
    double velocityError{};
    Vec3 guidanceCorrectionDeg{};
    Vec3 orbitPositionErrorRtnM{};
    Vec3 orbitVelocityErrorRtnMps{};
    Vec3 orbitControlForceBodyN{};
    double propellantRemaining{};
    double specificEnergyError{};
    double cumulativeDeltaVMps{};
    double appliedImpulseDeltaVMps{};
    int stageId{};
    int missionId{};
    bool orbitalCoast{};
    bool ascentTrackingActive{true};
    bool controlActive{true};
    bool actuatorSaturated{};
    bool guidanceActive{};
    bool terminalGuidanceActive{};
    bool propellantDepleted{};
    bool engineCutoff{};
    bool translationalImpulseApplied{};
    bool orbitControlSaturated{};

    Vec3 positionNed{};
    Vec3 velocityNed{};
    Vec3 predictedLandingErrorNed{};
    std::array<double, 4> gridFinDeflectionDeg{};
    double groundDistance{};
    double horizontalVelocity{};
    double verticalVelocity{};
    double landingError{};
    double brakingDistance{};
    double optimizerSolveTimeMs{};
    double optimizerSuccessRate{};
    double touchdownTiltDeg{};
    int recoveryPhase{};
    int optimizerStatus{};
    bool optimizerFallback{};
    bool recoveryFailed{};
};

struct PerformanceMetrics {
    double maxAttitudeErrorDeg{};
    double rmsAttitudeErrorDeg{};
    double settlingTimeSec{-1.0};
    double maxControlTorque{};
    double maxActuatorValue{};
    double maxAngularRateDegPerSec{};
    double maxDynamicPressure{};
    double finalMass{};
    bool saturated{};
    double maxPositionError{};
    double terminalAltitudeError{};
    double terminalVelocityError{};
    double terminalSemiMajorAxisErrorKm{};
    double terminalEccentricityError{};
    double terminalInclinationErrorDeg{};
    double finalPositionError{};
    double finalVelocityError{};
    double cumulativeDeltaVMps{};
    double propellantUsedKg{};
    double maxOrbitControlThrustN{};
};

struct RocketMissionConfig {
    const RocketMissionData* mission{};
    RocketModelDeviations deviations{};
    ControlConfig control{};
    RocketDisturbanceConfig disturbances{};
    RocketGuidanceConfig guidance{};
    Vec3 initialAttitudeErrorDeg{1.0, -1.5, 0.8};
    Vec3 initialRateErrorDegPerSec{0.02, -0.03, 0.02};
    double durationSec{600.0};
    double playbackSpeed{20.0};
    bool maxPlayback{};
};

class RocketMissionSimulation {
public:
    explicit RocketMissionSimulation(const RocketMissionConfig& config = {});
    void reset(const RocketMissionConfig& config);
    void step(double dt);
    [[nodiscard]] bool valid() const { return config_.mission != nullptr; }
    [[nodiscard]] bool complete() const { return time_ >= config_.durationSec; }
    [[nodiscard]] double time() const { return time_; }
    [[nodiscard]] const RocketMissionConfig& config() const { return config_; }
    [[nodiscard]] const SimulationSample& currentSample() const { return currentSample_; }
    [[nodiscard]] const std::vector<SimulationSample>& history() const { return history_; }
    [[nodiscard]] bool hasInsertionSnapshot() const { return insertionCaptured_; }
    [[nodiscard]] const SimulationSample& insertionSample() const { return insertionSample_; }
    [[nodiscard]] PerformanceMetrics metrics() const;

private:
    void recordSample(bool force = false);
    void advanceNominalCoast(double dt);

    RocketMissionConfig config_{};
    double time_{};
    double nextSampleTime_{};
    Vec3 positionEci_{};
    Vec3 velocityEci_{};
    RotationalState rotation_{};
    double mass_{};
    double stage1PropellantRemainingKg_{};
    double stage2PropellantRemainingKg_{};
    double tvcPitchRad_{};
    double tvcYawRad_{};
    int previousStageId_{};
    Vec3 lastTorque_{};
    double lastDynamicPressure_{};
    double lastDrag_{};
    bool lastSaturated_{};
    bool nominalCoastInitialized_{};
    bool insertionCaptured_{};
    bool stage1Separated_{};
    bool stage2CutoffLatched_{};
    bool orbitalCoast_{};
    bool lastGuidanceActive_{};
    bool lastTerminalGuidanceActive_{};
    bool lastPropellantDepleted_{};
    bool lastEngineCutoff_{};
    double nominalInsertionTimeSec_{};
    double targetSpecificEnergyJPerKg_{};
    Vec3 targetOrbitNormalEci_{};
    Quaternion commandedAttitude_{};
    Vec3 previousCommandDirectionEci_{};
    Vec3 lastGuidanceCorrectionDeg_{};
    double lastActualThrust_{};
    double lastSpecificEnergyError_{};
    Vec3 nominalCoastPositionEci_{};
    Vec3 nominalCoastVelocityEci_{};
    SimulationSample insertionSample_{};
    SimulationSample currentSample_{};
    std::vector<SimulationSample> history_{};
    AttitudeController controller_{};
};

struct SatelliteConfig {
    double orbitAltitudeKm{500.0};
    ClassicalOrbitElements orbit{};
    SatelliteMissionConfig mission{};
    double massKg{420.0};
    Vec3 inertiaKgM2{120.0, 100.0, 80.0};
    Vec3 initialErrorDeg{12.0, -8.0, 16.0};
    Vec3 initialRateDegPerSec{0.08, -0.05, 0.06};
    double wheelMaxTorqueNm{0.20};
    double wheelInertiaKgM2{0.08};
    double wheelMaxSpeedRpm{6000.0};
    bool disturbanceEnabled{true};
    double disturbanceTorqueNm{0.00025};
    bool inertiaPerturbation{false};
    double durationSec{240.0};
    double playbackSpeed{20.0};
    ControlConfig control{};
    SatelliteOrbitControlConfig orbitControl{};
    SatelliteDisturbanceConfig disturbances{};
    PerturbationConfig perturbations{};
};

struct RocketConfig {
    double wetMassKg{120000.0};
    double dryMassKg{32000.0};
    double thrustN{2.05e6};
    double specificImpulseSec{300.0};
    Vec3 wetInertiaKgM2{1.20e6, 8.00e6, 8.00e6};
    Vec3 dryInertiaKgM2{0.24e6, 1.70e6, 1.70e6};
    double pitchStartSec{5.0};
    double pitchEndSec{70.0};
    double finalPitchDeg{52.0};
    Vec3 initialErrorDeg{2.0, -3.0, 2.0};
    Vec3 initialRateDegPerSec{0.05, -0.08, 0.04};
    double maxTvcAngleDeg{6.0};
    double maxTvcRateDegPerSec{15.0};
    double thrustLeverArmM{18.0};
    double dragCoefficient{0.35};
    double referenceAreaM2{12.0};
    bool disturbanceEnabled{true};
    double durationSec{110.0};
    double playbackSpeed{2.0};
};

class SatelliteSimulation {
public:
    explicit SatelliteSimulation(const SatelliteConfig& config = {});

    void reset(const SatelliteConfig& config);
    void step(double dt);
    [[nodiscard]] const SatelliteConfig& config() const { return config_; }
    [[nodiscard]] double time() const { return time_; }
    [[nodiscard]] bool complete() const { return time_ >= config_.durationSec; }
    [[nodiscard]] const SimulationSample& currentSample() const { return currentSample_; }
    [[nodiscard]] const std::vector<SimulationSample>& history() const { return history_; }
    [[nodiscard]] PerformanceMetrics metrics() const;

private:
    void recordSample(bool force = false);

    SatelliteConfig config_{};
    double time_{};
    double nextSampleTime_{};
    Vec3 positionEci_{};
    Vec3 velocityEci_{};
    Vec3 nominalPositionEci_{};
    Vec3 nominalVelocityEci_{};
    RotationalState rotation_{};
    Vec3 wheelSpeedRadPerSec_{};
    Vec3 lastTorque_{};
    Vec3 lastOrbitControlForceBodyN_{};
    double massKg_{};
    double dryMassKg_{};
    double propellantRemainingKg_{};
    double cumulativeDeltaVMps_{};
    double appliedImpulseDeltaVMps_{};
    bool lastSaturated_{};
    bool lastOrbitControlActive_{};
    bool lastOrbitControlSaturated_{};
    bool deltaVImpulseApplied_{};
    SimulationSample currentSample_{};
    std::vector<SimulationSample> history_{};
    AttitudeController controller_{};
    Quaternion slewHoldAttitude_{};
};

class RocketSimulation {
public:
    explicit RocketSimulation(const RocketConfig& config = {});

    void reset(const RocketConfig& config);
    void step(double dt);
    [[nodiscard]] const RocketConfig& config() const { return config_; }
    [[nodiscard]] double time() const { return time_; }
    [[nodiscard]] bool complete() const { return time_ >= config_.durationSec; }
    [[nodiscard]] const SimulationSample& currentSample() const { return currentSample_; }
    [[nodiscard]] const std::vector<SimulationSample>& history() const { return history_; }
    [[nodiscard]] PerformanceMetrics metrics() const;

private:
    void recordSample(bool force = false);

    RocketConfig config_{};
    double time_{};
    double nextSampleTime_{};
    Vec3 positionLocal_{};
    Vec3 velocityLocal_{};
    RotationalState rotation_{};
    double mass_{};
    double tvcPitchRad_{};
    double tvcYawRad_{};
    Vec3 lastTorque_{};
    double lastDynamicPressure_{};
    double lastDrag_{};
    bool lastSaturated_{};
    SimulationSample currentSample_{};
    std::vector<SimulationSample> history_{};
};

bool exportCsv(const std::wstring& path, ScenarioKind scenario,
               const std::vector<SimulationSample>& samples);

} // namespace gnc
