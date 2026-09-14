#pragma once

#include "gnc/control.hpp"
#include "gnc/simulation.hpp"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace gnc {

constexpr double kRecoveryEarthRadiusM = 6378137.0;

enum class RecoveryPhase {
    Initialize, Flip, Boostback, Coast, EntryBurn, AeroDescent, Landing, Touchdown, Failed
};

enum class RecoverySolverStatus {
    NotRun, Solved, MaxIterations, Infeasible, NumericalError, Timeout, Fallback
};
enum class RecoveryTestCase { Nominal, PositionError, VelocityError, ThrustBias, AtmosphereWind, Combined };

struct RecoveryInitialState {
    double time{};
    Vec3 positionEcefM{};
    Vec3 velocityEcefMps{};
    Quaternion bodyToEcef{};
    Vec3 angularRateBodyRadPerSec{};
    double massTotalKg{90000.0};
    double propellantMassKg{62000.0};
};

struct LandingSite {
    double latitudeDeg{28.5};
    double longitudeDeg{-80.6};
    double altitudeM{};
};

struct RecoveryVehicleParameters {
    double dryMassKg{28000.0};
    Vec3 inertiaKgM2{1.6e6, 8.0e6, 8.0e6};
    double maxThrustN{2280000.0};
    double minThrottle{0.13};
    double maxThrottle{1.0};
    double specificImpulseSec{282.0};
    double ignitionDelaySec{0.25};
    double shutdownDelaySec{0.15};
    double thrustLagSec{0.35};
    double thrustBiasPercent{};
    double tvcLimitDeg{8.0};
    double tvcRateDegPerSec{20.0};
    double thrustLeverArmM{20.0};
    double maxRcsTorqueNm{250000.0};
    double referenceAreaM2{10.8};
    double dragCoefficient{0.42};
    double gridFinAreaM2{7.5};
    double gridFinLiftSlopePerRad{2.0};
    double gridFinLimitDeg{25.0};
    double gridFinRateDegPerSec{45.0};
};

struct RecoveryConfiguration {
    RecoveryInitialState initialState{};
    LandingSite landingSite{};
    RecoveryVehicleParameters vehicle{};
    ControlConfig attitudeControl{};
    double durationSec{450.0};
    double playbackSpeed{20.0};
    bool maxPlayback{};
    double guidanceFrequencyHz{10.0};
    double flipDurationSec{8.0};
    double flipAttitudeToleranceDeg{3.0};
    double flipRateToleranceDegPerSec{0.5};
    double flipStableTimeSec{0.8};
    double boostbackMinDurationSec{22.0};
    double boostbackMaxDurationSec{48.0};
    double boostbackPredictedErrorToleranceM{2500.0};
    double entryTriggerAltitudeM{32000.0};
    double entryTargetSpeedMps{320.0};
    double entryMaxDurationSec{18.0};
    double landingBurnSafetyMarginM{180.0};
    double touchdownPositionToleranceM{25.0};
    double touchdownHorizontalVelocityMps{3.0};
    double touchdownVerticalVelocityMps{3.5};
    int landingQpHorizon{32};
    double landingQpDtSec{1.0};
    double weightTerminalPosition{18.0};
    double weightTerminalVelocity{60.0};
    double weightControlEffort{0.08};
    double weightControlSmoothness{1.5};
    double maxHorizontalAccelerationMps2{7.0};
    double maxTiltDeg{18.0};
    int solverMaxIterations{500};
    double solverTolerance{1.0e-2};
    double atmosphereDensityScale{1.0};
    Vec3 windNedMps{};
    RecoveryTestCase testCase{RecoveryTestCase::Nominal};
};

struct RecoveryTruthState {
    double time{};
    Vec3 positionEcefM{};
    Vec3 velocityEcefMps{};
    Quaternion bodyToEcef{};
    Vec3 angularRateBodyRadPerSec{};
    double massKg{};
    double propellantKg{};
};

using RecoveryEstimatedState = RecoveryTruthState;

struct NedFrame {
    Vec3 northEcef{};
    Vec3 eastEcef{};
    Vec3 downEcef{};
    Vec3 originEcefM{};
};

[[nodiscard]] NedFrame makeLandingNedFrame(const LandingSite& site);
[[nodiscard]] Vec3 ecefVectorToNed(const Vec3& ecef, const NedFrame& frame);
[[nodiscard]] Vec3 nedVectorToEcef(const Vec3& ned, const NedFrame& frame);
[[nodiscard]] Vec3 ecefPositionToNed(const Vec3& ecef, const NedFrame& frame);
[[nodiscard]] Vec3 nedPositionToEcef(const Vec3& ned, const NedFrame& frame);
[[nodiscard]] Quaternion attitudeFromBodyX(const Vec3& bodyXDirectionEcef,
                                           const Vec3& preferredBodyYEcef);

class RecoveryNavigation {
public:
    [[nodiscard]] RecoveryEstimatedState estimate(const RecoveryTruthState& truth) const { return truth; }
};

struct LandingPrediction {
    Vec3 touchdownPositionNedM{};
    Vec3 touchdownVelocityNedMps{};
    Vec3 errorNedM{};
    double timeToGroundSec{};
    bool valid{};
};

class LandingPointPredictor {
public:
    [[nodiscard]] LandingPrediction predict(const RecoveryEstimatedState& state,
                                            const NedFrame& frame,
                                            const RecoveryConfiguration& config) const;
};

struct GuidanceCommand {
    Vec3 desiredAccelerationNedMps2{};
    Vec3 desiredThrustDirectionEcef{1.0, 0.0, 0.0};
    double desiredThrustN{};
    Quaternion desiredAttitude{};
    Vec3 desiredAngularRateBodyRadPerSec{};
    std::array<double, 4> gridFinCommandRad{};
    bool engineOn{};
    bool engineCutoff{};
};

struct QpProblem {
    int variableCount{};
    std::vector<double> hessian;
    std::vector<double> gradient;
    std::vector<double> lowerBound;
    std::vector<double> upperBound;
    int maxIterations{100};
    double tolerance{1.0e-5};
};

struct QpResult {
    RecoverySolverStatus status{RecoverySolverStatus::NotRun};
    std::vector<double> solution;
    int iterations{};
    double solveTimeMs{};
};

class QpSolverInterface {
public:
    virtual ~QpSolverInterface() = default;
    [[nodiscard]] virtual QpResult solve(const QpProblem& problem,
                                         const std::vector<double>& warmStart) = 0;
};

class ProjectedGradientQpSolver final : public QpSolverInterface {
public:
    [[nodiscard]] QpResult solve(const QpProblem& problem,
                                 const std::vector<double>& warmStart) override;
};

class ConvexLandingProblem {
public:
    [[nodiscard]] QpProblem build(const Vec3& positionNedM,
                                  const Vec3& velocityNedMps,
                                  const Vec3& previousAccelerationNedMps2,
                                  double massKg,
                                  const RecoveryConfiguration& config) const;
};

struct LandingGuidanceResult {
    Vec3 netAccelerationNedMps2{};
    RecoverySolverStatus solverStatus{RecoverySolverStatus::NotRun};
    double solveTimeMs{};
    int iterations{};
    bool fallbackUsed{};
};

class LandingGuidance {
public:
    LandingGuidance();
    void reset();
    [[nodiscard]] LandingGuidanceResult update(const RecoveryEstimatedState& state,
                                               const NedFrame& frame,
                                               const RecoveryConfiguration& config);

private:
    [[nodiscard]] Vec3 fallback(const Vec3& positionNedM, const Vec3& velocityNedMps,
                                double massKg, const RecoveryConfiguration& config) const;
    ConvexLandingProblem problemBuilder_{};
    std::unique_ptr<QpSolverInterface> solver_;
    std::vector<double> warmStart_{};
    Vec3 previousAccelerationNedMps2_{};
};

class RecoveryMissionManager {
public:
    void reset();
    void update(const RecoveryEstimatedState& state, const Vec3& positionNedM,
                const Vec3& velocityNedMps, double attitudeErrorDeg,
                double rateErrorDegPerSec, const LandingPrediction& prediction,
                double brakingDistanceM, const RecoveryConfiguration& config);
    void touchdown();
    void fail(std::wstring reason);
    [[nodiscard]] RecoveryPhase phase() const { return phase_; }
    [[nodiscard]] double phaseElapsed(double time) const { return time - phaseStartTimeSec_; }
    [[nodiscard]] const std::wstring& failureReason() const { return failureReason_; }

private:
    void transition(RecoveryPhase next, double time);
    RecoveryPhase phase_{RecoveryPhase::Initialize};
    double phaseStartTimeSec_{};
    double stableTimeSec_{};
    double previousUpdateTimeSec_{};
    std::wstring failureReason_{};
};

struct RecoveryActuatorCommand {
    double requestedThrustN{};
    bool engineOn{};
    double tvcPitchRad{};
    double tvcYawRad{};
    Vec3 appliedMomentBodyNm{};
    std::array<double, 4> gridFinRad{};
};

class RecoveryControlAllocator {
public:
    void reset();
    [[nodiscard]] RecoveryActuatorCommand update(const GuidanceCommand& guidance,
                                                 const Vec3& desiredMomentBodyNm,
                                                 const RecoveryEstimatedState& state,
                                                 RecoveryPhase phase, double dt,
                                                 const RecoveryConfiguration& config);

private:
    double tvcPitchRad_{};
    double tvcYawRad_{};
    std::array<double, 4> gridFinRad_{};
};

class RecoveryEngineModel {
public:
    void reset();
    [[nodiscard]] double update(bool commandOn, double requestedThrustN,
                                double propellantKg, double dt,
                                const RecoveryVehicleParameters& vehicle);
    [[nodiscard]] bool active() const { return actualThrustN_ > 1.0; }

private:
    double onCommandTimeSec_{};
    double offCommandTimeSec_{};
    double actualThrustN_{};
};

struct GridFinAerodynamicInput {
    double mach{};
    double dynamicPressurePa{};
    double angleOfAttackRad{};
    double sideslipRad{};
    std::array<double, 4> deflectionRad{};
};

struct GridFinAerodynamicOutput {
    Vec3 forceNedN{};
    Vec3 momentBodyNm{};
};

class GridFinAerodynamicModel {
public:
    [[nodiscard]] GridFinAerodynamicOutput evaluate(
        const GridFinAerodynamicInput& input,
        const RecoveryVehicleParameters& vehicle) const;
};

struct RecoveryMetrics {
    double landingPositionErrorM{};
    double horizontalTouchdownVelocityMps{};
    double verticalTouchdownVelocityMps{};
    double touchdownTiltDeg{};
    double propellantConsumedKg{};
    double maxAttitudeErrorDeg{};
    double maxTvcAngleDeg{};
    double maxGridFinDeflectionDeg{};
    double recoveryDurationSec{};
    double meanOptimizerSolveTimeMs{};
    double maxOptimizerSolveTimeMs{};
    double solverSuccessRate{};
    RecoveryPhase finalPhase{RecoveryPhase::Initialize};
    std::wstring failureReason{};
};

class RecoverySimulation {
public:
    explicit RecoverySimulation(const RecoveryConfiguration& config = {});
    void reset(const RecoveryConfiguration& config);
    void step(double dt);
    [[nodiscard]] bool complete() const;
    [[nodiscard]] double time() const { return truth_.time; }
    [[nodiscard]] const RecoveryConfiguration& config() const { return config_; }
    [[nodiscard]] const SimulationSample& currentSample() const { return currentSample_; }
    [[nodiscard]] const std::vector<SimulationSample>& history() const { return history_; }
    [[nodiscard]] RecoveryMetrics metrics() const;
    [[nodiscard]] RecoveryPhase phase() const { return mission_.phase(); }
    [[nodiscard]] const std::wstring& failureReason() const { return mission_.failureReason(); }

private:
    void updateGuidance(const RecoveryEstimatedState& estimated,
                        const Vec3& positionNed, const Vec3& velocityNed,
                        const LandingPrediction& prediction);
    void recordSample(bool force = false);
    [[nodiscard]] double brakingDistance(const Vec3& velocityNed, double massKg) const;

    RecoveryConfiguration config_{};
    NedFrame frame_{};
    RecoveryTruthState truth_{};
    RotationalState rotation_{};
    RecoveryNavigation navigation_{};
    LandingPointPredictor predictor_{};
    LandingGuidance landingGuidance_{};
    RecoveryMissionManager mission_{};
    RecoveryControlAllocator allocator_{};
    RecoveryEngineModel engine_{};
    GridFinAerodynamicModel gridFinAerodynamics_{};
    AttitudeController attitudeController_{};
    GuidanceCommand guidance_{};
    LandingPrediction prediction_{};
    RecoverySolverStatus lastSolverStatus_{RecoverySolverStatus::NotRun};
    double lastSolverTimeMs_{};
    bool lastSolverFallback_{};
    double nextGuidanceTimeSec_{};
    double nextSampleTimeSec_{};
    double lastThrustN_{};
    double lastDynamicPressurePa_{};
    double lastBrakingDistanceM_{};
    Vec3 lastControlTorqueNm_{};
    RecoveryActuatorCommand lastActuator_{};
    Quaternion flipStartAttitude_{};
    Quaternion flipTargetAttitude_{};
    double initialPropellantKg_{};
    int solverCalls_{};
    int solverSuccesses_{};
    double totalSolverTimeMs_{};
    double maxSolverTimeMs_{};
    Vec3 touchdownPositionNed_{};
    Vec3 touchdownVelocityNed_{};
    bool touchdownCaptured_{};
    SimulationSample currentSample_{};
    std::vector<SimulationSample> history_{};
};

[[nodiscard]] RecoveryConfiguration makeDefaultRecoveryConfiguration();
void applyRecoveryTestCase(RecoveryConfiguration& config, RecoveryTestCase testCase);
[[nodiscard]] std::wstring recoveryPhaseBilingual(RecoveryPhase phase);
[[nodiscard]] std::wstring recoverySolverStatusBilingual(RecoverySolverStatus status);
bool exportRecoveryCsv(const std::wstring& path, const std::vector<SimulationSample>& samples);
bool exportRecoveryInitialState(const std::wstring& path, const RecoveryInitialState& state,
                                const LandingSite& site);
bool importRecoveryInitialState(const std::wstring& path, RecoveryInitialState& state,
                                LandingSite& site);

} // namespace gnc
