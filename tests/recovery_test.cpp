#include "gnc/recovery.hpp"
#include "gnc/localization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

const char* caseName(gnc::RecoveryTestCase testCase) {
    switch (testCase) {
    case gnc::RecoveryTestCase::Nominal: return "nominal";
    case gnc::RecoveryTestCase::PositionError: return "position_error";
    case gnc::RecoveryTestCase::VelocityError: return "velocity_error";
    case gnc::RecoveryTestCase::ThrustBias: return "thrust_bias";
    case gnc::RecoveryTestCase::AtmosphereWind: return "atmosphere_wind";
    case gnc::RecoveryTestCase::Combined: return "combined";
    }
    return "unknown";
}

} // namespace

int main() {
    using namespace gnc;
    try {
        gnc::gui::setUiLanguage(gnc::gui::UiLanguage::Chinese);
        require(gnc::gui::localizeBilingual(L"姿态控制 / Attitude Control") == L"姿态控制",
                "Chinese UI localization");
        gnc::gui::setUiLanguage(gnc::gui::UiLanguage::English);
        require(gnc::gui::localizeBilingual(L"姿态控制 / Attitude Control") == L"Attitude Control",
                "English UI localization");
        require(gnc::gui::localizeBilingual(L"1.0 / 2.0") == L"1.0 / 2.0",
                "numeric separator is not treated as bilingual UI");
        gnc::gui::setUiLanguage(gnc::gui::UiLanguage::Chinese);

        const LandingSite site{};
        const NedFrame frame = makeLandingNedFrame(site);
        const Vec3 local{1234.0, -567.0, -8901.0};
        require((ecefPositionToNed(nedPositionToEcef(local, frame), frame) - local).norm()
                    < 1.0e-8,
                "ECEF/NED position round trip");
        {
            const RecoveryConfiguration handoffConfig = makeDefaultRecoveryConfiguration();
            const std::filesystem::path path = std::filesystem::current_path()
                / "recovery_handoff_test.csv";
            require(exportRecoveryInitialState(path.wstring(), handoffConfig.initialState,
                                               handoffConfig.landingSite),
                    "separation-state export");
            RecoveryInitialState imported{};
            LandingSite importedSite{};
            require(importRecoveryInitialState(path.wstring(), imported, importedSite),
                    "separation-state import");
            std::filesystem::remove(path);
            require((imported.positionEcefM - handoffConfig.initialState.positionEcefM).norm()
                        < 1.0e-8
                        && (imported.velocityEcefMps
                            - handoffConfig.initialState.velocityEcefMps).norm() < 1.0e-10
                        && std::abs(importedSite.latitudeDeg
                            - handoffConfig.landingSite.latitudeDeg) < 1.0e-12,
                    "separation-state handoff round trip");
        }

        constexpr std::array cases{
            RecoveryTestCase::Nominal,
            RecoveryTestCase::PositionError,
            RecoveryTestCase::VelocityError,
            RecoveryTestCase::ThrustBias,
            RecoveryTestCase::AtmosphereWind,
            RecoveryTestCase::Combined
        };
        int touchdowns{};
        for (RecoveryTestCase testCase : cases) {
            RecoveryConfiguration config = makeDefaultRecoveryConfiguration();
            applyRecoveryTestCase(config, testCase);
            RecoverySimulation simulation(config);
            std::set<int> phases;
            int previousPhase = simulation.currentSample().recoveryPhase;
            double firstAeroPrediction = -1.0;
            double lastAeroPrediction = -1.0;
            while (!simulation.complete()) {
                simulation.step(0.02);
                const SimulationSample& sample = simulation.currentSample();
                phases.insert(sample.recoveryPhase);
                if (sample.recoveryPhase == static_cast<int>(RecoveryPhase::AeroDescent)) {
                    const double predicted = std::hypot(sample.predictedLandingErrorNed.x,
                                                        sample.predictedLandingErrorNed.y);
                    if (firstAeroPrediction < 0.0) firstAeroPrediction = predicted;
                    lastAeroPrediction = predicted;
                }
                if (sample.recoveryPhase != previousPhase && testCase == RecoveryTestCase::Nominal) {
                    std::cout << "  transition=" << previousPhase << "->" << sample.recoveryPhase
                              << ",t=" << sample.time << ",alt=" << sample.altitude
                              << ",ground=" << sample.groundDistance << ",vh="
                              << sample.horizontalVelocity << ",vd=" << sample.verticalVelocity
                              << ",pred_n=" << sample.predictedLandingErrorNed.x
                              << ",fuel=" << sample.propellantRemaining << '\n';
                    previousPhase = sample.recoveryPhase;
                }
                require(std::isfinite(sample.position.normSquared())
                            && std::isfinite(sample.velocity.normSquared())
                            && std::isfinite(sample.attitude.normSquared())
                            && std::isfinite(sample.mass),
                        std::string(caseName(testCase)) + " state remains finite");
            }
            const RecoveryMetrics metrics = simulation.metrics();
            if (metrics.finalPhase == RecoveryPhase::Touchdown) ++touchdowns;
            std::cout << caseName(testCase)
                      << ",phase=" << static_cast<int>(metrics.finalPhase)
                      << ",landing_error_m=" << metrics.landingPositionErrorM
                      << ",vh_mps=" << metrics.horizontalTouchdownVelocityMps
                      << ",vd_mps=" << metrics.verticalTouchdownVelocityMps
                      << ",fuel_kg=" << metrics.propellantConsumedKg
                      << ",solver_success=" << metrics.solverSuccessRate
                      << ",solver_mean_ms=" << metrics.meanOptimizerSolveTimeMs
                      << ",solver_max_ms=" << metrics.maxOptimizerSolveTimeMs
                      << ",tilt_deg=" << metrics.touchdownTiltDeg
                      << ",phases=" << phases.size() << '\n';
            if (!metrics.failureReason.empty()) std::wcout << L"  failure=" << metrics.failureReason << L'\n';
            require(metrics.finalPhase == RecoveryPhase::Touchdown,
                    std::string(caseName(testCase)) + " must reach safe TOUCHDOWN");
            for (RecoveryPhase required : {RecoveryPhase::Flip, RecoveryPhase::Boostback,
                                           RecoveryPhase::Coast, RecoveryPhase::EntryBurn,
                                           RecoveryPhase::AeroDescent, RecoveryPhase::Landing,
                                           RecoveryPhase::Touchdown}) {
                require(phases.count(static_cast<int>(required)) != 0,
                        std::string(caseName(testCase)) + " complete phase sequence");
            }
            require(metrics.maxTvcAngleDeg <= config.vehicle.tvcLimitDeg + 1.0e-6,
                    std::string(caseName(testCase)) + " TVC limit");
            require(metrics.maxGridFinDeflectionDeg <= config.vehicle.gridFinLimitDeg + 1.0e-6,
                    std::string(caseName(testCase)) + " grid-fin limit");
            require(metrics.propellantConsumedKg >= 0.0
                        && metrics.propellantConsumedKg <= config.initialState.propellantMassKg + 1.0e-6,
                    std::string(caseName(testCase)) + " propellant accounting");
            require(metrics.landingPositionErrorM <= config.touchdownPositionToleranceM
                        && metrics.horizontalTouchdownVelocityMps
                            <= config.touchdownHorizontalVelocityMps
                        && std::abs(metrics.verticalTouchdownVelocityMps)
                            <= config.touchdownVerticalVelocityMps,
                    std::string(caseName(testCase)) + " touchdown safety limits");
            require(metrics.meanOptimizerSolveTimeMs < 1000.0 / config.guidanceFrequencyHz,
                    std::string(caseName(testCase)) + " optimizer runs within guidance period");
            if (testCase == RecoveryTestCase::AtmosphereWind) {
                require(metrics.maxGridFinDeflectionDeg > 0.01,
                        "atmosphere/wind case must command grid fins");
                require(firstAeroPrediction >= 0.0 && lastAeroPrediction >= 0.0
                            && lastAeroPrediction < firstAeroPrediction,
                        "grid-fin phase must reduce predicted landing error");
            }
            if (testCase == RecoveryTestCase::Nominal) {
                const std::filesystem::path path = std::filesystem::current_path()
                    / "recovery_csv_export_test.csv";
                require(exportRecoveryCsv(path.wstring(), simulation.history()),
                        "recovery CSV export");
                std::ifstream stream(path, std::ios::binary);
                const std::string csv((std::istreambuf_iterator<char>(stream)),
                                      std::istreambuf_iterator<char>());
                stream.close();
                std::filesystem::remove(path);
                require(csv.find("time_s,phase,ecef_x_m") == 0
                            && csv.find("solver_status") != std::string::npos
                            && csv.find("grid_fin_4_deg") != std::string::npos,
                        "recovery CSV schema and data");
            }
        }
        require(touchdowns == 6, "all six standard recoveries reach touchdown");
        std::cout << "Recovery standard cases passed (touchdowns=" << touchdowns << "/6)\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Recovery test failed: " << error.what() << '\n';
        return 1;
    }
}
