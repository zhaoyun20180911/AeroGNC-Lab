#include "gnc/math.hpp"
#include "gnc/mission.hpp"
#include "gnc/orbit.hpp"
#include "gnc/control.hpp"
#include "gnc/disturbance.hpp"
#include "gnc/simulation.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace {

int gPassed = 0;
int gFailed = 0;

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(double actual, double expected, double tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message + " (actual=" + std::to_string(actual)
                               + ", expected=" + std::to_string(expected) + ")");
    }
}

template <typename Function>
void test(const std::string& name, Function&& function) {
    try {
        function();
        ++gPassed;
        std::cout << "[PASS] " << name << '\n';
    } catch (const std::exception& error) {
        ++gFailed;
        std::cout << "[FAIL] " << name << ": " << error.what() << '\n';
    }
}

double attitudeErrorDeg(const gnc::SimulationSample& sample) {
    return gnc::quaternionAngularDistance(sample.attitude, sample.referenceAttitude) * gnc::kRadToDeg;
}

} // namespace

int main() {
    using namespace gnc;

    test("identity quaternion", [] {
        const Vec3 input{1.0, -2.0, 3.0};
        const Vec3 output = Quaternion::identity().rotate(input);
        requireNear((output - input).norm(), 0.0, 1.0e-12, "identity must preserve vectors");
    });

    test("axis rotations", [] {
        const Quaternion q = Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, 90.0 * kDegToRad);
        const Vec3 output = q.rotate({1.0, 0.0, 0.0});
        requireNear(output.x, 0.0, 1.0e-12, "90 degree Z rotation X component");
        requireNear(output.y, 1.0, 1.0e-12, "90 degree Z rotation Y component");
    });

    test("quaternion multiplication order", [] {
        const Quaternion qx = Quaternion::fromAxisAngle({1.0, 0.0, 0.0}, 90.0 * kDegToRad);
        const Quaternion qz = Quaternion::fromAxisAngle({0.0, 0.0, 1.0}, 90.0 * kDegToRad);
        const Vec3 result = (qz * qx).rotate({0.0, 1.0, 0.0});
        requireNear(result.z, 1.0, 1.0e-12, "rightmost rotation must apply first");
    });

    test("quaternion matrix round-trip", [] {
        const Quaternion original = Quaternion::fromEulerZYX(0.4, -0.25, 1.1);
        const Quaternion recovered = Quaternion::fromRotationMatrix(original.toRotationMatrix());
        requireNear(quaternionAngularDistance(original, recovered), 0.0, 1.0e-10,
                    "quaternion/DCM round trip");
    });

    test("Euler ZYX round-trip", [] {
        const Vec3 angles{0.32, -0.41, 0.76};
        const Vec3 recovered = Quaternion::fromEulerZYX(angles.x, angles.y, angles.z).toEulerZYX();
        requireNear((recovered - angles).norm(), 0.0, 1.0e-10, "Euler round trip");
    });

    test("q and -q equivalence", [] {
        const Quaternion q = Quaternion::fromEulerZYX(0.2, 0.4, -0.7);
        const Quaternion negative{-q.w, -q.x, -q.y, -q.z};
        requireNear(quaternionAngularDistance(q, negative), 0.0, 1.0e-12,
                    "double cover must represent the same attitude");
    });

    test("normalization", [] {
        const Quaternion unit = Quaternion{3.0, 2.0, -4.0, 1.0}.normalized();
        requireNear(unit.norm(), 1.0, 1.0e-12, "normalized quaternion norm");
    });

    test("full inertia matrix inverse", [] {
        Mat3 matrix = Mat3::diagonal({3.0, 4.0, 5.0});
        matrix.m[0][1] = matrix.m[1][0] = 0.2;
        matrix.m[1][2] = matrix.m[2][1] = -0.1;
        const Vec3 input{0.7, -1.2, 2.1};
        const Vec3 recovered = matrix.inverse() * (matrix * input);
        requireNear((recovered - input).norm(), 0.0, 1.0e-11, "general matrix inverse");
    });

    test("free rigid-body inertial angular momentum", [] {
        const Mat3 inertia = Mat3::diagonal({3.0, 4.0, 5.0});
        RotationalState state{Quaternion::identity(), {0.21, -0.17, 0.09}};
        const Vec3 initialMomentum = state.attitude.rotate(inertia * state.angularRate);
        for (int index = 0; index < 5000; ++index) {
            integrateRotationalRK4(state, inertia, {}, 0.002);
        }
        const Vec3 finalMomentum = state.attitude.rotate(inertia * state.angularRate);
        requireNear((finalMomentum - initialMomentum).norm(), 0.0, 2.0e-8,
                    "angular momentum should be inertially conserved");
        requireNear(state.attitude.norm(), 1.0, 1.0e-12, "RK4 must normalize quaternion");
    });

    test("constant torque spherical body", [] {
        const Mat3 inertia = Mat3::diagonal({2.0, 2.0, 2.0});
        RotationalState state{};
        for (int index = 0; index < 1000; ++index) {
            integrateRotationalRK4(state, inertia, {0.0, 0.0, 0.4}, 0.001);
        }
        requireNear(state.angularRate.z, 0.2, 1.0e-10, "omega = tau/I * t");
    });

    test("classical orbital elements round-trip", [] {
        ClassicalOrbitElements elements{};
        elements.semiMajorAxisKm = 7200.0;
        elements.eccentricity = 0.08;
        elements.inclinationDeg = 63.4;
        elements.raanDeg = 42.0;
        elements.argumentOfPerigeeDeg = 18.0;
        elements.trueAnomalyDeg = 127.0;
        require(validOrbitElements(elements), "valid elliptic orbit");
        const CartesianOrbitState state = classicalElementsToCartesian(elements);
        const ClassicalOrbitElements recovered = cartesianToClassicalElements(
            state.positionEciM, state.velocityEciMps);
        requireNear(recovered.semiMajorAxisKm, elements.semiMajorAxisKm, 1.0e-8, "semi-major axis");
        requireNear(recovered.eccentricity, elements.eccentricity, 1.0e-10, "eccentricity");
        requireNear(recovered.inclinationDeg, elements.inclinationDeg, 1.0e-9, "inclination");
        requireNear(recovered.raanDeg, elements.raanDeg, 1.0e-9, "RAAN");
        requireNear(recovered.argumentOfPerigeeDeg, elements.argumentOfPerigeeDeg, 1.0e-9,
                    "argument of perigee");
        requireNear(recovered.trueAnomalyDeg, elements.trueAnomalyDeg, 1.0e-9, "true anomaly");
    });

    test("control OFF, PID anti-windup and LQR selection", [] {
        const Vec3 inertia{120.0, 100.0, 80.0};
        const Vec3 authority{0.2, 0.2, 0.2};
        AttitudeController controller;
        ControlConfig off{};
        off.mode = ControlMode::Off;
        controller.configure(off, ControlVehicle::Satellite, inertia, authority);
        ControllerOutput output = controller.update(Quaternion::identity(),
            Quaternion::fromEulerZYX(0.2, 0.0, 0.0), {}, {}, 0.1);
        require(!output.active && output.desiredTorqueBody.norm() == 0.0, "control off means zero torque");

        const ControlRecommendation recommendation = recommendControlGains(
            ControlVehicle::Satellite, inertia, authority);
        ControlConfig pid{};
        pid.mode = ControlMode::Custom;
        pid.customMethod = ControllerMethod::PID;
        pid.proportionalGain = recommendation.proportionalGain;
        pid.integralGain = recommendation.integralGain;
        pid.derivativeGain = recommendation.derivativeGain;
        pid.integralLimit = recommendation.integralLimit;
        pid.antiWindup = true;
        controller.configure(pid, ControlVehicle::Satellite, inertia, authority);
        for (int index = 0; index < 2000; ++index) {
            output = controller.update(Quaternion::identity(),
                Quaternion::fromEulerZYX(1.0, 0.0, 0.0), {}, {}, 0.1);
        }
        require(std::abs(output.desiredTorqueBody.x) <= authority.x + 1.0e-12,
                "PID output respects actuator authority");
        controller.reset();

        ControlConfig lqr{};
        lqr.mode = ControlMode::Custom;
        lqr.customMethod = ControllerMethod::LQR;
        controller.configure(lqr, ControlVehicle::Satellite, inertia, authority);
        output = controller.update(Quaternion::identity(),
            Quaternion::fromEulerZYX(0.05, 0.0, 0.0), {}, {}, 0.02);
        require(output.active && output.method == ControllerMethod::LQR
                && output.desiredTorqueBody.x > 0.0, "LQR state feedback selection");
    });

    test("satellite four attitude objectives remain finite", [] {
        for (SatelliteObjective objective : {SatelliteObjective::NadirPointing,
                                             SatelliteObjective::InertialPointing,
                                             SatelliteObjective::TargetTracking,
                                             SatelliteObjective::SlewManeuver}) {
            SatelliteConfig config{};
            config.orbit.semiMajorAxisKm = 7100.0;
            config.orbit.eccentricity = 0.02;
            config.orbit.inclinationDeg = 45.0;
            config.orbit.raanDeg = 30.0;
            config.orbit.argumentOfPerigeeDeg = 15.0;
            config.orbit.trueAnomalyDeg = 20.0;
            config.mission.objective = objective;
            config.mission.targetEulerDeg = {5.0, -10.0, 20.0};
            config.mission.slewStartSec = 1.0;
            config.durationSec = 8.0;
            config.disturbanceEnabled = false;
            SatelliteSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            const SimulationSample& final = simulation.currentSample();
            require(std::isfinite(final.attitude.w) && std::isfinite(final.altitude),
                    "finite multi-objective satellite state");
            requireNear(final.attitude.norm(), 1.0, 1.0e-10, "normalized objective attitude");
        }
    });

    test("satellite orbit and closed-loop convergence", [] {
        SatelliteConfig config{};
        config.durationSec = 120.0;
        config.disturbanceEnabled = false;
        SatelliteSimulation simulation(config);
        const double initialError = attitudeErrorDeg(simulation.currentSample());
        while (!simulation.complete()) {
            simulation.step(0.02);
        }
        const SimulationSample& final = simulation.currentSample();
        require(std::abs(final.altitude - config.orbitAltitudeKm * 1000.0) < 50.0,
                "two-body circular orbit should retain altitude over short duration");
        require(attitudeErrorDeg(final) < 0.35, "nadir tracking should converge below 0.35 deg");
        require(attitudeErrorDeg(final) < initialError * 0.05, "closed loop should strongly reduce error");
        require(std::max({std::abs(final.wheelSpeedRpm.x), std::abs(final.wheelSpeedRpm.y),
                          std::abs(final.wheelSpeedRpm.z)}) <= config.wheelMaxSpeedRpm + 1.0e-6,
                "wheel speed saturation must be enforced");
    });

    test("satellite orbit control recovers from translational impulse", [] {
        auto run = [](bool orbitControlEnabled) {
            SatelliteConfig config{};
            config.durationSec = 1200.0;
            config.initialErrorDeg = {};
            config.initialRateDegPerSec = {};
            config.orbitControl.enabled = orbitControlEnabled;
            config.disturbances.constantEnabled = false;
            config.disturbances.pulseEnabled = false;
            config.disturbances.sineEnabled = false;
            config.disturbances.randomEnabled = false;
            config.disturbances.deltaVImpulseEnabled = true;
            config.disturbances.deltaVImpulseRtnMps = {0.0, 0.20, 0.0};
            config.disturbances.deltaVImpulseTimeSec = 60.0;
            SatelliteSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            return simulation;
        };
        const SatelliteSimulation openLoop = run(false);
        const SatelliteSimulation closedLoop = run(true);
        require(closedLoop.currentSample().translationalImpulseApplied,
                "configured RTN delta-v impulse must be applied once");
        requireNear(closedLoop.currentSample().appliedImpulseDeltaVMps, 0.20, 1.0e-12,
                    "applied RTN impulse magnitude");
        require(closedLoop.metrics().finalPositionError < openLoop.metrics().finalPositionError * 0.2,
                "closed-loop orbit control should strongly reduce position error");
        require(closedLoop.metrics().finalVelocityError < openLoop.metrics().finalVelocityError * 0.2,
                "closed-loop orbit control should strongly reduce velocity error");
        require(closedLoop.currentSample().propellantRemaining >= 0.0
                    && closedLoop.currentSample().mass >= 1.0,
                "propellant and wet mass remain physical");

        SatelliteConfig limitedConfig{};
        limitedConfig.durationSec = 80.0;
        limitedConfig.initialErrorDeg = {};
        limitedConfig.initialRateDegPerSec = {};
        limitedConfig.orbitControl.maxThrustN = 0.20;
        limitedConfig.orbitControl.propellantMassKg = 1.0e-6;
        limitedConfig.disturbances.constantEnabled = false;
        limitedConfig.disturbances.pulseEnabled = false;
        limitedConfig.disturbances.sineEnabled = false;
        limitedConfig.disturbances.deltaVImpulseEnabled = true;
        limitedConfig.disturbances.deltaVImpulseRtnMps = {0.0, 1.0, 0.0};
        limitedConfig.disturbances.deltaVImpulseTimeSec = 2.0;
        SatelliteSimulation limited(limitedConfig);
        while (!limited.complete()) limited.step(0.02);
        for (const SimulationSample& sample : limited.history()) {
            require(sample.thrust <= limitedConfig.orbitControl.maxThrustN + 1.0e-12,
                    "orbit-control thrust limit");
        }
        require(limited.currentSample().propellantDepleted
                    && limited.currentSample().propellantRemaining == 0.0,
                "orbit-control thrust must stop at propellant depletion");
    });

    test("rocket straight-thrust sanity", [] {
        RocketConfig config{};
        config.durationSec = 6.0;
        config.pitchStartSec = 20.0;
        config.pitchEndSec = 30.0;
        config.finalPitchDeg = 0.0;
        config.initialErrorDeg = {};
        config.initialRateDegPerSec = {};
        config.disturbanceEnabled = false;
        config.dragCoefficient = 0.0;
        RocketSimulation simulation(config);
        while (!simulation.complete()) {
            simulation.step(0.005);
        }
        const SimulationSample& final = simulation.currentSample();
        require(final.altitude > 50.0, "thrust-to-weight ratio should produce ascent");
        require(std::hypot(final.position.x, final.position.y) < 0.1,
                "zero TVC should remain vertical");
        require(final.mass < config.wetMassKg && final.mass >= config.dryMassKg,
                "mass depletion bounds");
    });

    test("rocket pitch tracking and TVC limits", [] {
        RocketConfig config{};
        config.durationSec = 28.0;
        config.pitchStartSec = 3.0;
        config.pitchEndSec = 18.0;
        config.finalPitchDeg = 18.0;
        config.disturbanceEnabled = false;
        RocketSimulation simulation(config);
        while (!simulation.complete()) {
            simulation.step(0.005);
        }
        const SimulationSample& final = simulation.currentSample();
        require(attitudeErrorDeg(final) < 1.0, "rocket attitude should track pitch program");
        require(std::abs(final.tvcPitchDeg) <= config.maxTvcAngleDeg + 1.0e-8,
                "TVC pitch angle limit");
        require(std::abs(final.tvcYawDeg) <= config.maxTvcAngleDeg + 1.0e-8,
                "TVC yaw angle limit");
    });

    test("default disturbance scenarios remain finite and recover", [] {
        SatelliteConfig satelliteConfig{};
        SatelliteSimulation satellite(satelliteConfig);
        while (!satellite.complete()) satellite.step(0.02);
        const SimulationSample& satelliteFinal = satellite.currentSample();
        const double satelliteError = attitudeErrorDeg(satelliteFinal);
        require(std::isfinite(satelliteError) && satelliteError < 0.5,
                "disturbed satellite final error = " + std::to_string(satelliteError) + " deg");

        RocketConfig rocketConfig{};
        RocketSimulation rocket(rocketConfig);
        while (!rocket.complete()) rocket.step(0.005);
        const SimulationSample& rocketFinal = rocket.currentSample();
        const double rocketError = attitudeErrorDeg(rocketFinal);
        require(std::isfinite(rocketError) && rocketError < 1.0,
                "disturbed rocket final error = " + std::to_string(rocketError) + " deg");
        require(std::isfinite(rocketFinal.altitude) && rocketFinal.altitude > 40000.0,
                "default ascent should exceed 40 km, altitude = " + std::to_string(rocketFinal.altitude));
        require(rocketFinal.mass >= rocketConfig.dryMassKg, "dry mass floor must hold");
    });

    test("disturbance components are selectable and deterministic", [] {
        SatelliteDisturbanceConfig satellite{};
        satellite.constantEnabled = true;
        satellite.constantTorqueNm = {1.0, -2.0, 3.0};
        satellite.pulseEnabled = false;
        satellite.sineEnabled = false;
        satellite.randomEnabled = false;
        requireNear((satelliteDisturbanceTorque(satellite, 12.0) - satellite.constantTorqueNm).norm(),
                    0.0, 1.0e-12, "constant satellite disturbance");
        satellite.constantEnabled = false;
        satellite.pulseEnabled = true;
        satellite.pulseAxis = {0.0, 2.0, 0.0};
        satellite.pulseMagnitudeNm = 4.0;
        satellite.pulseStartSec = 5.0;
        satellite.pulseDurationSec = 2.0;
        requireNear(satelliteDisturbanceTorque(satellite, 4.9).norm(), 0.0, 1.0e-12,
                    "pulse off before window");
        requireNear(satelliteDisturbanceTorque(satellite, 6.0).y, 4.0, 1.0e-12,
                    "pulse axis normalization");
        satellite.pulseEnabled = false;
        satellite.randomEnabled = true;
        satellite.randomSeed = 42;
        requireNear((satelliteDisturbanceTorque(satellite, 9.25)
                    - satelliteDisturbanceTorque(satellite, 9.25)).norm(), 0.0, 1.0e-15,
                    "seeded random disturbance repeatability");

        RocketDisturbanceConfig rocket{};
        rocket.gustEnabled = false;
        rocket.pulseTorqueEnabled = false;
        rocket.pulseForceEnabled = true;
        rocket.pulseForceDirectionEci = {0.0, 0.0, 3.0};
        rocket.pulseForceN = 1200.0;
        rocket.pulseForceStartSec = 8.0;
        rocket.pulseForceDurationSec = 1.5;
        const RocketDisturbanceOutput output = rocketDisturbance(rocket, 8.5, {6378137.0, 0.0, 0.0});
        requireNear(output.externalForceEciN.z, 1200.0, 1.0e-10, "rocket pulse force");
    });

    test("control OFF isolates control actuators", [] {
        SatelliteConfig satelliteConfig{};
        satelliteConfig.durationSec = 1.0;
        satelliteConfig.control.mode = ControlMode::Off;
        SatelliteSimulation satellite(satelliteConfig);
        while (!satellite.complete()) satellite.step(0.02);
        require(!satellite.currentSample().controlActive, "satellite control OFF state");
        requireNear(satellite.currentSample().controlTorque.norm(), 0.0, 1.0e-15,
                    "satellite control torque OFF");

        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        RocketMissionConfig rocketConfig{};
        rocketConfig.mission = repository.find(LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        rocketConfig.durationSec = 2.0;
        rocketConfig.control.mode = ControlMode::Off;
        RocketMissionSimulation rocket(rocketConfig);
        while (!rocket.complete()) rocket.step(0.02);
        require(!rocket.currentSample().controlActive, "rocket control OFF state");
        requireNear(rocket.currentSample().controlTorque.norm(), 0.0, 1.0e-12,
                    "rocket control torque OFF");
        requireNear(std::abs(rocket.currentSample().tvcPitchDeg)
                    + std::abs(rocket.currentSample().tvcYawDeg), 0.0, 1.0e-12,
                    "rocket TVC OFF");
    });

    test("CSV export schema and data", [] {
        SatelliteConfig config{};
        config.durationSec = 0.2;
        SatelliteSimulation simulation(config);
        while (!simulation.complete()) simulation.step(0.02);
        const std::filesystem::path path = std::filesystem::current_path() / "csv_export_test.csv";
        require(exportCsv(path.wstring(), ScenarioKind::Satellite, simulation.history()),
                "CSV export should succeed");
        std::ifstream stream(path, std::ios::binary);
        const std::string contents((std::istreambuf_iterator<char>(stream)),
                                   std::istreambuf_iterator<char>());
        require(contents.find("time_s,position_x_m") == 0, "CSV must begin with canonical schema");
        require(contents.find("wheel_x_rpm") != std::string::npos, "satellite actuator columns");
        require(contents.find("position_error_r_m") != std::string::npos
                    && contents.find("orbit_thrust_n") != std::string::npos
                    && contents.find("cumulative_delta_v_mps") != std::string::npos
                    && contents.find("delta_v_impulse_applied") != std::string::npos,
                "satellite CSV includes orbit-control, propellant, and impulse fields");
        require(std::count(contents.begin(), contents.end(), '\n') >= 2, "CSV must contain samples");
        stream.close();
        std::filesystem::remove(path);

        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        RocketMissionConfig rocketConfig{};
        rocketConfig.mission = repository.find(LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        rocketConfig.durationSec = 0.2;
        RocketMissionSimulation rocket(rocketConfig);
        while (!rocket.complete()) rocket.step(0.02);
        const std::filesystem::path rocketPath = std::filesystem::current_path() / "rocket_csv_export_test.csv";
        require(exportCsv(rocketPath.wstring(), ScenarioKind::Rocket, rocket.history()),
                "rocket CSV export should succeed");
        std::ifstream rocketStream(rocketPath, std::ios::binary);
        const std::string rocketContents((std::istreambuf_iterator<char>(rocketStream)),
                                         std::istreambuf_iterator<char>());
        require(rocketContents.find("guidance_pitch_correction_deg") != std::string::npos
                    && rocketContents.find("propellant_remaining_kg") != std::string::npos
                    && rocketContents.find("engine_cutoff") != std::string::npos,
                "rocket CSV includes guidance, propellant, and cutoff fields");
        rocketStream.close();
        std::filesystem::remove(rocketPath);
    });

    test("mission repository loads six immutable presets", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error),
                "mission repository load: " + error);
        require(repository.missions().size() == 6, "exactly six fixed missions");
        for (const RocketMissionData& mission : repository.missions()) {
            require(mission.trajectory.size() > 4500, "complete nominal trajectory samples");
            require(mission.endTimeSec() > 450.0, "nominal ascent duration");
            requireNear(mission.trajectory.front().referenceAttitude.norm(), 1.0, 1.0e-10,
                        "reference quaternion normalization");
            const double midpoint = 0.5 * (mission.startTimeSec() + mission.endTimeSec());
            const NominalTrajectoryPoint interpolated = mission.interpolate(midpoint);
            requireNear(interpolated.timeSec, midpoint, 1.0e-10, "time interpolation");
            requireNear(interpolated.referenceAttitude.norm(), 1.0, 1.0e-10, "SLERP normalization");
            require(interpolated.massKg > 0.0 && interpolated.positionEciM.norm() > 6.0e6,
                    "physical mission state");
        }
        const RocketMissionData* wenchang = repository.find(
            LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        require(wenchang != nullptr, "Wenchang 300 km mapping");
        requireNear(wenchang->summary.nominalWetMassKg(), 172000.0, 1.0e-8,
                    "nominal wet mass from mission summary");

        std::unordered_map<std::string, std::string> embeddedFiles;
        for (const auto& entry : std::filesystem::directory_iterator(GNC_TEST_DATA_DIR)) {
            if (!entry.is_regular_file() || entry.path().extension() != ".csv") continue;
            std::ifstream input(entry.path(), std::ios::binary);
            embeddedFiles.emplace(entry.path().filename().string(),
                std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()));
        }
        MissionRepository embeddedRepository;
        require(embeddedRepository.loadEmbedded(embeddedFiles, error),
                "embedded mission repository load: " + error);
        require(embeddedRepository.missions().size() == 6,
                "embedded repository contains all six missions");
        require(embeddedRepository.find(LaunchSite::CapeCanaveral,
                    TargetOrbit::Leo500Km51_6Deg) != nullptr,
                "embedded Cape Canaveral 500 km mapping");
    });

    test("rocket truth model follows nominal and enters orbital coast", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        const RocketMissionData* mission = repository.find(
            LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        require(mission != nullptr, "mission lookup");
        RocketMissionConfig config{};
        config.mission = mission;
        config.durationSec = mission->endTimeSec() + 20.0;
        config.initialAttitudeErrorDeg = {0.2, -0.3, 0.15};
        config.initialRateErrorDegPerSec = {};
        RocketMissionSimulation simulation(config);
        while (!simulation.complete()) simulation.step(0.02);
        const SimulationSample& final = simulation.currentSample();
        require(final.orbitalCoast, "coast phase must activate after nominal file ends");
        require(final.thrust == 0.0 && final.controlTorque.norm() == 0.0,
                "thrust and TVC torque inactive during coast");
        require(std::isnan(final.positionError) && std::isfinite(final.altitude),
                "ascent tracking error is unavailable while coast state remains finite");
        require(simulation.hasInsertionSnapshot(), "insertion snapshot captured");
        require(std::isfinite(simulation.metrics().terminalAltitudeError),
                "insertion performance remains available after coast begins");
        require(final.mass >= (mission->summary.payloadMassKg + mission->summary.stage2DryMassKg),
                "dry mass floor");
        require(attitudeErrorDeg(final) < 5.0, "attitude remains bounded through coast");
    });

    test("all six rocket missions propagate through file end", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        for (const RocketMissionData& mission : repository.missions()) {
            RocketMissionConfig config{};
            config.mission = &mission;
            config.durationSec = mission.endTimeSec() + 1.0;
            config.initialAttitudeErrorDeg = {0.1, -0.1, 0.05};
            config.initialRateErrorDegPerSec = {};
            RocketMissionSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            const auto& final = simulation.currentSample();
            require(final.orbitalCoast, "orbital coast for mission " + mission.missionKey);
            require(std::isnan(final.positionError) && std::isnan(final.velocityError)
                    && final.position.norm() > 6.0e6,
                    "coast state is finite without a misleading ascent error for mission "
                        + mission.missionKey);
            require(final.stageId == 4 && final.thrust == 0.0,
                    "terminal unpowered stage for mission " + mission.missionKey);
        }
    });

    test("rocket insertion metrics are frozen during orbital coast", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        const RocketMissionData* mission = repository.find(
            LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        require(mission != nullptr, "mission lookup");

        auto run = [&](double coastDurationSec) {
            RocketMissionConfig config{};
            config.mission = mission;
            config.durationSec = mission->endTimeSec() + coastDurationSec;
            config.initialAttitudeErrorDeg = {0.2, -0.3, 0.15};
            RocketMissionSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            return simulation.metrics();
        };

        const PerformanceMetrics early = run(1.0);
        const PerformanceMetrics late = run(50.0);
        requireNear(late.terminalAltitudeError, early.terminalAltitudeError, 1.0e-9,
                    "frozen insertion altitude error");
        requireNear(late.terminalVelocityError, early.terminalVelocityError, 1.0e-9,
                    "frozen insertion velocity error");
        requireNear(late.terminalSemiMajorAxisErrorKm, early.terminalSemiMajorAxisErrorKm,
                    1.0e-9, "frozen insertion semi-major axis error");
        requireNear(late.terminalEccentricityError, early.terminalEccentricityError,
                    1.0e-12, "frozen insertion eccentricity error");
    });

    test("rocket guidance closes trajectory loop and respects stage propellant", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        const RocketMissionData* mission = repository.find(
            LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        require(mission != nullptr, "mission lookup");

        auto run = [&](bool guidanceEnabled, double thrustDeviation = 5.0) {
            RocketMissionConfig config{};
            config.mission = mission;
            config.durationSec = mission->endTimeSec() + 100.0;
            config.initialAttitudeErrorDeg = {};
            config.initialRateErrorDegPerSec = {};
            config.deviations.thrustPercent = thrustDeviation;
            config.guidance.enabled = guidanceEnabled;
            config.disturbances.gustEnabled = false;
            config.disturbances.pulseTorqueEnabled = false;
            RocketMissionSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            return simulation;
        };

        const RocketMissionSimulation openLoop = run(false);
        const RocketMissionSimulation closedLoop = run(true);
        const PerformanceMetrics openMetrics = openLoop.metrics();
        const PerformanceMetrics closedMetrics = closedLoop.metrics();
        require(openLoop.insertionSample().mass
                    > mission->summary.payloadMassKg + mission->summary.stage2DryMassKg,
                "positive thrust deviation must not burn through stage dry mass");
        const auto stage1Burnout = std::find_if(openLoop.history().begin(), openLoop.history().end(),
            [](const SimulationSample& sample) {
                return sample.stageId == 1 && sample.propellantDepleted && sample.thrust == 0.0;
            });
        require(stage1Burnout != openLoop.history().end()
                    && stage1Burnout->mass >= mission->summary.payloadMassKg
                        + mission->summary.stage1DryMassKg + mission->summary.stage2DryMassKg
                        + mission->summary.stage2PropellantMassKg - 1.0e-6,
                "stage 1 depletion must coast without consuming stage 2 or structural mass");
        require(closedLoop.insertionSample().engineCutoff
                    && closedLoop.insertionSample().thrust == 0.0,
                "adaptive insertion cutoff must remove thrust");
        require(std::abs(closedMetrics.terminalSemiMajorAxisErrorKm)
                    < std::abs(openMetrics.terminalSemiMajorAxisErrorKm),
                "closed-loop guidance should reduce terminal semi-major-axis error");
    });

    test("rocket terminal guidance covers all six missions and thrust deviations", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        for (const RocketMissionData& mission : repository.missions()) {
            for (double thrustDeviation : {-5.0, 5.0}) {
                RocketMissionConfig config{};
                config.mission = &mission;
                config.durationSec = mission.endTimeSec() + 100.0;
                config.initialAttitudeErrorDeg = {};
                config.initialRateErrorDegPerSec = {};
                config.deviations.thrustPercent = thrustDeviation;
                config.disturbances.gustEnabled = false;
                config.disturbances.pulseTorqueEnabled = false;
                RocketMissionSimulation simulation(config);
                while (!simulation.complete()) simulation.step(0.02);
                const PerformanceMetrics metrics = simulation.metrics();
                const std::string caseName = mission.missionKey + " thrust "
                    + std::to_string(thrustDeviation) + "%";
                require(simulation.hasInsertionSnapshot(), "insertion captured: " + caseName);
                require(simulation.insertionSample().engineCutoff, "engine cutoff: " + caseName);
                require(simulation.insertionSample().mass
                            >= mission.summary.payloadMassKg + mission.summary.stage2DryMassKg,
                        "stage dry mass protected: " + caseName);
                require(std::isfinite(metrics.terminalSemiMajorAxisErrorKm)
                            && std::abs(metrics.terminalSemiMajorAxisErrorKm) < 10.0,
                        "terminal semi-major-axis error below 10 km: " + caseName
                            + ", actual=" + std::to_string(metrics.terminalSemiMajorAxisErrorKm));
            }
        }
    });

    test("rocket attitude-controller gains affect guided trajectory", [] {
        MissionRepository repository;
        std::string error;
        require(repository.load(std::filesystem::path(GNC_TEST_DATA_DIR), error), error);
        const RocketMissionData* mission = repository.find(
            LaunchSite::Wenchang, TargetOrbit::Leo300Km28_5Deg);
        require(mission != nullptr, "mission lookup");
        const double authority = mission->summary.stage1ThrustN * mission->summary.thrustLeverArmM
            * std::sin(mission->summary.maxTvcAngleDeg * kDegToRad);
        const ControlRecommendation recommendation = recommendControlGains(
            ControlVehicle::LaunchVehicle, mission->summary.nominalWetInertiaKgM2,
            {mission->summary.maxRollRcsTorqueNm, authority, authority});

        auto run = [&](double gainScale) {
            RocketMissionConfig config{};
            config.mission = mission;
            config.durationSec = 200.0;
            config.control.mode = ControlMode::Custom;
            config.control.customMethod = ControllerMethod::PD;
            config.control.proportionalGain = recommendation.proportionalGain * gainScale;
            config.control.derivativeGain = recommendation.derivativeGain * gainScale;
            config.disturbances.gustEnabled = false;
            config.disturbances.pulseTorqueEnabled = false;
            RocketMissionSimulation simulation(config);
            while (!simulation.complete()) simulation.step(0.02);
            return simulation.currentSample();
        };
        const SimulationSample normal = run(1.0);
        const SimulationSample weak = run(0.05);
        require((normal.position - weak.position).norm() > 10.0,
                "changing inner-loop gains must change the flown trajectory");
        require(std::isfinite(normal.altitude) && std::isfinite(weak.altitude),
                "both controller configurations remain finite");
    });

    std::cout << "\n" << gPassed << " passed, " << gFailed << " failed\n";
    return gFailed == 0 ? 0 : 1;
}
