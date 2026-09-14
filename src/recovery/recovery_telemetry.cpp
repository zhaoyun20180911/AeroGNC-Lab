#include "gnc/recovery.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace gnc {
namespace {

double maximumAbsolute(const Vec3& value) {
    return std::max({std::abs(value.x), std::abs(value.y), std::abs(value.z)});
}

double touchdownTilt(const Quaternion& bodyToEcef, const NedFrame& frame) {
    const Vec3 bodyAxisEcef = bodyToEcef.rotate({1.0, 0.0, 0.0}).normalized();
    const Vec3 localUpEcef = -frame.downEcef;
    return std::acos(clamp(bodyAxisEcef.dot(localUpEcef), -1.0, 1.0)) * kRadToDeg;
}

} // namespace

void RecoverySimulation::recordSample(bool force) {
    Vec3 positionNed = ecefPositionToNed(truth_.positionEcefM, frame_);
    Vec3 velocityNed = ecefVectorToNed(truth_.velocityEcefMps, frame_);
    if (touchdownCaptured_) {
        positionNed = touchdownPositionNed_;
        velocityNed = touchdownVelocityNed_;
    }
    const Quaternion reference = guidance_.desiredAttitude.normSquared() > 0.5
        ? guidance_.desiredAttitude : truth_.bodyToEcef;
    const Quaternion error = attitudeError(truth_.bodyToEcef, reference);
    const double tvcLimit = config_.vehicle.tvcLimitDeg;
    const double finLimit = config_.vehicle.gridFinLimitDeg;

    currentSample_ = {};
    currentSample_.time = truth_.time;
    currentSample_.position = truth_.positionEcefM;
    currentSample_.velocity = truth_.velocityEcefMps;
    currentSample_.attitude = truth_.bodyToEcef;
    currentSample_.referenceAttitude = reference;
    currentSample_.eulerDeg = truth_.bodyToEcef.toEulerZYX() * kRadToDeg;
    currentSample_.referenceEulerDeg = reference.toEulerZYX() * kRadToDeg;
    currentSample_.attitudeErrorDeg = error.toEulerZYX() * kRadToDeg;
    currentSample_.angularRateDegPerSec = truth_.angularRateBodyRadPerSec * kRadToDeg;
    currentSample_.controlTorque = lastControlTorqueNm_;
    currentSample_.mass = truth_.massKg;
    currentSample_.altitude = std::max(0.0, -positionNed.z);
    currentSample_.speed = velocityNed.norm();
    currentSample_.dynamicPressure = lastDynamicPressurePa_;
    currentSample_.tvcPitchDeg = lastActuator_.tvcPitchRad * kRadToDeg;
    currentSample_.tvcYawDeg = lastActuator_.tvcYawRad * kRadToDeg;
    currentSample_.thrust = lastThrustN_;
    currentSample_.propellantRemaining = truth_.propellantKg;
    currentSample_.positionNed = positionNed;
    currentSample_.velocityNed = velocityNed;
    currentSample_.predictedLandingErrorNed = prediction_.errorNedM;
    for (int i = 0; i < 4; ++i) {
        currentSample_.gridFinDeflectionDeg[i] = lastActuator_.gridFinRad[i] * kRadToDeg;
    }
    currentSample_.groundDistance = std::hypot(positionNed.x, positionNed.y);
    currentSample_.horizontalVelocity = std::hypot(velocityNed.x, velocityNed.y);
    currentSample_.verticalVelocity = velocityNed.z;
    currentSample_.landingError = currentSample_.groundDistance;
    currentSample_.brakingDistance = lastBrakingDistanceM_;
    currentSample_.optimizerSolveTimeMs = lastSolverTimeMs_;
    currentSample_.optimizerSuccessRate = solverCalls_ > 0
        ? static_cast<double>(solverSuccesses_) / static_cast<double>(solverCalls_) : 0.0;
    currentSample_.touchdownTiltDeg = touchdownTilt(truth_.bodyToEcef, frame_);
    currentSample_.recoveryPhase = static_cast<int>(mission_.phase());
    currentSample_.optimizerStatus = static_cast<int>(lastSolverStatus_);
    currentSample_.optimizerFallback = lastSolverFallback_;
    currentSample_.recoveryFailed = mission_.phase() == RecoveryPhase::Failed;
    currentSample_.engineCutoff = !engine_.active();
    currentSample_.propellantDepleted = truth_.propellantKg <= 1.0e-9;
    currentSample_.guidanceActive = mission_.phase() != RecoveryPhase::Touchdown
                                 && mission_.phase() != RecoveryPhase::Failed;
    currentSample_.controlActive = currentSample_.guidanceActive;
    currentSample_.actuatorSaturated = std::abs(currentSample_.tvcPitchDeg) >= tvcLimit - 1.0e-3
        || std::abs(currentSample_.tvcYawDeg) >= tvcLimit - 1.0e-3;
    for (double fin : currentSample_.gridFinDeflectionDeg) {
        currentSample_.actuatorSaturated = currentSample_.actuatorSaturated
            || std::abs(fin) >= finLimit - 1.0e-3;
    }

    if (force || truth_.time + 1.0e-10 >= nextSampleTimeSec_) {
        if (history_.empty() || std::abs(history_.back().time - truth_.time) > 1.0e-9) {
            history_.push_back(currentSample_);
        } else {
            history_.back() = currentSample_;
        }
        nextSampleTimeSec_ = truth_.time + 0.10;
    }
}

RecoveryMetrics RecoverySimulation::metrics() const {
    RecoveryMetrics result{};
    result.finalPhase = mission_.phase();
    result.failureReason = mission_.failureReason();
    result.propellantConsumedKg = initialPropellantKg_ - truth_.propellantKg;
    result.recoveryDurationSec = truth_.time - config_.initialState.time;
    result.meanOptimizerSolveTimeMs = solverCalls_ > 0
        ? totalSolverTimeMs_ / static_cast<double>(solverCalls_) : 0.0;
    result.maxOptimizerSolveTimeMs = maxSolverTimeMs_;
    result.solverSuccessRate = solverCalls_ > 0
        ? static_cast<double>(solverSuccesses_) / static_cast<double>(solverCalls_) : 0.0;
    for (const SimulationSample& point : history_) {
        result.maxAttitudeErrorDeg = std::max(result.maxAttitudeErrorDeg,
            quaternionAngularDistance(point.attitude, point.referenceAttitude) * kRadToDeg);
        result.maxTvcAngleDeg = std::max(result.maxTvcAngleDeg,
            std::max(std::abs(point.tvcPitchDeg), std::abs(point.tvcYawDeg)));
        for (double fin : point.gridFinDeflectionDeg) {
            result.maxGridFinDeflectionDeg = std::max(result.maxGridFinDeflectionDeg,
                                                       std::abs(fin));
        }
    }
    const Vec3 finalPosition = touchdownCaptured_
        ? touchdownPositionNed_ : ecefPositionToNed(truth_.positionEcefM, frame_);
    const Vec3 finalVelocity = touchdownCaptured_
        ? touchdownVelocityNed_ : ecefVectorToNed(truth_.velocityEcefMps, frame_);
    result.landingPositionErrorM = std::hypot(finalPosition.x, finalPosition.y);
    result.horizontalTouchdownVelocityMps = std::hypot(finalVelocity.x, finalVelocity.y);
    result.verticalTouchdownVelocityMps = finalVelocity.z;
    result.touchdownTiltDeg = touchdownTilt(truth_.bodyToEcef, frame_);
    return result;
}

bool exportRecoveryCsv(const std::wstring& path, const std::vector<SimulationSample>& samples) {
    std::ofstream stream(std::filesystem::path(path), std::ios::binary);
    if (!stream) return false;
    stream << "time_s,phase,ecef_x_m,ecef_y_m,ecef_z_m,ecef_vx_mps,ecef_vy_mps,ecef_vz_mps,"
              "ned_north_m,ned_east_m,ned_down_m,ned_vnorth_mps,ned_veast_mps,ned_vdown_mps,"
              "predicted_error_north_m,predicted_error_east_m,predicted_error_down_m,"
              "ground_distance_m,horizontal_velocity_mps,vertical_velocity_mps,altitude_m,speed_mps,"
              "q_w,q_x,q_y,q_z,ref_q_w,ref_q_x,ref_q_y,ref_q_z,"
              "omega_x_deg_s,omega_y_deg_s,omega_z_deg_s,torque_x_nm,torque_y_nm,torque_z_nm,"
              "mass_kg,propellant_kg,thrust_n,dynamic_pressure_pa,tvc_pitch_deg,tvc_yaw_deg,"
              "grid_fin_1_deg,grid_fin_2_deg,grid_fin_3_deg,grid_fin_4_deg,braking_distance_m,"
              "solver_status,solver_time_ms,solver_success_rate,solver_fallback,touchdown_tilt_deg,failed\r\n";
    stream << std::setprecision(12);
    for (const SimulationSample& s : samples) {
        stream << s.time << ',' << s.recoveryPhase << ','
               << s.position.x << ',' << s.position.y << ',' << s.position.z << ','
               << s.velocity.x << ',' << s.velocity.y << ',' << s.velocity.z << ','
               << s.positionNed.x << ',' << s.positionNed.y << ',' << s.positionNed.z << ','
               << s.velocityNed.x << ',' << s.velocityNed.y << ',' << s.velocityNed.z << ','
               << s.predictedLandingErrorNed.x << ',' << s.predictedLandingErrorNed.y << ','
               << s.predictedLandingErrorNed.z << ',' << s.groundDistance << ','
               << s.horizontalVelocity << ',' << s.verticalVelocity << ',' << s.altitude << ','
               << s.speed << ',' << s.attitude.w << ',' << s.attitude.x << ',' << s.attitude.y << ','
               << s.attitude.z << ',' << s.referenceAttitude.w << ',' << s.referenceAttitude.x << ','
               << s.referenceAttitude.y << ',' << s.referenceAttitude.z << ','
               << s.angularRateDegPerSec.x << ',' << s.angularRateDegPerSec.y << ','
               << s.angularRateDegPerSec.z << ',' << s.controlTorque.x << ','
               << s.controlTorque.y << ',' << s.controlTorque.z << ',' << s.mass << ','
               << s.propellantRemaining << ',' << s.thrust << ',' << s.dynamicPressure << ','
               << s.tvcPitchDeg << ',' << s.tvcYawDeg << ',' << s.gridFinDeflectionDeg[0] << ','
               << s.gridFinDeflectionDeg[1] << ',' << s.gridFinDeflectionDeg[2] << ','
               << s.gridFinDeflectionDeg[3] << ',' << s.brakingDistance << ','
               << s.optimizerStatus << ',' << s.optimizerSolveTimeMs << ','
               << s.optimizerSuccessRate << ',' << (s.optimizerFallback ? 1 : 0) << ','
               << s.touchdownTiltDeg << ',' << (s.recoveryFailed ? 1 : 0) << "\r\n";
    }
    return static_cast<bool>(stream);
}

} // namespace gnc
