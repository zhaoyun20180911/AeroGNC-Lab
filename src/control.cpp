#include "gnc/control.hpp"

#include <algorithm>
#include <cmath>

namespace gnc {
namespace {

double component(const Vec3& value, int axis) {
    return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

void setComponent(Vec3& value, int axis, double componentValue) {
    if (axis == 0) value.x = componentValue;
    else if (axis == 1) value.y = componentValue;
    else value.z = componentValue;
}

Vec3 safeAuthority(const Vec3& authority) {
    return {std::max(1.0e-12, std::abs(authority.x)),
            std::max(1.0e-12, std::abs(authority.y)),
            std::max(1.0e-12, std::abs(authority.z))};
}

} // namespace

ControlRecommendation recommendControlGains(ControlVehicle vehicle,
                                             const Vec3& inertiaDiagonalKgM2,
                                             const Vec3& actuatorAuthorityNm) {
    const double naturalFrequency = vehicle == ControlVehicle::Satellite ? 0.055 : 0.68;
    const double dampingRatio = vehicle == ControlVehicle::Satellite ? 0.95 : 0.92;
    const Vec3 authority = safeAuthority(actuatorAuthorityNm);
    ControlRecommendation result{};
    for (int axis = 0; axis < 3; ++axis) {
        const double inertia = std::max(1.0e-9, component(inertiaDiagonalKgM2, axis));
        const double nominalKp = inertia * naturalFrequency * naturalFrequency;
        const double authorityLimitedKp = 0.80 * component(authority, axis)
                                        / (vehicle == ControlVehicle::Satellite ? 0.35 : 0.20);
        const double kp = std::min(nominalKp, authorityLimitedKp);
        const double effectiveFrequency = std::sqrt(kp / inertia);
        const double kd = 2.0 * dampingRatio * inertia * effectiveFrequency;
        const double ki = kp * effectiveFrequency * 0.12;
        const double integralLimit = 0.45 * component(authority, axis) / std::max(1.0e-12, ki);
        setComponent(result.proportionalGain, axis, kp);
        setComponent(result.derivativeGain, axis, kd);
        setComponent(result.integralGain, axis, ki);
        setComponent(result.integralLimit, axis, integralLimit);
    }
    result.attitudeErrorWeight = vehicle == ControlVehicle::Satellite ? 10.0 : 16.0;
    result.angularRateWeight = vehicle == ControlVehicle::Satellite ? 2.0 : 4.0;
    result.controlEffortWeight = 1.0;
    return result;
}

void AttitudeController::configure(const ControlConfig& config, ControlVehicle vehicle,
                                   const Vec3& inertiaDiagonalKgM2,
                                   const Vec3& actuatorAuthorityNm) {
    config_ = config;
    vehicle_ = vehicle;
    actuatorAuthorityNm_ = safeAuthority(actuatorAuthorityNm);
    recommendation_ = recommendControlGains(vehicle, inertiaDiagonalKgM2, actuatorAuthorityNm_);
    reset();
}

void AttitudeController::reset() {
    integralState_ = {};
}

ControllerMethod AttitudeController::effectiveMethod() const {
    if (config_.mode == ControlMode::Default) {
        return vehicle_ == ControlVehicle::Satellite ? ControllerMethod::PD : ControllerMethod::LQR;
    }
    return config_.customMethod;
}

ControllerOutput AttitudeController::update(const Quaternion& currentBodyToInertial,
                                            const Quaternion& targetBodyToInertial,
                                            const Vec3& currentAngularRateBody,
                                            const Vec3& targetAngularRateBody,
                                            double dt) {
    ControllerOutput output{};
    output.method = effectiveMethod();
    const Quaternion errorQuaternion = attitudeError(currentBodyToInertial, targetBodyToInertial);
    output.attitudeErrorRadians = errorQuaternion.vector() * 2.0;
    if (config_.mode == ControlMode::Off) {
        integralState_ = {};
        return output;
    }
    output.active = true;
    const Vec3 rateError = currentAngularRateBody - targetAngularRateBody;

    Vec3 kp = recommendation_.proportionalGain;
    Vec3 ki = recommendation_.integralGain;
    Vec3 kd = recommendation_.derivativeGain;
    Vec3 integralLimit = recommendation_.integralLimit;
    if (config_.mode == ControlMode::Custom && output.method != ControllerMethod::LQR) {
        kp = config_.proportionalGain;
        kd = config_.derivativeGain;
        if (output.method == ControllerMethod::PID) {
            ki = config_.integralGain;
            integralLimit = config_.integralLimit;
        } else {
            ki = {};
        }
    }
    if (output.method == ControllerMethod::LQR) {
        const double attitudeScale = std::sqrt(std::max(1.0e-9, config_.mode == ControlMode::Default
            ? recommendation_.attitudeErrorWeight : config_.attitudeErrorWeight)
            / recommendation_.attitudeErrorWeight);
        const double rateScale = std::sqrt(std::max(1.0e-9, config_.mode == ControlMode::Default
            ? recommendation_.angularRateWeight : config_.angularRateWeight)
            / recommendation_.angularRateWeight);
        const double effort = std::sqrt(std::max(1.0e-9, config_.mode == ControlMode::Default
            ? recommendation_.controlEffortWeight : config_.controlEffortWeight)
            / recommendation_.controlEffortWeight);
        kp = recommendation_.proportionalGain * (attitudeScale / effort);
        kd = recommendation_.derivativeGain * (rateScale / effort);
        ki = {};
    }

    Vec3 candidateIntegral = integralState_;
    if (output.method == ControllerMethod::PID && dt > 0.0) {
        candidateIntegral += output.attitudeErrorRadians * dt;
        candidateIntegral.x = clamp(candidateIntegral.x, -std::abs(integralLimit.x), std::abs(integralLimit.x));
        candidateIntegral.y = clamp(candidateIntegral.y, -std::abs(integralLimit.y), std::abs(integralLimit.y));
        candidateIntegral.z = clamp(candidateIntegral.z, -std::abs(integralLimit.z), std::abs(integralLimit.z));
    }

    Vec3 torque{};
    for (int axis = 0; axis < 3; ++axis) {
        const double raw = component(kp, axis) * component(output.attitudeErrorRadians, axis)
            + component(ki, axis) * component(candidateIntegral, axis)
            - component(kd, axis) * component(rateError, axis);
        const double limited = clamp(raw, -component(actuatorAuthorityNm_, axis),
                                     component(actuatorAuthorityNm_, axis));
        if (std::abs(raw - limited) > 1.0e-10) {
            output.torqueLimited = true;
            if (output.method == ControllerMethod::PID && config_.antiWindup
                && raw * component(output.attitudeErrorRadians, axis) > 0.0) {
                setComponent(candidateIntegral, axis, component(integralState_, axis));
            }
        }
        setComponent(torque, axis, limited);
    }
    if (output.method == ControllerMethod::PID) integralState_ = candidateIntegral;
    else integralState_ = {};
    output.desiredTorqueBody = torque;
    return output;
}

} // namespace gnc

