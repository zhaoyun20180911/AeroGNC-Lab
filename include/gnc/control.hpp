#pragma once

#include "gnc/config.hpp"

namespace gnc {

enum class ControlVehicle { Satellite, LaunchVehicle };

struct ControlRecommendation {
    Vec3 proportionalGain{};
    Vec3 integralGain{};
    Vec3 derivativeGain{};
    Vec3 integralLimit{};
    double attitudeErrorWeight{10.0};
    double angularRateWeight{2.0};
    double controlEffortWeight{1.0};
};

struct ControllerOutput {
    Vec3 desiredTorqueBody{};
    Vec3 attitudeErrorRadians{};
    ControllerMethod method{ControllerMethod::PD};
    bool active{};
    bool torqueLimited{};
};

[[nodiscard]] ControlRecommendation recommendControlGains(
    ControlVehicle vehicle, const Vec3& inertiaDiagonalKgM2, const Vec3& actuatorAuthorityNm);

class AttitudeController {
public:
    void configure(const ControlConfig& config, ControlVehicle vehicle,
                   const Vec3& inertiaDiagonalKgM2, const Vec3& actuatorAuthorityNm);
    void reset();
    [[nodiscard]] ControllerOutput update(
        const Quaternion& currentBodyToInertial,
        const Quaternion& targetBodyToInertial,
        const Vec3& currentAngularRateBody,
        const Vec3& targetAngularRateBody,
        double dt);

    [[nodiscard]] const ControlRecommendation& recommendation() const { return recommendation_; }
    [[nodiscard]] ControllerMethod effectiveMethod() const;
    [[nodiscard]] bool active() const { return config_.mode != ControlMode::Off; }

private:
    ControlConfig config_{};
    ControlVehicle vehicle_{ControlVehicle::Satellite};
    Vec3 actuatorAuthorityNm_{};
    ControlRecommendation recommendation_{};
    Vec3 integralState_{};
};

} // namespace gnc

