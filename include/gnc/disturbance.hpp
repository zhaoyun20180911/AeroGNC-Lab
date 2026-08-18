#pragma once

#include "gnc/config.hpp"

namespace gnc {

struct RocketDisturbanceOutput {
    Vec3 windVelocityEciMps{};
    Vec3 externalForceEciN{};
    Vec3 externalTorqueBodyNm{};
};

[[nodiscard]] Vec3 satelliteDisturbanceTorque(
    const SatelliteDisturbanceConfig& config, double timeSec);

[[nodiscard]] RocketDisturbanceOutput rocketDisturbance(
    const RocketDisturbanceConfig& config, double timeSec, const Vec3& positionEciM);

} // namespace gnc

