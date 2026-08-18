#pragma once

#include "gnc/config.hpp"

namespace gnc {

constexpr double kEarthGravitationalParameter = 3.986004418e14;
constexpr double kEarthEquatorialRadiusM = 6378137.0;

struct CartesianOrbitState {
    Vec3 positionEciM{};
    Vec3 velocityEciMps{};
};

[[nodiscard]] CartesianOrbitState classicalElementsToCartesian(
    const ClassicalOrbitElements& elements,
    double gravitationalParameter = kEarthGravitationalParameter);

[[nodiscard]] ClassicalOrbitElements cartesianToClassicalElements(
    const Vec3& positionEciM,
    const Vec3& velocityEciMps,
    double gravitationalParameter = kEarthGravitationalParameter);

[[nodiscard]] bool validOrbitElements(const ClassicalOrbitElements& elements);

} // namespace gnc

