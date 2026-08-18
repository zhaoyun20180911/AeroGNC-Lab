#include "gnc/orbit.hpp"

#include <cmath>

namespace gnc {
namespace {

Mat3 rotationX(double angle) {
    Mat3 result = Mat3::identity();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    result.m[1][1] = c; result.m[1][2] = -s;
    result.m[2][1] = s; result.m[2][2] = c;
    return result;
}

Mat3 rotationZ(double angle) {
    Mat3 result = Mat3::identity();
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    result.m[0][0] = c; result.m[0][1] = -s;
    result.m[1][0] = s; result.m[1][1] = c;
    return result;
}

double wrappedDegrees(double radians) {
    double degrees = radians * kRadToDeg;
    while (degrees < 0.0) degrees += 360.0;
    while (degrees >= 360.0) degrees -= 360.0;
    return degrees;
}

} // namespace

bool validOrbitElements(const ClassicalOrbitElements& elements) {
    return std::isfinite(elements.semiMajorAxisKm)
        && std::isfinite(elements.eccentricity)
        && elements.semiMajorAxisKm * 1000.0 > kEarthEquatorialRadiusM
        && elements.eccentricity >= 0.0 && elements.eccentricity < 1.0
        && elements.semiMajorAxisKm * 1000.0 * (1.0 - elements.eccentricity) > kEarthEquatorialRadiusM;
}

CartesianOrbitState classicalElementsToCartesian(const ClassicalOrbitElements& elements,
                                                  double gravitationalParameter) {
    const double semiMajorAxis = elements.semiMajorAxisKm * 1000.0;
    const double eccentricity = clamp(elements.eccentricity, 0.0, 0.999999999);
    const double inclination = elements.inclinationDeg * kDegToRad;
    const double raan = elements.raanDeg * kDegToRad;
    const double argument = elements.argumentOfPerigeeDeg * kDegToRad;
    const double anomaly = elements.trueAnomalyDeg * kDegToRad;
    const double semilatusRectum = semiMajorAxis * (1.0 - eccentricity * eccentricity);
    const double radius = semilatusRectum / (1.0 + eccentricity * std::cos(anomaly));
    const Vec3 positionPerifocal{radius * std::cos(anomaly), radius * std::sin(anomaly), 0.0};
    const double velocityScale = std::sqrt(gravitationalParameter / semilatusRectum);
    const Vec3 velocityPerifocal{-velocityScale * std::sin(anomaly),
                                 velocityScale * (eccentricity + std::cos(anomaly)), 0.0};
    const Mat3 perifocalToEci = rotationZ(raan) * rotationX(inclination) * rotationZ(argument);
    return {perifocalToEci * positionPerifocal, perifocalToEci * velocityPerifocal};
}

ClassicalOrbitElements cartesianToClassicalElements(const Vec3& position,
                                                     const Vec3& velocity,
                                                     double gravitationalParameter) {
    ClassicalOrbitElements elements{};
    const double radius = position.norm();
    const double speedSquared = velocity.normSquared();
    const Vec3 angularMomentum = position.cross(velocity);
    const double h = angularMomentum.norm();
    const Vec3 node = Vec3{0.0, 0.0, 1.0}.cross(angularMomentum);
    const double n = node.norm();
    const Vec3 eccentricityVector = velocity.cross(angularMomentum) / gravitationalParameter
                                  - position / radius;
    const double eccentricity = eccentricityVector.norm();
    const double specificEnergy = 0.5 * speedSquared - gravitationalParameter / radius;
    elements.semiMajorAxisKm = -gravitationalParameter / (2.0 * specificEnergy) / 1000.0;
    elements.eccentricity = eccentricity;
    elements.inclinationDeg = std::acos(clamp(angularMomentum.z / std::max(1.0e-15, h), -1.0, 1.0))
                            * kRadToDeg;
    elements.raanDeg = n > 1.0e-12 ? wrappedDegrees(std::atan2(node.y, node.x)) : 0.0;
    if (n > 1.0e-12 && eccentricity > 1.0e-12) {
        const double cosine = clamp(node.dot(eccentricityVector) / (n * eccentricity), -1.0, 1.0);
        const double sine = node.cross(eccentricityVector).dot(angularMomentum) / (n * eccentricity * h);
        elements.argumentOfPerigeeDeg = wrappedDegrees(std::atan2(sine, cosine));
    } else {
        elements.argumentOfPerigeeDeg = 0.0;
    }
    if (eccentricity > 1.0e-12) {
        const double cosine = clamp(eccentricityVector.dot(position) / (eccentricity * radius), -1.0, 1.0);
        const double sine = eccentricityVector.cross(position).dot(angularMomentum)
                          / (eccentricity * radius * h);
        elements.trueAnomalyDeg = wrappedDegrees(std::atan2(sine, cosine));
    } else if (n > 1.0e-12) {
        const double cosine = clamp(node.dot(position) / (n * radius), -1.0, 1.0);
        const double sine = node.cross(position).dot(angularMomentum) / (n * radius * h);
        elements.trueAnomalyDeg = wrappedDegrees(std::atan2(sine, cosine));
    } else {
        elements.trueAnomalyDeg = wrappedDegrees(std::atan2(position.y, position.x));
    }
    return elements;
}

} // namespace gnc

