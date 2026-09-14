#include "gnc/recovery.hpp"

#include <cmath>

namespace gnc {

NedFrame makeLandingNedFrame(const LandingSite& site) {
    const double latitude = site.latitudeDeg * kDegToRad;
    const double longitude = site.longitudeDeg * kDegToRad;
    const double sinLat = std::sin(latitude), cosLat = std::cos(latitude);
    const double sinLon = std::sin(longitude), cosLon = std::cos(longitude);
    NedFrame frame{};
    frame.northEcef = {-sinLat * cosLon, -sinLat * sinLon, cosLat};
    frame.eastEcef = {-sinLon, cosLon, 0.0};
    frame.downEcef = {-cosLat * cosLon, -cosLat * sinLon, -sinLat};
    frame.originEcefM = frame.downEcef * (-(kRecoveryEarthRadiusM + site.altitudeM));
    return frame;
}

Vec3 ecefVectorToNed(const Vec3& ecef, const NedFrame& frame) {
    return {ecef.dot(frame.northEcef), ecef.dot(frame.eastEcef), ecef.dot(frame.downEcef)};
}

Vec3 nedVectorToEcef(const Vec3& ned, const NedFrame& frame) {
    return frame.northEcef * ned.x + frame.eastEcef * ned.y + frame.downEcef * ned.z;
}

Vec3 ecefPositionToNed(const Vec3& ecef, const NedFrame& frame) {
    return ecefVectorToNed(ecef - frame.originEcefM, frame);
}

Vec3 nedPositionToEcef(const Vec3& ned, const NedFrame& frame) {
    return frame.originEcefM + nedVectorToEcef(ned, frame);
}

Quaternion attitudeFromBodyX(const Vec3& direction, const Vec3& preferredY) {
    Vec3 bodyX = direction.normalized();
    if (bodyX.normSquared() < 1.0e-12) bodyX = {1.0, 0.0, 0.0};
    Vec3 bodyY = (preferredY - bodyX * preferredY.dot(bodyX)).normalized();
    if (bodyY.normSquared() < 1.0e-12) bodyY = Vec3{0.0, 0.0, 1.0}.cross(bodyX).normalized();
    if (bodyY.normSquared() < 1.0e-12) bodyY = {0.0, 1.0, 0.0};
    const Vec3 bodyZ = bodyX.cross(bodyY).normalized();
    Mat3 rotation{};
    rotation.m[0][0] = bodyX.x; rotation.m[1][0] = bodyX.y; rotation.m[2][0] = bodyX.z;
    rotation.m[0][1] = bodyY.x; rotation.m[1][1] = bodyY.y; rotation.m[2][1] = bodyY.z;
    rotation.m[0][2] = bodyZ.x; rotation.m[1][2] = bodyZ.y; rotation.m[2][2] = bodyZ.z;
    return Quaternion::fromRotationMatrix(rotation);
}

} // namespace gnc
