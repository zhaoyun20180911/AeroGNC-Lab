#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace gnc {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

struct Vec3 {
    double x{};
    double y{};
    double z{};

    constexpr Vec3() = default;
    constexpr Vec3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue) {}

    constexpr Vec3 operator+(const Vec3& rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vec3 operator-(const Vec3& rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(double scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    constexpr Vec3 operator/(double scalar) const { return {x / scalar, y / scalar, z / scalar}; }
    constexpr Vec3& operator+=(const Vec3& rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    constexpr Vec3& operator-=(const Vec3& rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    constexpr Vec3& operator*=(double scalar) { x *= scalar; y *= scalar; z *= scalar; return *this; }

    [[nodiscard]] constexpr double dot(const Vec3& rhs) const { return x * rhs.x + y * rhs.y + z * rhs.z; }
    [[nodiscard]] constexpr Vec3 cross(const Vec3& rhs) const {
        return {y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x};
    }
    [[nodiscard]] double normSquared() const { return dot(*this); }
    [[nodiscard]] double norm() const { return std::sqrt(normSquared()); }
    [[nodiscard]] Vec3 normalized(double epsilon = 1.0e-12) const {
        const double length = norm();
        return length > epsilon ? *this / length : Vec3{};
    }
};

constexpr Vec3 operator*(double scalar, const Vec3& value) { return value * scalar; }

struct Mat3 {
    double m[3][3]{};

    static Mat3 identity();
    static Mat3 diagonal(const Vec3& diagonal);
    [[nodiscard]] Vec3 diagonal() const { return {m[0][0], m[1][1], m[2][2]}; }
    [[nodiscard]] Mat3 transposed() const;
    [[nodiscard]] double determinant() const;
    [[nodiscard]] Mat3 inverse() const;
    [[nodiscard]] Vec3 operator*(const Vec3& value) const;
    [[nodiscard]] Mat3 operator*(const Mat3& rhs) const;
    [[nodiscard]] Mat3 operator*(double scalar) const;
    [[nodiscard]] Mat3 operator+(const Mat3& rhs) const;
};

struct Quaternion {
    double w{1.0};
    double x{};
    double y{};
    double z{};

    constexpr Quaternion() = default;
    constexpr Quaternion(double wValue, double xValue, double yValue, double zValue)
        : w(wValue), x(xValue), y(yValue), z(zValue) {}

    static Quaternion identity() { return {}; }
    static Quaternion fromAxisAngle(const Vec3& axis, double angleRadians);
    static Quaternion fromEulerZYX(double rollRadians, double pitchRadians, double yawRadians);
    static Quaternion fromRotationMatrix(const Mat3& matrix);

    [[nodiscard]] double normSquared() const { return w * w + x * x + y * y + z * z; }
    [[nodiscard]] double norm() const { return std::sqrt(normSquared()); }
    [[nodiscard]] Quaternion normalized() const;
    [[nodiscard]] Quaternion conjugate() const { return {w, -x, -y, -z}; }
    [[nodiscard]] Quaternion inverse() const;
    [[nodiscard]] Vec3 vector() const { return {x, y, z}; }
    [[nodiscard]] Quaternion shortest() const { return w < 0.0 ? Quaternion{-w, -x, -y, -z} : *this; }
    [[nodiscard]] Quaternion operator*(const Quaternion& rhs) const;
    [[nodiscard]] Quaternion operator+(const Quaternion& rhs) const {
        return {w + rhs.w, x + rhs.x, y + rhs.y, z + rhs.z};
    }
    [[nodiscard]] Quaternion operator*(double scalar) const {
        return {w * scalar, x * scalar, y * scalar, z * scalar};
    }
    [[nodiscard]] Vec3 rotate(const Vec3& bodyVector) const;
    [[nodiscard]] Mat3 toRotationMatrix() const;
    [[nodiscard]] Vec3 toEulerZYX() const;
};

struct RotationalState {
    Quaternion attitude{}; // Active body-to-inertial rotation, q_BI.
    Vec3 angularRate{};    // Body angular rate expressed in body coordinates.
};

[[nodiscard]] Vec3 rigidBodyAngularAcceleration(
    const Vec3& angularRateBody,
    const Mat3& inertiaBody,
    const Vec3& torqueBody,
    const Mat3& inertiaDerivativeBody = {});

[[nodiscard]] Quaternion attitudeDerivative(const Quaternion& bodyToInertial, const Vec3& angularRateBody);

void integrateRotationalRK4(
    RotationalState& state,
    const Mat3& inertiaBody,
    const Vec3& torqueBody,
    double dt,
    const Mat3& inertiaDerivativeBody = {});

[[nodiscard]] Quaternion attitudeError(const Quaternion& currentBodyToInertial,
                                       const Quaternion& targetBodyToInertial);

[[nodiscard]] double quaternionAngularDistance(const Quaternion& lhs, const Quaternion& rhs);

[[nodiscard]] Quaternion slerp(const Quaternion& from, const Quaternion& to, double fraction);

} // namespace gnc
