#include "gnc/math.hpp"

#include <stdexcept>

namespace gnc {

Mat3 Mat3::identity() {
    Mat3 result{};
    result.m[0][0] = 1.0;
    result.m[1][1] = 1.0;
    result.m[2][2] = 1.0;
    return result;
}

Mat3 Mat3::diagonal(const Vec3& diagonalValue) {
    Mat3 result{};
    result.m[0][0] = diagonalValue.x;
    result.m[1][1] = diagonalValue.y;
    result.m[2][2] = diagonalValue.z;
    return result;
}

Mat3 Mat3::transposed() const {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.m[row][column] = m[column][row];
        }
    }
    return result;
}

double Mat3::determinant() const {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
         - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
         + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

Mat3 Mat3::inverse() const {
    const double determinantValue = determinant();
    if (std::abs(determinantValue) < 1.0e-15) {
        throw std::runtime_error("Singular 3x3 matrix");
    }

    Mat3 result{};
    result.m[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) / determinantValue;
    result.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) / determinantValue;
    result.m[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) / determinantValue;
    result.m[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) / determinantValue;
    result.m[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) / determinantValue;
    result.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) / determinantValue;
    result.m[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) / determinantValue;
    result.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) / determinantValue;
    result.m[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) / determinantValue;
    return result;
}

Vec3 Mat3::operator*(const Vec3& value) const {
    return {
        m[0][0] * value.x + m[0][1] * value.y + m[0][2] * value.z,
        m[1][0] * value.x + m[1][1] * value.y + m[1][2] * value.z,
        m[2][0] * value.x + m[2][1] * value.y + m[2][2] * value.z
    };
}

Mat3 Mat3::operator*(const Mat3& rhs) const {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            for (std::size_t inner = 0; inner < 3; ++inner) {
                result.m[row][column] += m[row][inner] * rhs.m[inner][column];
            }
        }
    }
    return result;
}

Mat3 Mat3::operator*(double scalar) const {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.m[row][column] = m[row][column] * scalar;
        }
    }
    return result;
}

Mat3 Mat3::operator+(const Mat3& rhs) const {
    Mat3 result{};
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 3; ++column) {
            result.m[row][column] = m[row][column] + rhs.m[row][column];
        }
    }
    return result;
}

Quaternion Quaternion::fromAxisAngle(const Vec3& axis, double angleRadians) {
    const Vec3 unitAxis = axis.normalized();
    const double halfAngle = 0.5 * angleRadians;
    const double sine = std::sin(halfAngle);
    return {std::cos(halfAngle), unitAxis.x * sine, unitAxis.y * sine, unitAxis.z * sine};
}

Quaternion Quaternion::fromEulerZYX(double rollRadians, double pitchRadians, double yawRadians) {
    const Quaternion roll = fromAxisAngle({1.0, 0.0, 0.0}, rollRadians);
    const Quaternion pitch = fromAxisAngle({0.0, 1.0, 0.0}, pitchRadians);
    const Quaternion yaw = fromAxisAngle({0.0, 0.0, 1.0}, yawRadians);
    return (yaw * pitch * roll).normalized();
}

Quaternion Quaternion::fromRotationMatrix(const Mat3& matrix) {
    Quaternion result{};
    const double trace = matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2];
    if (trace > 0.0) {
        const double scale = std::sqrt(trace + 1.0) * 2.0;
        result.w = 0.25 * scale;
        result.x = (matrix.m[2][1] - matrix.m[1][2]) / scale;
        result.y = (matrix.m[0][2] - matrix.m[2][0]) / scale;
        result.z = (matrix.m[1][0] - matrix.m[0][1]) / scale;
    } else if (matrix.m[0][0] > matrix.m[1][1] && matrix.m[0][0] > matrix.m[2][2]) {
        const double scale = std::sqrt(1.0 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2]) * 2.0;
        result.w = (matrix.m[2][1] - matrix.m[1][2]) / scale;
        result.x = 0.25 * scale;
        result.y = (matrix.m[0][1] + matrix.m[1][0]) / scale;
        result.z = (matrix.m[0][2] + matrix.m[2][0]) / scale;
    } else if (matrix.m[1][1] > matrix.m[2][2]) {
        const double scale = std::sqrt(1.0 + matrix.m[1][1] - matrix.m[0][0] - matrix.m[2][2]) * 2.0;
        result.w = (matrix.m[0][2] - matrix.m[2][0]) / scale;
        result.x = (matrix.m[0][1] + matrix.m[1][0]) / scale;
        result.y = 0.25 * scale;
        result.z = (matrix.m[1][2] + matrix.m[2][1]) / scale;
    } else {
        const double scale = std::sqrt(1.0 + matrix.m[2][2] - matrix.m[0][0] - matrix.m[1][1]) * 2.0;
        result.w = (matrix.m[1][0] - matrix.m[0][1]) / scale;
        result.x = (matrix.m[0][2] + matrix.m[2][0]) / scale;
        result.y = (matrix.m[1][2] + matrix.m[2][1]) / scale;
        result.z = 0.25 * scale;
    }
    return result.normalized();
}

Quaternion Quaternion::normalized() const {
    const double length = norm();
    return length > 1.0e-15 ? Quaternion{w / length, x / length, y / length, z / length}
                            : Quaternion::identity();
}

Quaternion Quaternion::inverse() const {
    const double magnitudeSquared = normSquared();
    if (magnitudeSquared < 1.0e-15) {
        return Quaternion::identity();
    }
    const Quaternion conjugated = conjugate();
    return {conjugated.w / magnitudeSquared, conjugated.x / magnitudeSquared,
            conjugated.y / magnitudeSquared, conjugated.z / magnitudeSquared};
}

Quaternion Quaternion::operator*(const Quaternion& rhs) const {
    return {
        w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
        w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
        w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
        w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w
    };
}

Vec3 Quaternion::rotate(const Vec3& bodyVector) const {
    const Quaternion unit = normalized();
    const Quaternion pure{0.0, bodyVector.x, bodyVector.y, bodyVector.z};
    const Quaternion rotated = unit * pure * unit.conjugate();
    return rotated.vector();
}

Mat3 Quaternion::toRotationMatrix() const {
    const Quaternion q = normalized();
    Mat3 result{};
    result.m[0][0] = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    result.m[0][1] = 2.0 * (q.x * q.y - q.w * q.z);
    result.m[0][2] = 2.0 * (q.x * q.z + q.w * q.y);
    result.m[1][0] = 2.0 * (q.x * q.y + q.w * q.z);
    result.m[1][1] = 1.0 - 2.0 * (q.x * q.x + q.z * q.z);
    result.m[1][2] = 2.0 * (q.y * q.z - q.w * q.x);
    result.m[2][0] = 2.0 * (q.x * q.z - q.w * q.y);
    result.m[2][1] = 2.0 * (q.y * q.z + q.w * q.x);
    result.m[2][2] = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    return result;
}

Vec3 Quaternion::toEulerZYX() const {
    const Mat3 rotation = toRotationMatrix();
    const double pitch = std::asin(clamp(-rotation.m[2][0], -1.0, 1.0));
    double roll{};
    double yaw{};
    if (std::abs(std::cos(pitch)) > 1.0e-9) {
        roll = std::atan2(rotation.m[2][1], rotation.m[2][2]);
        yaw = std::atan2(rotation.m[1][0], rotation.m[0][0]);
    } else {
        roll = 0.0;
        yaw = std::atan2(-rotation.m[0][1], rotation.m[1][1]);
    }
    return {roll, pitch, yaw};
}

Vec3 rigidBodyAngularAcceleration(const Vec3& angularRateBody,
                                  const Mat3& inertiaBody,
                                  const Vec3& torqueBody,
                                  const Mat3& inertiaDerivativeBody) {
    const Vec3 angularMomentum = inertiaBody * angularRateBody;
    const Vec3 inertiaRateMomentum = inertiaDerivativeBody * angularRateBody;
    return inertiaBody.inverse() * (torqueBody - angularRateBody.cross(angularMomentum) - inertiaRateMomentum);
}

Quaternion attitudeDerivative(const Quaternion& bodyToInertial, const Vec3& angularRateBody) {
    return (bodyToInertial * Quaternion{0.0, angularRateBody.x, angularRateBody.y, angularRateBody.z}) * 0.5;
}

void integrateRotationalRK4(RotationalState& state,
                            const Mat3& inertiaBody,
                            const Vec3& torqueBody,
                            double dt,
                            const Mat3& inertiaDerivativeBody) {
    struct Derivative { Quaternion q; Vec3 omega; };
    const auto derivative = [&](const Quaternion& attitude, const Vec3& angularRate) {
        return Derivative{
            attitudeDerivative(attitude, angularRate),
            rigidBodyAngularAcceleration(angularRate, inertiaBody, torqueBody, inertiaDerivativeBody)
        };
    };

    const Derivative k1 = derivative(state.attitude, state.angularRate);
    const Derivative k2 = derivative(
        state.attitude + k1.q * (0.5 * dt), state.angularRate + k1.omega * (0.5 * dt));
    const Derivative k3 = derivative(
        state.attitude + k2.q * (0.5 * dt), state.angularRate + k2.omega * (0.5 * dt));
    const Derivative k4 = derivative(
        state.attitude + k3.q * dt, state.angularRate + k3.omega * dt);

    state.attitude = (state.attitude
        + (k1.q + k2.q * 2.0 + k3.q * 2.0 + k4.q) * (dt / 6.0)).normalized();
    state.angularRate += (k1.omega + k2.omega * 2.0 + k3.omega * 2.0 + k4.omega) * (dt / 6.0);
}

Quaternion attitudeError(const Quaternion& currentBodyToInertial,
                         const Quaternion& targetBodyToInertial) {
    return (currentBodyToInertial.inverse() * targetBodyToInertial).normalized().shortest();
}

double quaternionAngularDistance(const Quaternion& lhs, const Quaternion& rhs) {
    const Quaternion error = attitudeError(lhs, rhs);
    return 2.0 * std::atan2(error.vector().norm(), clamp(error.w, 0.0, 1.0));
}

Quaternion slerp(const Quaternion& from, const Quaternion& to, double fraction) {
    Quaternion start = from.normalized();
    Quaternion end = to.normalized();
    double cosine = start.w * end.w + start.x * end.x + start.y * end.y + start.z * end.z;
    if (cosine < 0.0) {
        end = {-end.w, -end.x, -end.y, -end.z};
        cosine = -cosine;
    }
    const double u = clamp(fraction, 0.0, 1.0);
    if (cosine > 0.9995) {
        return (start * (1.0 - u) + end * u).normalized();
    }
    const double angle = std::acos(clamp(cosine, -1.0, 1.0));
    const double sine = std::sin(angle);
    const double startWeight = std::sin((1.0 - u) * angle) / sine;
    const double endWeight = std::sin(u * angle) / sine;
    return (start * startWeight + end * endWeight).normalized();
}

} // namespace gnc

