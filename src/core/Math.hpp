// =============================================================================
//  core/Math.hpp
//
//  Minimal, dependency-free linear-algebra primitives used throughout the
//  simulator: Vec3, Mat3 and a unit Quaternion. Everything is double precision
//  because the 6DOF integrator accumulates small increments over long runs and
//  is sensitive to round-off (especially quaternion drift).
//
//  Conventions (see core/Frames.hpp and docs/PHASE1_PHYSICS.md):
//    * Right-handed coordinate systems.
//    * A Quat stored here represents the rotation that takes a vector expressed
//      in the BODY frame and re-expresses it in the WORLD (NED) frame:
//          v_world = q.rotate(v_body)
//      i.e. q == q_world_from_body, with rotation matrix R = q.toMatrix().
// =============================================================================
#pragma once

#include <cmath>
#include <ostream>

namespace fsim {

// -----------------------------------------------------------------------------
// Vec3
// -----------------------------------------------------------------------------
struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};

    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    constexpr Vec3 operator-() const { return {-x, -y, -z}; }
    constexpr Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }

    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s)      { x *= s;   y *= s;   z *= s;   return *this; }

    constexpr double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    constexpr Vec3   cross(const Vec3& o) const {
        return {y * o.z - z * o.y,
                z * o.x - x * o.z,
                x * o.y - y * o.x};
    }

    double norm()        const { return std::sqrt(dot(*this)); }
    double normSquared() const { return dot(*this); }

    Vec3 normalized() const {
        const double n = norm();
        return (n > 0.0) ? (*this / n) : Vec3{0.0, 0.0, 0.0};
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

inline std::ostream& operator<<(std::ostream& os, const Vec3& v) {
    return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
}

// -----------------------------------------------------------------------------
// Mat3  (row-major 3x3; primarily used for the inertia tensor)
// -----------------------------------------------------------------------------
struct Mat3 {
    // m[row][col]
    double m[3][3]{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

    constexpr Mat3() = default;

    static constexpr Mat3 identity() {
        Mat3 r;
        r.m[0][0] = 1.0; r.m[1][1] = 1.0; r.m[2][2] = 1.0;
        return r;
    }

    // Diagonal inertia (Ixx, Iyy, Izz) with optional Ixz cross term, which is
    // the dominant product of inertia for a symmetric aircraft.
    static Mat3 inertia(double Ixx, double Iyy, double Izz, double Ixz = 0.0) {
        Mat3 r;
        r.m[0][0] =  Ixx; r.m[1][1] = Iyy; r.m[2][2] =  Izz;
        r.m[0][2] = -Ixz; r.m[2][0] = -Ixz;   // standard aircraft sign convention
        return r;
    }

    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }

    Mat3 transpose() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                r.m[i][j] = m[j][i];
        return r;
    }

    double determinant() const {
        return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    }

    // Full 3x3 inverse via cofactors. Used once to pre-compute I^-1.
    Mat3 inverse() const {
        const double det = determinant();
        Mat3 r;
        const double invDet = (det != 0.0) ? 1.0 / det : 0.0;
        r.m[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * invDet;
        r.m[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * invDet;
        r.m[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invDet;
        r.m[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * invDet;
        r.m[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invDet;
        r.m[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * invDet;
        r.m[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * invDet;
        r.m[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * invDet;
        r.m[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * invDet;
        return r;
    }
};

// -----------------------------------------------------------------------------
// Quat  (unit quaternion; Hamilton convention, [w, x, y, z])
// -----------------------------------------------------------------------------
struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};

    constexpr Quat() = default;
    constexpr Quat(double w_, double x_, double y_, double z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    static constexpr Quat identity() { return {1.0, 0.0, 0.0, 0.0}; }

    // Hamilton product (this ⊗ o).
    constexpr Quat operator*(const Quat& o) const {
        return {w * o.w - x * o.x - y * o.y - z * o.z,
                w * o.x + x * o.w + y * o.z - z * o.y,
                w * o.y - x * o.z + y * o.w + z * o.x,
                w * o.z + x * o.y - y * o.x + z * o.w};
    }

    constexpr Quat operator*(double s) const { return {w * s, x * s, y * s, z * s}; }
    constexpr Quat operator+(const Quat& o) const { return {w + o.w, x + o.x, y + o.y, z + o.z}; }

    constexpr Quat conjugate() const { return {w, -x, -y, -z}; }

    double norm() const { return std::sqrt(w * w + x * x + y * y + z * z); }

    Quat normalized() const {
        const double n = norm();
        return (n > 0.0) ? Quat{w / n, x / n, y / n, z / n} : identity();
    }

    // Rotate a vector from body frame into world frame: v_world = q v_body q*.
    Vec3 rotate(const Vec3& v) const {
        // Efficient form: t = 2 * (qv x v); v' = v + w*t + qv x t.
        const Vec3 qv{x, y, z};
        const Vec3 t = 2.0 * qv.cross(v);
        return v + w * t + qv.cross(t);
    }

    // Inverse rotation (world -> body): uses the conjugate.
    Vec3 rotateInverse(const Vec3& v) const { return conjugate().rotate(v); }

    // Rotation matrix R such that v_world = R * v_body.
    Mat3 toMatrix() const {
        Mat3 r;
        const double xx = x * x, yy = y * y, zz = z * z;
        const double xy = x * y, xz = x * z, yz = y * z;
        const double wx = w * x, wy = w * y, wz = w * z;
        r.m[0][0] = 1.0 - 2.0 * (yy + zz);
        r.m[0][1] = 2.0 * (xy - wz);
        r.m[0][2] = 2.0 * (xz + wy);
        r.m[1][0] = 2.0 * (xy + wz);
        r.m[1][1] = 1.0 - 2.0 * (xx + zz);
        r.m[1][2] = 2.0 * (yz - wx);
        r.m[2][0] = 2.0 * (xz - wy);
        r.m[2][1] = 2.0 * (yz + wx);
        r.m[2][2] = 1.0 - 2.0 * (xx + yy);
        return r;
    }

    constexpr double dot(const Quat& o) const {
        return w * o.w + x * o.x + y * o.y + z * o.z;
    }

    // Spherical linear interpolation, used to smoothly interpolate attitude
    // between two fixed-timestep physics states for variable-rate rendering.
    // t in [0,1]; the result is normalised.
    static Quat slerp(const Quat& a, Quat b, double t) {
        double d = a.dot(b);
        // Take the shorter arc.
        if (d < 0.0) { b = b * -1.0; d = -d; }
        if (d > 0.9995) {
            // Nearly parallel: fall back to normalised linear interpolation.
            return Quat{a.w + (b.w - a.w) * t,
                        a.x + (b.x - a.x) * t,
                        a.y + (b.y - a.y) * t,
                        a.z + (b.z - a.z) * t}.normalized();
        }
        const double theta0 = std::acos(d);
        const double theta  = theta0 * t;
        const double sin0   = std::sin(theta0);
        const double s0 = std::cos(theta) - d * std::sin(theta) / sin0;
        const double s1 = std::sin(theta) / sin0;
        return Quat{a.w * s0 + b.w * s1,
                    a.x * s0 + b.x * s1,
                    a.y * s0 + b.y * s1,
                    a.z * s0 + b.z * s1}.normalized();
    }

    // Build from an aerospace 3-2-1 Euler sequence (yaw psi, pitch theta,
    // roll phi), all in radians. Produces q_world_from_body.
    static Quat fromEuler(double roll, double pitch, double yaw) {
        const double cr = std::cos(roll * 0.5),  sr = std::sin(roll * 0.5);
        const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
        const double cy = std::cos(yaw * 0.5),   sy = std::sin(yaw * 0.5);
        return {cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }
};

// Convenience: Euler angles (roll phi, pitch theta, yaw psi) in radians from a
// q_world_from_body quaternion, using the standard aerospace 3-2-1 extraction.
struct EulerAngles { double roll{0}, pitch{0}, yaw{0}; };

inline EulerAngles toEuler(const Quat& q) {
    EulerAngles e;
    // roll (x-axis rotation)
    const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    e.roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation), clamped to avoid NaN at the poles
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    e.pitch = std::asin(sinp);
    // yaw (z-axis rotation)
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    e.yaw = std::atan2(siny_cosp, cosy_cosp);
    return e;
}

constexpr double kPi      = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;

inline constexpr double radians(double deg) { return deg * kDegToRad; }
inline constexpr double degrees(double rad) { return rad * kRadToDeg; }

} // namespace fsim
