// =============================================================================
//  core/Mat4.hpp
//
//  4x4 matrices for the rendering pipeline (view & projection transforms),
//  stored COLUMN-MAJOR to match OpenGL so they can be uploaded to GLSL uniforms
//  without transposing. This lives in the headless core (not the GL renderer)
//  so the camera/projection math can be unit-tested with no GPU.
//
//  Element (row r, col c) is stored at index c*4 + r.
// =============================================================================
#pragma once

#include "core/Math.hpp"

#include <cmath>

namespace fsim {

struct Mat4 {
    double m[16]{}; // column-major; default zero

    static Mat4 identity() {
        Mat4 r;
        r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0;
        return r;
    }

    double  operator()(int row, int col) const { return m[col * 4 + row]; }
    double& operator()(int row, int col)       { return m[col * 4 + row]; }

    // Matrix product (this * b), column-major.
    Mat4 operator*(const Mat4& b) const {
        Mat4 r; // zero-initialised
        for (int c = 0; c < 4; ++c)
            for (int row = 0; row < 4; ++row) {
                double s = 0.0;
                for (int k = 0; k < 4; ++k)
                    s += m[k * 4 + row] * b.m[c * 4 + k];
                r.m[c * 4 + row] = s;
            }
        return r;
    }

    // Transform a point (implicit w=1) with perspective divide.
    Vec3 transformPoint(const Vec3& v) const {
        const double x = m[0]*v.x + m[4]*v.y + m[8]*v.z + m[12];
        const double y = m[1]*v.x + m[5]*v.y + m[9]*v.z + m[13];
        const double z = m[2]*v.x + m[6]*v.y + m[10]*v.z + m[14];
        const double w = m[3]*v.x + m[7]*v.y + m[11]*v.z + m[15];
        const double iw = (w != 0.0) ? 1.0 / w : 1.0;
        return {x * iw, y * iw, z * iw};
    }

    // Transform a direction (w=0), no translation, no divide.
    Vec3 transformDir(const Vec3& v) const {
        return {m[0]*v.x + m[4]*v.y + m[8]*v.z,
                m[1]*v.x + m[5]*v.y + m[9]*v.z,
                m[2]*v.x + m[6]*v.y + m[10]*v.z};
    }
};

// Right-handed perspective projection mapping z to [-1, 1] (OpenGL clip space).
inline Mat4 perspective(double fovyRadians, double aspect, double zNear, double zFar) {
    const double f = 1.0 / std::tan(fovyRadians * 0.5);
    Mat4 r; // zero
    r.m[0]  = f / aspect;
    r.m[5]  = f;
    r.m[10] = (zFar + zNear) / (zNear - zFar);
    r.m[11] = -1.0;
    r.m[14] = (2.0 * zFar * zNear) / (zNear - zFar);
    return r;
}

// Right-handed look-at view matrix (GL convention: camera looks down -Z).
inline Mat4 lookAt(const Vec3& eye, const Vec3& center, const Vec3& up) {
    const Vec3 f = (center - eye).normalized();
    const Vec3 s = f.cross(up).normalized();
    const Vec3 u = s.cross(f);
    Mat4 r = Mat4::identity();
    r.m[0] = s.x; r.m[4] = s.y; r.m[8]  = s.z;
    r.m[1] = u.x; r.m[5] = u.y; r.m[9]  = u.z;
    r.m[2] = -f.x; r.m[6] = -f.y; r.m[10] = -f.z;
    r.m[12] = -s.dot(eye);
    r.m[13] = -u.dot(eye);
    r.m[14] =  f.dot(eye);
    return r;
}

} // namespace fsim
