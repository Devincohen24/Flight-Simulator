// =============================================================================
//  render/gl/GlCommon.hpp
//
//  Central include point for the OpenGL backend. This entire subtree
//  (render/gl/**) is compiled only when FSIM_BUILD_RENDERER=ON and links
//  against GLFW + an OpenGL 3.3 core loader (glad). The headless simulation
//  core never includes any of this -- the boundary is strict.
// =============================================================================
#pragma once

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <cstdio>

namespace fsim::gl {

// Convert a Mat4 (double, column-major) to the float array GL expects.
inline void toFloat16(const double* src, float* dst) {
    for (int i = 0; i < 16; ++i) dst[i] = static_cast<float>(src[i]);
}

} // namespace fsim::gl
