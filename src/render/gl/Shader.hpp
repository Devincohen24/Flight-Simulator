// =============================================================================
//  render/gl/Shader.hpp -- compile/link a GLSL program and set uniforms.
// =============================================================================
#pragma once

#include "render/gl/GlCommon.hpp"
#include "core/Mat4.hpp"
#include "core/Math.hpp"

#include <string>

namespace fsim::gl {

class Shader {
public:
    Shader() = default;
    ~Shader() { if (program_) glDeleteProgram(program_); }

    Shader(const Shader&) = delete;
    Shader& operator=(const Shader&) = delete;

    // Compile and link from GLSL source strings. Returns false (and logs) on
    // error.
    bool build(const char* vertexSrc, const char* fragmentSrc);

    void use() const { glUseProgram(program_); }
    GLuint id() const { return program_; }

    void setMat4(const char* name, const Mat4& m) const {
        float f[16];
        toFloat16(m.m, f);
        glUniformMatrix4fv(loc(name), 1, GL_FALSE, f);
    }
    void setVec3(const char* name, const Vec3& v) const {
        glUniform3f(loc(name), float(v.x), float(v.y), float(v.z));
    }
    void setFloat(const char* name, float v) const { glUniform1f(loc(name), v); }

private:
    GLuint program_{0};
    GLint loc(const char* name) const { return glGetUniformLocation(program_, name); }
    static GLuint compile(GLenum type, const char* src);
};

} // namespace fsim::gl
