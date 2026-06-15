// =============================================================================
//  render/gl/Shader.cpp
// =============================================================================
#include "render/gl/Shader.hpp"

namespace fsim::gl {

GLuint Shader::compile(GLenum type, const char* src) {
    const GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok = GL_FALSE;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[shader] %s compile error:\n%s\n",
                     type == GL_VERTEX_SHADER ? "vertex" : "fragment", log);
        glDeleteShader(s);
        return 0;
    }
    return s;
}

bool Shader::build(const char* vertexSrc, const char* fragmentSrc) {
    const GLuint vs = compile(GL_VERTEX_SHADER, vertexSrc);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, fragmentSrc);
    if (!vs || !fs) return false;

    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);

    GLint ok = GL_FALSE;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[1024];
        glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
        std::fprintf(stderr, "[shader] link error:\n%s\n", log);
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    return ok == GL_TRUE;
}

} // namespace fsim::gl
