// =============================================================================
//  render/gl/Mesh.hpp -- an indexed triangle mesh with interleaved position +
//  normal vertex data (a VAO/VBO/EBO triple). Vertices are in GL space.
// =============================================================================
#pragma once

#include "render/gl/GlCommon.hpp"

#include <cstdint>
#include <vector>

namespace fsim::gl {

struct Vertex {
    float px, py, pz;   // position (GL space)
    float nx, ny, nz;   // normal
};

class Mesh {
public:
    Mesh() = default;
    ~Mesh() { destroy(); }

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    void upload(const std::vector<Vertex>& vertices,
                const std::vector<uint32_t>& indices);

    void draw() const {
        glBindVertexArray(vao_);
        glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
        glBindVertexArray(0);
    }

private:
    GLuint vao_{0}, vbo_{0}, ebo_{0};
    GLsizei indexCount_{0};

    void destroy() {
        if (ebo_) glDeleteBuffers(1, &ebo_);
        if (vbo_) glDeleteBuffers(1, &vbo_);
        if (vao_) glDeleteVertexArrays(1, &vao_);
        vao_ = vbo_ = ebo_ = 0;
    }
};

inline void Mesh::upload(const std::vector<Vertex>& vertices,
                         const std::vector<uint32_t>& indices) {
    destroy();
    indexCount_ = static_cast<GLsizei>(indices.size());

    glGenVertexArrays(1, &vao_);
    glBindVertexArray(vao_);

    glGenBuffers(1, &vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &ebo_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(indices.size() * sizeof(uint32_t)),
                 indices.data(), GL_STATIC_DRAW);

    // layout(location=0) position, layout(location=1) normal.
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

} // namespace fsim::gl
