// =============================================================================
//  render/gl/MeshBuilders.hpp
//
//  Geometry generation. The terrain mesh is sampled from the SAME HeightField
//  the landing gear queries, so the drawn ground and the collision ground are
//  identical. The aircraft mesh is a simple built-in airplane silhouette in the
//  body frame (replaceable later by a loaded OBJ/glTF model).
// =============================================================================
#pragma once

#include "render/gl/Mesh.hpp"
#include "terrain/Terrain.hpp"

namespace fsim::gl {

// Build a square terrain patch of side 'extent' metres, 'segments' cells per
// side, centred (snapped to the grid) on (focusNorth, focusEast). Vertices are
// emitted in GL space (positions and normals already converted from NED).
void buildTerrainMesh(const HeightField& field,
                      double focusNorth, double focusEast,
                      double extent, int segments,
                      std::vector<Vertex>& outVerts,
                      std::vector<uint32_t>& outIndices);

// Build a simple aircraft mesh (fuselage + wings + tailplane + fin) in the BODY
// frame (+x forward, +y right, +z down). The renderer applies the model matrix.
void buildAircraftMesh(std::vector<Vertex>& outVerts,
                       std::vector<uint32_t>& outIndices);

} // namespace fsim::gl
