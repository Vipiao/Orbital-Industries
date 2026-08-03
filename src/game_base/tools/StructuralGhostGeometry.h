// StructuralGhostGeometry.h
#pragma once

#include <array>
#include <glm/glm.hpp>
#include <memory>

class GraphicsEngine;
class Geometry;

// Transparent preview geometry for one structural block shape.
//
// The mesh depends on the shape alone, so it survives the ghost sliding along a
// surface, turning, or moving between grids — those are per-instance transforms.
// Only a new shape rebuilds it, which is why the tool can hand its current shape
// over every frame.
class StructuralGhostGeometry {
public:
    explicit StructuralGhostGeometry(GraphicsEngine* graphics);
    ~StructuralGhostGeometry();

    // Owns a geometry handle in the graphics engine; copying would release it twice.
    StructuralGhostGeometry(const StructuralGhostGeometry&) = delete;
    StructuralGhostGeometry& operator=(const StructuralGhostGeometry&) = delete;

    // Geometry for the given shape, built on first use and whenever the shape changes.
    // Instances of the previous shape expire with it.
    std::weak_ptr<Geometry> get(const std::array<glm::ivec3, 8>& vertices);

private:
    void release();

    GraphicsEngine*           m_graphics{nullptr};
    std::array<glm::ivec3, 8> m_vertices{};
    std::weak_ptr<Geometry>   m_geometry{};
};
