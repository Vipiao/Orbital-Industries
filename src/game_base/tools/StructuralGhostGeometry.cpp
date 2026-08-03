// StructuralGhostGeometry.cpp
#include "StructuralGhostGeometry.h"

#include "../StructuralBlock.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instancedGeometry/InstancedGeometry.h"

#include <stdexcept>
#include <vector>

StructuralGhostGeometry::StructuralGhostGeometry(GraphicsEngine* graphics)
    : m_graphics{graphics} {
    if (!m_graphics) {
        throw std::runtime_error("GraphicsEngine cannot be null");
    }
}

StructuralGhostGeometry::~StructuralGhostGeometry() {
    release();
}

std::weak_ptr<Geometry> StructuralGhostGeometry::get(
    const std::array<glm::ivec3, 8>& vertices) {
    if (!m_geometry.expired() && vertices == m_vertices) {
        return m_geometry;
    }

    release();

    const PolyhedronProcessor::MeshData meshData{
        StructuralBlock::generateTriangleMeshData(vertices)};
    if (meshData.isEmpty()) {
        return {};
    }

    std::vector<GeometryVertex> geometryVertices;
    geometryVertices.reserve(meshData.positions.size());
    for (size_t ii = 0; ii < meshData.positions.size(); ii++) {
        GeometryVertex vertex{};
        vertex.position = glm::vec3{meshData.positions[ii]};
        vertex.normal   = glm::vec3{meshData.normals[ii]};
        vertex.tangent  = glm::vec3{meshData.tangents[ii]};
        vertex.uv       = glm::vec2{meshData.uvs[ii]};
        geometryVertices.push_back(vertex);
    }

    m_geometry = m_graphics->createInstanceGeometry(geometryVertices, RenderLayer::Transparent);
    m_vertices = vertices;
    return m_geometry;
}

void StructuralGhostGeometry::release() {
    if (!m_geometry.expired()) {
        m_graphics->releaseInstanceGeometry(m_geometry);
    }
    m_geometry.reset();
}
