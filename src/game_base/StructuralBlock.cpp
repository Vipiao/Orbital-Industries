// StructuralBlock.cpp
#include "StructuralBlock.h"

StructuralBlock::StructuralBlock(const glm::ivec3& coords)
    : GridCell(coords, TYPE) 
{
    // Initialize as standard cube in PolyhedronProcessor vertex order
    // Coordinates from (0,0,0) to (maxSize,maxSize,maxSize)
    m_localVertices = {{
        {0, 0, 0},           // 0: bottom-back-left
        {m_maxSize, 0, 0},   // 1: bottom-back-right
        {m_maxSize, m_maxSize, 0}, // 2: bottom-front-right
        {0, m_maxSize, 0},   // 3: bottom-front-left
        {0, 0, m_maxSize},   // 4: top-back-left
        {m_maxSize, 0, m_maxSize},   // 5: top-back-right
        {m_maxSize, m_maxSize, m_maxSize}, // 6: top-front-right
        {0, m_maxSize, m_maxSize}    // 7: top-front-left
    }};
}

PolyhedronProcessor::AxisResult StructuralBlock::getAxes() const {
    return PolyhedronProcessor::getAxis(
        std::vector<glm::ivec3>(m_localVertices.begin(), m_localVertices.end()), 
        m_maxSize
    );
}

bool StructuralBlock::setVertices(const std::array<glm::ivec3, 8>& newVertices) {
    // Convert to vector for validation
    std::vector<glm::ivec3> verticesVec(newVertices.begin(), newVertices.end());
    
    // Validate the new shape
    if (!validateVertices(newVertices, m_maxSize)) {
        return false; // Invalid shape, don't update
    }
    
    // Shape is valid, update vertices
    m_localVertices = newVertices;
    return true;
}

bool StructuralBlock::validateVertices(const std::array<glm::ivec3, 8>& vertices, int maxSize) {
    // Convert to vector for validation
    std::vector<glm::ivec3> verticesVec(vertices.begin(), vertices.end());
    return PolyhedronProcessor::validatePolyhedron(verticesVec, maxSize);
}

std::vector<glm::dvec3> StructuralBlock::getVertices() const {
    return PolyhedronProcessor::getUniqueVertices(
        std::vector<glm::ivec3>(m_localVertices.begin(), m_localVertices.end()), 
        m_maxSize
    );
}

StructuralBlock::MeshData StructuralBlock::generateTriangleMeshData() const {
    // Get triangles from PolyhedronProcessor
    std::vector<glm::ivec3> verticesVec(m_localVertices.begin(), m_localVertices.end());
    auto triangles = PolyhedronProcessor::getTriangles(verticesVec, m_maxSize);
    
    // Generate complete mesh data using PolyhedronProcessor
    return PolyhedronProcessor::generateMeshData(triangles);
}