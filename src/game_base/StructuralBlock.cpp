// StructuralBlock.cpp
#include "StructuralBlock.h"
#include "../utils/MassInertiaCalculator.h"

// Define static default vertices
const std::array<glm::ivec3, 8> StructuralBlock::DEFAULT_VERTICES = {{
    {0, 0, 0},           // 0: bottom-back-left
    {MAX_SIZE, 0, 0},    // 1: bottom-back-right
    {MAX_SIZE, MAX_SIZE, 0}, // 2: bottom-front-right
    {0, MAX_SIZE, 0},    // 3: bottom-front-left
    {0, 0, MAX_SIZE},    // 4: top-back-left
    {MAX_SIZE, 0, MAX_SIZE},    // 5: top-back-right
    {MAX_SIZE, MAX_SIZE, MAX_SIZE}, // 6: top-front-right
    {0, MAX_SIZE, MAX_SIZE}     // 7: top-front-left
}};

StructuralBlock::StructuralBlock(const glm::ivec3& coords)
    : GridCell(coords, TYPE) 
{
    // Copy from static default vertices
    m_localVertices = DEFAULT_VERTICES;
}

PolyhedronProcessor::AxisResult StructuralBlock::getAxes() const {
    return PolyhedronProcessor::getAxis(
        std::vector<glm::ivec3>(m_localVertices.begin(), m_localVertices.end()), 
        MAX_SIZE
    );
}

bool StructuralBlock::setVertices(const std::array<glm::ivec3, 8>& newVertices) {
    // Convert to vector for validation
    std::vector<glm::ivec3> verticesVec(newVertices.begin(), newVertices.end());
    
    // Validate the new shape
    if (!PolyhedronProcessor::validatePolyhedron(verticesVec, MAX_SIZE)) {
        return false; // Invalid shape, don't update
    }

    // Mark mass properties as dirty before updating vertices
    m_massPropertiesDirty = true;
    
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
        MAX_SIZE
    );
}

StructuralBlock::MeshData StructuralBlock::generateTriangleMeshData() const {
    // Get triangles from PolyhedronProcessor
    std::vector<glm::ivec3> verticesVec(m_localVertices.begin(), m_localVertices.end());
    auto triangles = PolyhedronProcessor::getTriangles(verticesVec, MAX_SIZE);
    
    // Generate complete mesh data using PolyhedronProcessor
    return PolyhedronProcessor::generateMeshData(triangles);
}

std::tuple<double, glm::dvec3, glm::dmat3> StructuralBlock::getMassProperties() const {
    // Return cached values if still valid
    if (!m_massPropertiesDirty) {
        return std::make_tuple(m_cachedMass, m_cachedCenterOfMass, m_cachedInertiaTensor);
    }

    // Convert vertices array to vector for MassInertiaCalculator
    std::vector<glm::ivec3> verticesVec(m_localVertices.begin(), m_localVertices.end());
    
    // Use density = 1.0 for now
    const double density = 1.0;
    
    // Calculate mass properties using the new polyhedron calculation
    auto massProps = MassInertiaCalculator::calculatePolyhedronMassProperties(verticesVec, MAX_SIZE, density);
    
    // Cache the results
    m_cachedMass = massProps.mass;
    m_cachedCenterOfMass = massProps.centerOfMass;
    m_cachedInertiaTensor = massProps.inertiaTensor;
    m_massPropertiesDirty = false;
    
    return std::make_tuple(m_cachedMass, m_cachedCenterOfMass, m_cachedInertiaTensor);
}