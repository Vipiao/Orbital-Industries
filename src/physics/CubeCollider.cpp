// CubeCollider.cpp
#include "CubeCollider.h"
#include "../utils/PolyhedronProcessor.h"

CubeCollider::CubeCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double width)
    : PolyhedronCollider(position, orientation, 
                         PolyhedronProcessor::generateCubeVertices(width),
                         PolyhedronProcessor::generateCubeAxes(),
                         PolyhedronProcessor::generateCubeAxes()) // For cubes, face axes and edge axes are the same
    , m_width(width)
{
}

void CubeCollider::updateCachedCollisionAxes() const {
    // For cubes, face axes and edge axes are identical (X, Y, Z)
    // So we can calculate once and reuse, saving 3 matrix-vector multiplications
    
    std::vector<glm::dvec3> worldAxes;
    worldAxes.reserve(m_localFaceAxes.size());
    for (const glm::dvec3& faceAxis : m_localFaceAxes) {
        worldAxes.push_back(m_orientation * faceAxis);
    }
    
    // Transform filter normals to world space
    std::vector<glm::dvec3> worldFilterNormals;
    worldFilterNormals.reserve(m_filterNormals.size());
    for (const glm::dvec3& filterNormal : m_filterNormals) {
        worldFilterNormals.push_back(m_orientation * filterNormal);
    }
    
    m_cachedCollisionAxes = {worldAxes, worldAxes, worldFilterNormals}; // Reuse face axes for edge axes
}