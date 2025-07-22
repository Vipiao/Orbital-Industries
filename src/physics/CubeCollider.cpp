// CubeCollider.cpp
#include "CubeCollider.h"

CubeCollider::CubeCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double width,
                           ColliderReference* reference)
    : PolyhedronCollider(position, orientation, 
                         generateCubeVertices(width),
                         generateCubeAxes(),
                         generateCubeAxes(), // For cubes, face axes and edge axes are the same
                         reference)
    , m_width(width)
{
}

std::vector<glm::dvec3> CubeCollider::generateCubeVertices(double width) {
    double halfWidth = width * 0.5;
    
    return {
        {-halfWidth, -halfWidth, -halfWidth}, { halfWidth, -halfWidth, -halfWidth},
        { halfWidth,  halfWidth, -halfWidth}, {-halfWidth,  halfWidth, -halfWidth},
        {-halfWidth, -halfWidth,  halfWidth}, { halfWidth, -halfWidth,  halfWidth},
        { halfWidth,  halfWidth,  halfWidth}, {-halfWidth,  halfWidth,  halfWidth}
    };
}

std::vector<glm::dvec3> CubeCollider::generateCubeAxes() {
    return {
        glm::dvec3(1.0, 0.0, 0.0),  // X-axis
        glm::dvec3(0.0, 1.0, 0.0),  // Y-axis
        glm::dvec3(0.0, 0.0, 1.0)   // Z-axis
    };
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