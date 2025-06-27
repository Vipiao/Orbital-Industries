// CubeCollider.cpp
#include "CubeCollider.h"
#include "BallCollider.h"
#include "GridCollider.h"
#include "CollisionDetectionUtils.h"
#include <glm/gtx/transform.hpp>

CubeCollider::CubeCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double width,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_width(width)
{
    
}

CollisionResult CubeCollider::collideWith(Collider* other) {
    // Double dispatch: call collideWithCube on other
    return other->collideWithCube(this);
}

void CubeCollider::updateAABB() {
    // Calculate AABB by finding min/max of all vertices
    std::vector<glm::dvec3> vertices = getVertices();
    
    if (vertices.empty()) {
        m_AABBMin = m_position - glm::dvec3(m_width * 0.5);
        m_AABBMax = m_position + glm::dvec3(m_width * 0.5);
        return;
    }
    
    m_AABBMin = vertices[0];
    m_AABBMax = vertices[0];
    
    for (const glm::dvec3& vertex : vertices) {
        m_AABBMin = glm::min(m_AABBMin, vertex);
        m_AABBMax = glm::max(m_AABBMax, vertex);
    }
}

CollisionResult CubeCollider::collideWithBall(BallCollider* ball) {
    // Use utility function for ball-cube collision (swap order for cube-ball)
    CollisionResult result = CollisionDetectionUtils::detectBallCube(
        ball->m_position, ball->m_radius,
        m_position, m_orientation, m_width,
        ball, this);
    
    // Flip normal direction since we called ball-cube instead of cube-ball
    for (glm::dvec3& normal : result.m_normals) {
        normal = -normal;
    }
    
    return result;
}

CollisionResult CubeCollider::collideWithCube(CubeCollider* other) {
    return CollisionDetectionUtils::detectCubeCube(
        m_position, m_orientation, m_width,
        other->m_position, other->m_orientation, other->m_width,
        this, other);
}

CollisionResult CubeCollider::collideWithGrid(GridCollider* grid) {
    return CollisionDetectionUtils::detectCubeGrid(
        m_position, m_orientation, m_width,
        grid, this, grid);
}

bool CubeCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

std::vector<glm::dvec3> CubeCollider::getVertices() const {
    double halfWidth = m_width * 0.5;
    std::vector<glm::dvec3> localVertices = {
        {-halfWidth, -halfWidth, -halfWidth}, { halfWidth, -halfWidth, -halfWidth},
        { halfWidth,  halfWidth, -halfWidth}, {-halfWidth,  halfWidth, -halfWidth},
        {-halfWidth, -halfWidth,  halfWidth}, { halfWidth, -halfWidth,  halfWidth},
        { halfWidth,  halfWidth,  halfWidth}, {-halfWidth,  halfWidth,  halfWidth}
    };
    
    std::vector<glm::dvec3> worldVertices;
    worldVertices.reserve(8);
    
    for (const glm::dvec3& localVertex : localVertices) {
        worldVertices.push_back(m_position + m_orientation * localVertex);
    }
    
    return worldVertices;
}