// BallCollider.cpp
#include "BallCollider.h"
#include "GridCollider.h"
#include "CubeCollider.h"
#include "CollisionDetectionUtils.h"
#include <glm/gtx/norm.hpp>

BallCollider::BallCollider(const glm::dvec3& position,
                           const glm::dquat& orientation,
                           double radius,
                           ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_radius(radius)
{
    
}

CollisionResult BallCollider::collideWith(Collider* other) {
    // Double dispatch: call collideWithBall on other without casting
    return other->collideWithBall(this);
}

void BallCollider::updateTransformAndAABB() {
    m_AABBMin = m_position - glm::dvec3(m_radius);
    m_AABBMax = m_position + glm::dvec3(m_radius);
}

CollisionResult BallCollider::collideWithBall(BallCollider* other) {
    return CollisionDetectionUtils::detectBallBall(
        m_position, m_radius,
        other->m_position, other->m_radius,
        this, other);
}

CollisionResult BallCollider::collideWithCube(CubeCollider* cube) {
    return CollisionDetectionUtils::detectBallCube(
        m_position, m_radius,
        cube->m_position, cube->m_orientation, cube->m_width,
        this, cube);
}

CollisionResult BallCollider::collideWithGrid(GridCollider* grid) {
    return CollisionDetectionUtils::detectBallGrid(
        m_position, m_radius, grid, this, grid);
}


bool BallCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}