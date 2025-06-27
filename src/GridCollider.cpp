// GridCollider.cpp
#include "GridCollider.h"
#include "BallCollider.h"
#include "CollisionDetectionUtils.h"
#include <glm/gtx/transform.hpp>

GridCollider::GridCollider(const glm::dvec3& position,
                          const glm::dquat& orientation,
                          ColliderReference* reference)
    : Collider(position, orientation, reference)
{
    
}

CollisionResult GridCollider::collideWith(Collider* other) {
    // Double dispatch: call collideWithGrid on other
    return other->collideWithGrid(this);
}

void GridCollider::updateAABB() {
    if (m_cells.empty()) {
        // If no cells, set a minimal AABB
        m_AABBMin = m_position - glm::dvec3(0.1);
        m_AABBMax = m_position + glm::dvec3(0.1);
        return;
    }
    
    // Update all sub-collider transforms and AABBs
    updateSubColliderTransformsAndAABB();
    
    // Initialize bounds with first collider
    auto it = m_cells.begin();
    glm::dvec3 minBounds = it->second->m_AABBMin;
    glm::dvec3 maxBounds = it->second->m_AABBMax;
    
    // Find overall bounds from all sub-colliders
    for (const auto& pair : m_cells) {
        const auto& cubeCollider = pair.second;
        minBounds = glm::min(minBounds, cubeCollider->m_AABBMin);
        maxBounds = glm::max(maxBounds, cubeCollider->m_AABBMax);
    }
    
    m_AABBMin = minBounds;
    m_AABBMax = maxBounds;
}

CollisionResult GridCollider::collideWithBall(BallCollider* ball) {
    return CollisionDetectionUtils::detectBallGrid(
        ball->m_position, ball->m_radius, this, ball, this);
}

CollisionResult GridCollider::collideWithCube(CubeCollider* cube) {
    return CollisionDetectionUtils::detectCubeGrid(
        cube->m_position, cube->m_orientation, cube->m_width, this, cube, this);
}

CollisionResult GridCollider::collideWithGrid(GridCollider* other) {
    return CollisionDetectionUtils::detectGridGrid(this, other, this, other);
}

bool GridCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

void GridCollider::addCell(const glm::ivec3& coord, double width) {
    // Create a new cube collider for this cell
    glm::dvec3 worldPos = gridToWorld(glm::dvec3(coord));
    auto cubeCollider = std::make_unique<CubeCollider>(worldPos, m_orientation, width, m_reference);
    
    m_cells[coord] = std::move(cubeCollider);
    
    // Update AABBs after adding
    updateAABB();
}

void GridCollider::removeCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        m_cells.erase(it);
        // Update AABBs after removing
        updateAABB();
    }
}

bool GridCollider::hasCell(const glm::ivec3& coord) const {
    return m_cells.find(coord) != m_cells.end();
}

CubeCollider* GridCollider::getCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        return it->second.get();
    }
    return nullptr;
}

void GridCollider::updateSubColliderTransformsAndAABB() {
    // Update position and orientation of all sub-colliders
    for (const auto& pair : m_cells) {
        const glm::ivec3& coord = pair.first;
        CubeCollider* subCollider = pair.second.get();
        
        // Calculate world position for this grid cell center
        glm::dvec3 cellCenter = glm::dvec3(coord) + glm::dvec3(0.5);
        subCollider->m_position = gridToWorld(cellCenter);
        subCollider->m_orientation = m_orientation;
        
        // Update the sub-collider's AABB
        subCollider->updateAABB();
    }
}

glm::dvec3 GridCollider::gridToWorld(const glm::dvec3& gridCoord) const {
    // Convert grid coordinates to world coordinates
    glm::dvec3 localPos = gridCoord;
    
    // Apply orientation and position transform
    return m_position + m_orientation * localPos;
}