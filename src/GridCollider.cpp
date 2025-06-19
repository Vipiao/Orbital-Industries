// GridCollider.cpp
#include "GridCollider.h"
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
        const auto& ballCollider = pair.second;
        minBounds = glm::min(minBounds, ballCollider->m_AABBMin);
        maxBounds = glm::max(maxBounds, ballCollider->m_AABBMax);
    }
    
    m_AABBMin = minBounds;
    m_AABBMax = maxBounds;
}

CollisionResult GridCollider::collideWithBall(BallCollider* ball) {
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Test collision with each sub-collider
    for (const auto& pair : m_cells) {
        BallCollider* subCollider = pair.second.get();
        
        // Quick AABB check first
        if (!subCollider->checkAABBCollision(ball)) {
            continue;
        }
        
        // Perform detailed collision detection
        CollisionResult result = subCollider->collideWithBall(ball);
        
        if (result.m_hasCollision) {
            // Add all collision data to our result
            allNormals.insert(allNormals.end(), result.m_normals.begin(), result.m_normals.end());
            allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
            allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        return CollisionResult(true, allNormals, allContactPoints, allPenetrationDepths, this, ball);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult GridCollider::collideWithGrid(GridCollider* other) {
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Test collision between all pairs of sub-colliders
    for (const auto& pair1 : m_cells) {
        BallCollider* collider1 = pair1.second.get();
        
        for (const auto& pair2 : other->m_cells) {
            BallCollider* collider2 = pair2.second.get();
            
            // Quick AABB check first
            if (!collider1->checkAABBCollision(collider2)) {
                continue;
            }
            
            // Perform detailed collision detection
            CollisionResult result = collider1->collideWithBall(collider2);
            
            if (result.m_hasCollision) {
                // Add all collision data to our result
                allNormals.insert(allNormals.end(), result.m_normals.begin(), result.m_normals.end());
                allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
                allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
            }
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        return CollisionResult(true, allNormals, allContactPoints, allPenetrationDepths, this, other);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
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

void GridCollider::addCell(const glm::ivec3& coord, double radius) {
    // Create a new ball collider for this cell
    glm::dvec3 worldPos = gridToWorld(glm::dvec3(coord));
    auto ballCollider = std::make_unique<BallCollider>(worldPos, m_orientation, radius, m_reference);
    
    m_cells[coord] = std::move(ballCollider);
    
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

BallCollider* GridCollider::getCell(const glm::ivec3& coord) {
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
        BallCollider* subCollider = pair.second.get();
        
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