// GridCollider.cpp
#include "GridCollider.h"
#include <glm/gtx/transform.hpp>
#include <algorithm>

GridCollider::GridCollider(const glm::dvec3& position,
                          const glm::dquat& orientation,
                          ColliderReference* reference)
    : Collider(position, orientation, reference)
{
    
}

void GridCollider::updateSimpleAABB() {
    if (m_cells.empty()) {
        // If no cells, set a minimal AABB
        m_AABBMin = m_position - glm::dvec3(0.1);
        m_AABBMax = m_position + glm::dvec3(0.1);
        return;
    }

    // Update cached corners if needed
    if (m_cornersDirty) {
        updateLocalCorners();
        m_cornersDirty = false;
    }
    
    // Transform first corner to initialize world AABB
    glm::dvec3 worldCorner = m_position + m_orientation * m_localCorners[0];
    m_AABBMin = worldCorner;
    m_AABBMax = worldCorner;
    
    // Transform remaining corners and expand world AABB
    for (int i = 1; i < 8; ++i) {
        worldCorner = m_position + m_orientation * m_localCorners[i];
        m_AABBMin = glm::min(m_AABBMin, worldCorner);
        m_AABBMax = glm::max(m_AABBMax, worldCorner);
    }
}

void GridCollider::updateAdvancedAABB() {
    if (m_cells.empty()) {
        // If no cells, set a minimal AABB
        m_AABBMin = m_position - glm::dvec3(0.1);
        m_AABBMax = m_position + glm::dvec3(0.1);
        return;
    }
    
    // Update all sub-colliders first
    updateSubColliderTransformsAndAABB();
    
    // Calculate precise AABB from all sub-colliders
    auto it = m_cells.begin();
    m_AABBMin = it->second->m_AABBMin;
    m_AABBMax = it->second->m_AABBMax;
    
    for (const auto& pair : m_cells) {
        const auto& cubeCollider = pair.second;
        m_AABBMin = glm::min(m_AABBMin, cubeCollider->m_AABBMin);
        m_AABBMax = glm::max(m_AABBMax, cubeCollider->m_AABBMax);
    }
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

    // Update axis counts
    m_xAxisCounts[coord.x]++;
    m_yAxisCounts[coord.y]++;
    m_zAxisCounts[coord.z]++;
    
    // Update local AABB
    if (m_cells.size() == 1) { // This is the first cell
        // First cell - initialize AABB
        m_localAABBMin = coord;
        m_localAABBMax = coord;
        m_cornersDirty = true;
    } else {
        // Expand AABB to include new cell
        glm::ivec3 newMin = glm::min(m_localAABBMin, coord);
        glm::ivec3 newMax = glm::max(m_localAABBMax, coord);
        
        if (newMin != m_localAABBMin || newMax != m_localAABBMax) {
            m_localAABBMin = newMin;
            m_localAABBMax = newMax;
            m_cornersDirty = true;
        }
    }

    updateFilterNormalsForCell(coord);
}

void GridCollider::removeCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        // Check if this cell is on the border before removing
        bool onBorder = (coord.x == m_localAABBMin.x || coord.x == m_localAABBMax.x ||
                        coord.y == m_localAABBMin.y || coord.y == m_localAABBMax.y ||
                        coord.z == m_localAABBMin.z || coord.z == m_localAABBMax.z);
        
        // Update axis counts
        m_xAxisCounts[coord.x]--;
        if (m_xAxisCounts[coord.x] == 0) {
            m_xAxisCounts.erase(coord.x);
        }
        m_yAxisCounts[coord.y]--;
        if (m_yAxisCounts[coord.y] == 0) {
            m_yAxisCounts.erase(coord.y);
        }
        m_zAxisCounts[coord.z]--;
        if (m_zAxisCounts[coord.z] == 0) {
            m_zAxisCounts.erase(coord.z);
        }
        
        // Only recalculate AABB if the removed cell was on the border
        if (onBorder && !m_xAxisCounts.empty()) {
            auto xRange = std::minmax_element(m_xAxisCounts.begin(), m_xAxisCounts.end());
            auto yRange = std::minmax_element(m_yAxisCounts.begin(), m_yAxisCounts.end());
            auto zRange = std::minmax_element(m_zAxisCounts.begin(), m_zAxisCounts.end());
            m_localAABBMin = glm::ivec3(xRange.first->first, yRange.first->first, zRange.first->first);
            m_localAABBMax = glm::ivec3(xRange.second->first, yRange.second->first, zRange.second->first);
            m_cornersDirty = true;
        }

        m_cells.erase(it);
        updateFilterNormalsAfterRemoval(coord);
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
        subCollider->updateSimpleAABB();
    }
}

glm::dvec3 GridCollider::gridToWorld(const glm::dvec3& gridCoord) const {
    // Convert grid coordinates to world coordinates
    glm::dvec3 localPos = gridCoord;
    
    // Apply orientation and position transform
    return m_position + m_orientation * localPos;
}
glm::dvec3 GridCollider::worldToGrid(const glm::dvec3& worldCoord) const {
    // Inverse transformation: rotate by conjugate orientation, then translate
    return glm::conjugate(m_orientation) * (worldCoord - m_position);
}

void GridCollider::updateFilterNormalsForCell(const glm::ivec3& coord) {
    // Standard grid directions (6-connectivity)
    static const glm::dvec3 directions[6] = {
        { 1.0,  0.0,  0.0},  // +X
        {-1.0,  0.0,  0.0},  // -X
        { 0.0,  1.0,  0.0},  // +Y
        { 0.0, -1.0,  0.0},  // -Y
        { 0.0,  0.0,  1.0},  // +Z
        { 0.0,  0.0, -1.0}   // -Z
    };
    
    CubeCollider* currentCube = getCell(coord);
    if (!currentCube) return;
    
    // Clear existing filter normals for this cube
    currentCube->clearFilterNormals();
    
    // Check each direction for neighbors
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + glm::ivec3(directions[i]);
        CubeCollider* neighborCube = getCell(neighborCoord);
        
        if (neighborCube) {
            // Add filter normal pointing toward neighbor
            currentCube->addFilterNormal(directions[i]);
            
            // Add filter normal to neighbor pointing back toward current cube
            neighborCube->addFilterNormal(-directions[i]);
        }
    }
}

void GridCollider::updateFilterNormalsAfterRemoval(const glm::ivec3& removedCoord) {
    // Standard grid directions (6-connectivity)
    static const glm::dvec3 directions[6] = {
        { 1.0,  0.0,  0.0},  // +X
        {-1.0,  0.0,  0.0},  // -X
        { 0.0,  1.0,  0.0},  // +Y
        { 0.0, -1.0,  0.0},  // -Y
        { 0.0,  0.0,  1.0},  // +Z
        { 0.0,  0.0, -1.0}   // -Z
    };
    
    // Update neighbors of the removed cell
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = removedCoord + glm::ivec3(directions[i]);
        CubeCollider* neighborCube = getCell(neighborCoord);
        
        if (neighborCube) {
            // Remove filter normal pointing toward the removed cube
            neighborCube->removeFilterNormal(-directions[i]);
        }
    }
}

void GridCollider::updateLocalCorners() {
    // Expand local AABB by half diagonal of cube to encompass entire cubes
    const double halfDiagonal = std::sqrt(3.0) * 0.5;
    glm::dvec3 expandedMin = glm::dvec3(m_localAABBMin) - glm::dvec3(halfDiagonal);
    glm::dvec3 expandedMax = glm::dvec3(m_localAABBMax) + glm::dvec3(halfDiagonal);
    
    m_localCorners[0] = glm::dvec3(expandedMin.x, expandedMin.y, expandedMin.z);
    m_localCorners[1] = glm::dvec3(expandedMax.x, expandedMin.y, expandedMin.z);
    m_localCorners[2] = glm::dvec3(expandedMin.x, expandedMax.y, expandedMin.z);
    m_localCorners[3] = glm::dvec3(expandedMax.x, expandedMax.y, expandedMin.z);
    m_localCorners[4] = glm::dvec3(expandedMin.x, expandedMin.y, expandedMax.z);
    m_localCorners[5] = glm::dvec3(expandedMax.x, expandedMin.y, expandedMax.z);
    m_localCorners[6] = glm::dvec3(expandedMin.x, expandedMax.y, expandedMax.z);
    m_localCorners[7] = glm::dvec3(expandedMax.x, expandedMax.y, expandedMax.z);
}