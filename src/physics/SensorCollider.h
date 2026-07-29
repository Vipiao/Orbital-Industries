// SensorCollider.h
#pragma once

#include "Collider.h"
#include "utils/HashFunctions.h"
#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Sensor collider for spatial queries without collision resolution
 * 
 * SensorColliders participate in broad-phase collision detection (AABB overlap tracking)
 * but never generate collision results. Use them for trigger volumes, area queries,
 * or any spatial query that doesn't need physics response.
 * 
 * Overlapping colliders are automatically tracked and can be queried in O(1).
 */
class SensorCollider : public Collider {
public:
    SensorCollider(const glm::dvec3& position,
                   const glm::dvec3& halfScale);
    
    virtual ~SensorCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateSimpleAABB(uint64_t currentTimestep) override;
    virtual void updateAdvancedAABB(uint64_t currentTimestep) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    virtual RayIntersectionResult intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const override;
    
    // Query overlapping colliders (O(1) - returns cached overlaps)
    std::vector<Collider*> getOverlappingColliders() const {
        return std::vector<Collider*>(m_overlappingColliders.begin(), 
                                       m_overlappingColliders.end());
    }
    
    // Access half-scale
    const glm::dvec3& getHalfScale() const { return m_halfScale; }
    void setHalfScale(const glm::dvec3& halfScale) { m_halfScale = halfScale; }
    
    // Type identification
    static constexpr int TYPE_ID = Hash::hashName("SensorCollider");

private:
    glm::dvec3 m_halfScale; // Half-widths in each axis (x, y, z)
};