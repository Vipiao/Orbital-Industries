// Collider.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <utility>
#include <vector>
#include "../utils/GeometryUtils.h"
#include "../utils/PointerStorage.h"

class CoordinateSystem {
public:
    glm::dvec3 m_position;
    glm::dquat m_orientation;
    
    CoordinateSystem(const glm::dvec3& position = glm::dvec3(0.0), 
                     const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0))
        : m_position(position), m_orientation(orientation) {}
    
    virtual ~CoordinateSystem() = default;
};

class Collider : public CoordinateSystem, public IPointerStorage {
public:
    // Unique ID for debugging
    const int m_debugId;

    Collider(const glm::dvec3& position = glm::dvec3(0.0), 
             const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0))
        : CoordinateSystem(position, orientation)
        , m_debugId(s_nextId++)
        , m_AABBMin(0.0)
        , m_AABBMax(0.0)
        , m_simpleAABBValidUntilTime(0)
        , m_advancedAABBValidUntilTime(0)
        , m_positionValidUntilTime(0)
    {}
    
    virtual ~Collider() = default;

    // Type identification system
    static const int TYPE_ID;
    virtual int getTypeId() const = 0;
    
    // Pure virtual methods that must be implemented by derived classes
    virtual void updateSimpleAABB(uint64_t currentTimestep) = 0;

    // Advanced AABB update for precise collision detection (default calls updateSimpleAABB)
    virtual void updateAdvancedAABB(uint64_t currentTimestep) { updateSimpleAABB(currentTimestep); }

    // Axis-aligned bounding box collision detection
    virtual bool checkAABBCollision(const Collider* other) const = 0;

    // Ray intersection (world space)
    virtual RayIntersectionResult intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const = 0;

    // Transform local point to world coordinates
    glm::dvec3 localToWorld(const glm::dvec3& localPoint) const {
        return m_position + m_orientation * localPoint;
    }
    
    // Transform world point to local coordinates
    glm::dvec3 worldToLocal(const glm::dvec3& worldPoint) const {
        return glm::conjugate(m_orientation) * (worldPoint - m_position);
    }
    
    // Filter normal management (default no-op implementations)
    virtual void addFilterNormal(const glm::dvec3& normal) {}
    virtual void removeFilterNormal(const glm::dvec3& normal) {}
    virtual void clearFilterNormals() {}
    virtual const std::vector<glm::dvec3>& getFilterNormals() const { 
        static const std::vector<glm::dvec3> empty; 
        return empty; 
    }

    // Dependent positioning system
    void updatePosition(uint64_t currentTimestep);

    // Dependent positioning
    CoordinateSystem* m_dependentPosition = nullptr;
    glm::dvec3 m_dependentOffset{0.0};
    uint64_t m_positionValidUntilTime = 0;

    // Timestep-based validity tracking
    uint64_t m_simpleAABBValidUntilTime;
    uint64_t m_advancedAABBValidUntilTime;

    // AABB data
    glm::dvec3 m_AABBMin;
    glm::dvec3 m_AABBMax;

private:
    static int s_nextId;
};