// Collider.h
#pragma once

#include "ColliderReference.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <utility>
#include <vector>

class Collider {
public:
    Collider(const glm::dvec3& position = glm::dvec3(0.0), 
             const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
             ColliderReference* reference = nullptr)
        : m_position(position)
        , m_orientation(orientation)
        , m_reference(reference)
        , m_AABBMin(0.0)
        , m_AABBMax(0.0)
    {}
    
    virtual ~Collider() = default;

    // Type identification system
    static const int TYPE_ID;
    virtual int getTypeId() const = 0;
    
    // Pure virtual methods that must be implemented by derived classes
    virtual void updateSimpleAABB() = 0;

    // Advanced AABB update for precise collision detection (default calls updateSimpleAABB)
    virtual void updateAdvancedAABB() { updateSimpleAABB(); }

    // Axis-aligned bounding box collision detection
    virtual bool checkAABBCollision(const Collider* other) const = 0;

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

    // Public member variables
    glm::dvec3 m_position;
    glm::dquat m_orientation;
    ColliderReference* m_reference;

    // AABB data
    glm::dvec3 m_AABBMin;
    glm::dvec3 m_AABBMax;
};