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
    virtual void updateTransformAndAABB() = 0;

    // Axis-aligned bounding box collision detection
    virtual bool checkAABBCollision(const Collider* other) const = 0;
    
    // Public member variables
    glm::dvec3 m_position;
    glm::dquat m_orientation;
    ColliderReference* m_reference;

    // AABB data
    glm::dvec3 m_AABBMin;
    glm::dvec3 m_AABBMax;
};