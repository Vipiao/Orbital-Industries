// Collider.h
#pragma once

#include "ColliderReference.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <utility>
#include <vector>

// Forward declaration
class Collider;
class BallCollider;
class GridCollider;

struct CollisionResult {
    bool m_hasCollision = false;
    std::vector<glm::dvec3> m_normals;           // Collision normals
    std::vector<glm::dvec3> m_contactPoints;    // Contact points in world space
    std::vector<double> m_penetrationDepths;    // How much objects are overlapping at each contact point

    // For collision resolution optimization
    std::vector<double> m_collisionMasses;      // Collision mass for each contact point
    bool m_collisionMassesCalculated = false;   // Flag to avoid recalculation

    // For collision resolution
    Collider* m_colliderA = nullptr;
    Collider* m_colliderB = nullptr;
    
    CollisionResult() = default;
    
    CollisionResult(bool collision, const std::vector<glm::dvec3>& norms, const std::vector<glm::dvec3>& contacts, const std::vector<double>& depths)
        : m_hasCollision(collision), m_normals(norms), m_contactPoints(contacts), m_penetrationDepths(depths) {}
    
    // Constructor with collider references
    CollisionResult(bool collision, const std::vector<glm::dvec3>& norms, const std::vector<glm::dvec3>& contacts, 
                   const std::vector<double>& depths, Collider* a, Collider* b)
        : m_hasCollision(collision), m_normals(norms), m_contactPoints(contacts), m_penetrationDepths(depths), 
          m_colliderA(a), m_colliderB(b) {}
    
    CollisionResult(bool collision, const glm::dvec3& norm, const glm::dvec3& contact, double depth = 0.0)
        : m_hasCollision(collision) {
        if (collision) {
            m_normals.push_back(norm);
            m_contactPoints.push_back(contact);
            m_penetrationDepths.push_back(depth);
        }
    }
};

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
    
    // Pure virtual methods that must be implemented by derived classes
    virtual CollisionResult collideWith(Collider* other) = 0;
    virtual void updateAABB() = 0;
    
    // Double dispatch methods - can be overridden by derived classes
    virtual CollisionResult collideWithBall(BallCollider* ball) = 0;
    virtual CollisionResult collideWithGrid(GridCollider* grid) = 0;

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