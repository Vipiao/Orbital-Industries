// CollisionResult.h
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <stdexcept>

// Forward declaration for type-safe collider references
class Collider;

struct CollisionResult {
    bool m_hasCollision = false;
    std::vector<glm::dvec3> m_normals;           // Collision normals. Pointing from A to B.
    std::vector<glm::dvec3> m_contactPoints;    // Contact points in world space
    std::vector<double> m_penetrationDepths;    // How much objects are overlapping at each contact point

    // Local space contact points for dynamic overlap calculation
    std::vector<glm::dvec3> m_contactPointsLocalA;  // Contact points relative to collider A
    std::vector<glm::dvec3> m_contactPointsLocalB;  // Contact points relative to collider B

    // For collision resolution optimization
    std::vector<double> m_collisionMasses;      // Collision mass for each contact point
    bool m_collisionMassesCalculated = false;   // Flag to avoid recalculation

    // For collision resolution - now type-safe!
    Collider* m_colliderA = nullptr;
    Collider* m_colliderB = nullptr;
    
    // Default constructor for no collision
    CollisionResult() = default;
    
    // Constructor with collider references (mandatory for collisions)
    CollisionResult(bool collision, const std::vector<glm::dvec3>& norms, const std::vector<glm::dvec3>& contacts, 
                   const std::vector<double>& depths, Collider* a, Collider* b)
        : m_hasCollision(collision), m_normals(norms), m_contactPoints(contacts), m_penetrationDepths(depths), 
          m_colliderA(a), m_colliderB(b) {
        if (collision) {
            if (!a || !b) {
                throw std::invalid_argument("CollisionResult: Colliders cannot be null for collision results");
            }
            calculateLocalContactPoints();
        }
    }
    
    // Single contact point constructor
    CollisionResult(bool collision, const glm::dvec3& norm, const glm::dvec3& contact, double depth, Collider* a, Collider* b)
        : m_hasCollision(collision), m_colliderA(a), m_colliderB(b) {
        if (collision) {
            if (!a || !b) {
                throw std::invalid_argument("CollisionResult: Colliders cannot be null for collision results");
            }
            m_normals.push_back(norm);
            m_contactPoints.push_back(contact);
            m_penetrationDepths.push_back(depth);
            calculateLocalContactPoints();
        }
    }

private:
    // Helper function to calculate and store local contact points
    void calculateLocalContactPoints();
};