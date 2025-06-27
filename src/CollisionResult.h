// CollisionResult.h
#pragma once

#include <glm/glm.hpp>
#include <vector>

// Forward declaration for type-safe collider references
class Collider;

struct CollisionResult {
    bool m_hasCollision = false;
    std::vector<glm::dvec3> m_normals;           // Collision normals
    std::vector<glm::dvec3> m_contactPoints;    // Contact points in world space
    std::vector<double> m_penetrationDepths;    // How much objects are overlapping at each contact point

    // For collision resolution optimization
    std::vector<double> m_collisionMasses;      // Collision mass for each contact point
    bool m_collisionMassesCalculated = false;   // Flag to avoid recalculation

    // For collision resolution - now type-safe!
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