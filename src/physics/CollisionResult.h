// CollisionResult.h
#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <stdexcept>

// Forward declaration
class Collider;

struct ContactData {
    glm::dvec3 normal;
    double penetration;
    glm::dvec3 compliantNormal;
    double compliantPenetration;
    
    // Constructor for non-compliant contacts
    ContactData(const glm::dvec3& norm, double pen) 
        : normal(norm), penetration(pen), compliantNormal(0.0), compliantPenetration(-1.0) {}
    
    // Constructor for compliant contacts  
    ContactData(const glm::dvec3& norm, double pen, const glm::dvec3& compNorm, double compPen)
        : normal(norm), penetration(pen), compliantNormal(compNorm), compliantPenetration(compPen) {}
};

// Collision data stored on individual colliders
struct CollisionData {
    Collider* otherCollider;
    std::vector<ContactData> contactData;
    std::vector<glm::dvec3> contactPoints;
    std::vector<glm::dvec3> contactPointsLocalA;
    std::vector<glm::dvec3> contactPointsLocalB;
};

struct CollisionResult {
    bool m_hasCollision = false;
    std::vector<ContactData> m_contactData;     // Combined normal/penetration/compliant data
    std::vector<glm::dvec3> m_contactPoints;    // Contact points in world space

    // Local space contact points for dynamic overlap calculation
    std::vector<glm::dvec3> m_contactPointsLocalA;  // Contact points relative to collider A
    std::vector<glm::dvec3> m_contactPointsLocalB;  // Contact points relative to collider B

    // For collision resolution optimization
    bool m_isFiltered = false;                  // Flag to indicate collision was filtered out

    // Colliders involved in collision (resolution gets RigidBody via PointerStorage)
    Collider* m_colliderA = nullptr;
    Collider* m_colliderB = nullptr;
    
    // Default constructor for no collision
    CollisionResult() = default;
    
    // Main constructor with collision data
    CollisionResult(bool collision, std::vector<ContactData> contactData, std::vector<glm::dvec3> contacts, 
                   Collider* a, Collider* b)
        : m_hasCollision(collision), m_contactData(std::move(contactData)), m_contactPoints(std::move(contacts)),
          m_colliderA(a), m_colliderB(b) {
        if (collision) {
            if (!a || !b) {
                throw std::invalid_argument("CollisionResult: Colliders cannot be null for collision results");
            }
            calculateLocalContactPoints();
        }
    }
    
    // Constructor with pre-calculated local contact points (for cache optimization)
    CollisionResult(bool collision, std::vector<ContactData> contactData, std::vector<glm::dvec3> contacts, 
                   Collider* a, Collider* b, std::vector<glm::dvec3> localA, std::vector<glm::dvec3> localB)
        : m_hasCollision(collision), m_contactData(std::move(contactData)), m_contactPoints(std::move(contacts)),
          m_contactPointsLocalA(std::move(localA)), m_contactPointsLocalB(std::move(localB)), 
          m_colliderA(a), m_colliderB(b) {
        if (collision && (!a || !b)) {
            throw std::invalid_argument("CollisionResult: Colliders cannot be null for collision results");
        }
        // Skip calculateLocalContactPoints() since we already have the local coords
    }

    // Constructor for filtered collision results
    CollisionResult(Collider* a, Collider* b, bool filtered)
        : m_hasCollision(false), m_isFiltered(filtered), m_colliderA(a), m_colliderB(b) {
    }

private:
    // Helper function to calculate and store local contact points
    void calculateLocalContactPoints();
};