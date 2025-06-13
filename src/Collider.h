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

struct CollisionResult {
    bool hasCollision = false;
    std::vector<glm::dvec3> normals;           // Collision normals
    std::vector<glm::dvec3> contactPoints;    // Contact points in world space
    std::vector<double> penetrationDepths;    // How much objects are overlapping at each contact point

    // For collision resolution optimization
    std::vector<double> collisionMasses;      // Collision mass for each contact point
    bool collisionMassesCalculated = false;   // Flag to avoid recalculation

    // For collision resolution
    Collider* colliderA = nullptr;
    Collider* colliderB = nullptr;
    
    CollisionResult() = default;
    
    CollisionResult(bool collision, const std::vector<glm::dvec3>& norms, const std::vector<glm::dvec3>& contacts, const std::vector<double>& depths)
        : hasCollision(collision), normals(norms), contactPoints(contacts), penetrationDepths(depths) {}
    
    // Constructor with collider references
    CollisionResult(bool collision, const std::vector<glm::dvec3>& norms, const std::vector<glm::dvec3>& contacts, 
                   const std::vector<double>& depths, Collider* a, Collider* b)
        : hasCollision(collision), normals(norms), contactPoints(contacts), penetrationDepths(depths), 
          colliderA(a), colliderB(b) {}
    
    CollisionResult(bool collision, const glm::dvec3& norm, const glm::dvec3& contact, double depth = 0.0)
        : hasCollision(collision) {
        if (collision) {
            normals.push_back(norm);
            contactPoints.push_back(contact);
            penetrationDepths.push_back(depth);
        }
    }
};

class Collider {
public:
    Collider(const glm::dvec3& position = glm::dvec3(0.0), 
             const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
             ColliderReference* reference = nullptr)
        : position(position)
        , orientation(orientation)
        , reference(reference)
        , collisionBoxMin(0.0)
        , collisionBoxMax(0.0)
    {}
    
    virtual ~Collider() = default;
    
    // Pure virtual methods that must be implemented by derived classes
    virtual CollisionResult collideWith(Collider* other) = 0;
    virtual void updateCollisionBoxes() = 0;
    
    // Double dispatch methods - can be overridden by derived classes
    virtual CollisionResult collideWithBall(BallCollider* ball) = 0;

    // Axis-aligned bounding box collision detection
    virtual bool checkAABBCollision(const Collider* other) const = 0;
    
    // Public member variables
    glm::dvec3 position;
    glm::dquat orientation;
    ColliderReference* reference;

    // Collision box data
    glm::dvec3 collisionBoxMin;
    glm::dvec3 collisionBoxMax;
};