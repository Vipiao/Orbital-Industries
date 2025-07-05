// PhysicsEngine.h
#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "CollisionDetector.h"
#include "CollisionResult.h"
#include "ColliderReference.h"

// Forward declaration
class DebugRenderer;

class PhysicsEngine {
public:
    // Simple rigid body structure
    struct RigidBody : public ColliderReference {
        glm::dvec3 m_position;        // Position in world space
        glm::dvec3 m_velocity;        // Linear velocity (already includes deltaTime)
        glm::dvec3 m_forces;          // Accumulated forces
        
        glm::dquat m_orientation;     // Orientation quaternion
        glm::dvec3 m_angularVelocity; // Angular velocity (already includes deltaTime)
        glm::dvec3 m_torques;         // Accumulated torques
        
        double m_mass;                // Mass in kg
        double m_momentOfInertia;     // Simplified moment of inertia (scalar)
        
        bool m_isStatic;              // If true, this body won't move

        Collider* m_collider;         // Associated collider for collision detection
        glm::dvec3 m_colliderOffset;  // Offset from center of mass to collider origin (in local space)
    };
    
    PhysicsEngine();
    ~PhysicsEngine();
    
    // Add a rigid body to the simulation
    RigidBody* addRigidBody(const glm::dvec3& position, 
                           const glm::dquat& orientation,
                           double mass = 1.0, 
                           double momentOfInertia = 1.0,
                           bool isStatic = false,
                           Collider* collider = nullptr);
    
    //
    uint64_t getCurrentPhysicsTimeStep() const { return m_currentPhysicsTimeStep; }
    
    // Remove a rigid body from the simulation
    void removeRigidBody(RigidBody* body);
    
    // Apply a force at the center of mass
    void applyForce(RigidBody* body, const glm::dvec3& force);
    
    // Apply a force at a specific point (will generate torque)
    void applyForceAtPoint(RigidBody* body, const glm::dvec3& force, const glm::dvec3& point);
    
    // Apply a torque directly
    void applyTorque(RigidBody* body, const glm::dvec3& torque);
    
    // Set gravity
    void setGravity(const glm::dvec3& gravity);

    // Update collider transform for a specific rigid body
    void updateColliderTransform(RigidBody* body);
    
    // Run physics simulation
    void run();

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer) { m_debugRenderer = debugRenderer; }
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }
    
private:
    // Physics simulation steps
    void applyForces();
    void updatePositions();
    void handleCollisions();
    void resolveCollision(CollisionResult& collision);
    void separateOverlaps(CollisionResult& collision);
    
    // Static helper functions for collision resolution
    static double getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                  const glm::dvec3& contactPoint, const glm::dvec3& normal);

    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    glm::dvec3 m_gravity{0.0, 0.0, 0.0}; // Default zero gravity
    uint64_t m_currentPhysicsTimeStep{0};

    CollisionDetector m_collisionDetector;

    // Debug visualization
    DebugRenderer* m_debugRenderer = nullptr;
};