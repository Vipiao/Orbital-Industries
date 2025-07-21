// PhysicsEngine.h
#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "CollisionDetector.h"
#include "CollisionResult.h"
#include "ColliderReference.h"
#include <chrono>

// Forward declaration
class TimeHandler;
class DebugRenderer;

class PhysicsEngine {
public:
    // Simple rigid body structure
    struct RigidBody : public ColliderReference {
        glm::dvec3 m_position;        // Position in world space
        glm::dvec3 m_velocity;        // Linear velocity (already includes deltaTime)
        glm::dvec3 m_forces;          // Accumulated forces
        
        glm::dquat m_orientation;     // Orientation quaternion
        glm::dvec3 m_angularMomentumBody; // Angular momentum L in body space (primary state variable)
        glm::dvec3 m_torques;         // Accumulated torques
        
        double m_mass;                // Mass in kg
        glm::dmat3 m_inertiaTensor;   // Inertia tensor in local space
        double m_invMass;             // Cached inverse mass (1/mass)
        glm::dmat3 m_invInertiaTensor; // Cached inverse inertia tensor in local space
        
        bool m_isStatic;              // If true, this body won't move

        Collider* m_collider;         // Associated collider for collision detection
        glm::dvec3 m_colliderOffset;  // Offset from center of mass to collider origin (in local space)
    };

    // State machine enums
    enum class RunState {
        APPLY_FORCES,
        UPDATE_POSITIONS, 
        HANDLE_COLLISIONS,
        DONE
    };
    
    enum class CollisionProcessState { DETECT, RESOLVE, SEPARATE, DONE };
    
    PhysicsEngine(TimeHandler* timeHandler);
    ~PhysicsEngine();
    
    // Add a rigid body to the simulation
    RigidBody* addRigidBody(const glm::dvec3& position, 
                           const glm::dquat& orientation,
                           double mass = 1.0, 
                           const glm::dmat3& inertiaTensor = glm::dmat3(1.0),
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
    bool runUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer) { m_debugRenderer = debugRenderer; }
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }
    
private:
    // Physics simulation steps
    void applyForces();
    void updatePositions();
    bool handleCollisionsUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    void resolveCollision(CollisionResult& collision);
    void separateOverlaps(CollisionResult& collision);
    
    // Static helper functions for collision resolution
    static double getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                  const glm::dvec3& contactPoint, const glm::dvec3& normal);

    // State machine variables
    RunState m_runState = RunState::APPLY_FORCES;
    CollisionProcessState m_collisionProcessState = CollisionProcessState::DETECT;
    size_t m_currentCollisionIndex = 0;
    int m_separationIteration = 0;
    std::vector<CollisionResult> m_activeCollisions;
    
    TimeHandler* m_timeHandler;

    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    glm::dvec3 m_gravity{0.0, 0.0, 0.0}; // Default zero gravity
    uint64_t m_currentPhysicsTimeStep{0};

    CollisionDetector m_collisionDetector;

    // Debug visualization
    DebugRenderer* m_debugRenderer = nullptr;
};