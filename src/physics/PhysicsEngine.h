// PhysicsEngine.h
#pragma once

#include <vector>
#include <memory>
#include "RigidBody.h"
#include "CollisionDetector.h"
#include "CollisionResult.h"
#include <chrono>
#include "../utils/Generator.h"

// Forward declaration
class TimeHandler;
class DebugRenderer;

class PhysicsEngine {
public:
    // State machine enums
    enum class RunState {
        HANDLE_COLLISIONS,
        APPLY_FORCES,
        UPDATE_POSITIONS, 
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
                           bool isStatic = false);

    // Connect a collider to a rigid body
    void connectCollider(RigidBody* body, std::weak_ptr<Collider> colliderWeak);
    
    // Disconnect a collider from a rigid body
    void disconnectCollider(RigidBody* body);
    
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

    // Access to collision detector for creating colliders
    CollisionDetector& getCollisionDetector() { return m_collisionDetector; }

    // Get the timestamp of the last physics step
    std::chrono::time_point<std::chrono::high_resolution_clock> getLastPhysicsStepTime() const { return m_lastPhysicsStepTime; }
    
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

    // Static helper function for compliant collision detection
    static bool shouldUseCompliantHandling(RigidBody* bodyA, RigidBody* bodyB, 
                                          const glm::dvec3& contactPoint, const glm::dvec3& normal,
                                          const glm::dvec3& compliantNormal,
                                          double compliantPenetration,
                                          const glm::dvec3* relativeVel = nullptr);

    // State machine variables
    RunState m_runState = RunState::HANDLE_COLLISIONS;
    CollisionProcessState m_collisionProcessState = CollisionProcessState::DETECT;
    std::unique_ptr<Generator<bool>> m_collisionGenerator;
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

    // Timestamp of the last physics step completion
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastPhysicsStepTime;
};