// PhysicsEngine.h
#pragma once

#include <vector>
#include <memory>
#include "RigidBody.h"
#include "CollisionDetector.h"
#include "CollisionResult.h"
#include <chrono>
#include "utils/Generator.h"
#include "PhysicsUnits.h"

// Forward declaration
class TimeHandler;
class DebugRenderer;

class PhysicsEngine {
public:
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
    std::weak_ptr<RigidBody> addRigidBody(const glm::dvec3& position, 
                                          const glm::dquat& orientation,
                                          double mass = 1.0, 
                                          const glm::dmat3& inertiaTensor = glm::dmat3(1.0),
                                          bool isStatic = false);

    /**
     * @brief Attach a collider to a rigid body
     * @param body Rigid body to attach to
     * @param colliderWeak Collider to attach
     * @param localPosition Position of collider relative to center of mass (body space)
     * @param localOrientation Orientation offset in body space
     * @param isTrigger If true, collider detects but doesn't respond physically
     * @throws std::runtime_error if collider is already attached to a body
     */
    void attachCollider(std::weak_ptr<RigidBody> bodyWeak, std::weak_ptr<Collider> colliderWeak,
                       const glm::dvec3& localPosition = glm::dvec3(0.0),
                       const glm::dquat& localOrientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                       bool isTrigger = false);
    
    // Detach a specific collider from a rigid body
    void detachCollider(std::weak_ptr<RigidBody> bodyWeak, Collider* collider);
    void detachAllColliders(std::weak_ptr<RigidBody> bodyWeak);
    void updateColliderTransform(std::weak_ptr<RigidBody> bodyWeak);
    
    //
    uint64_t getCurrentPhysicsTimeStep() const { return m_currentPhysicsTimeStep; }
    uint64_t getPhysicsHz() const { return m_physicsHz; }

    // Remove a rigid body from the simulation
    void removeRigidBody(std::weak_ptr<RigidBody> bodyWeak);
    
    // Apply a force at the center of mass
    void applyForce(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& force);
    
    // Apply a force at a specific point (will generate torque)
    void applyForceAtPoint(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& force, const glm::dvec3& point);
    
    // Apply a force at a body-local point with a body-local force vector (will generate torque)
    void applyLocalForceAtPoint(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& localForce, const glm::dvec3& localPoint);

    // Apply a torque directly
    void applyTorque(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& torque);
    
    // Set gravity
    void setGravity(const glm::dvec3& gravity);

    // Run physics simulation
    bool runUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Debug support
    void setDebugRenderer(DebugRenderer* debugRenderer) { m_debugRenderer = debugRenderer; }
    DebugRenderer* getDebugRenderer() const { return m_debugRenderer; }

    // Access to collision detector for creating colliders
    CollisionDetector& getCollisionDetector() { return m_collisionDetector; }

    const std::vector<CollisionData>& getCollisions(const Collider* collider) const {
        return m_collisionDetector.getCollisions(collider);
    }

    // Get the timestamp of the last physics step
    std::chrono::time_point<std::chrono::high_resolution_clock> getLastPhysicsStepTime() const { return m_lastPhysicsStepTime; }
    
    // Access to all rigid bodies for external systems
    std::vector<std::weak_ptr<RigidBody>> getRigidBodies() const;

private:
    // Physics simulation steps
    void applyForces();
    void updatePositions();
    bool handleCollisionsUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    void resolveCollision(std::shared_ptr<RigidBody> bodyShared);
    void separateOverlaps(std::shared_ptr<RigidBody> bodyShared);
    
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
    RunState m_runState = RunState::APPLY_FORCES;
    CollisionProcessState m_collisionProcessState = CollisionProcessState::DETECT;
    std::unique_ptr<Generator<bool>> m_collisionGenerator;
    int m_separationIteration = 0;
    size_t m_currentBodyIndex = 0;
    bool m_stepInProgress = false;
    
    TimeHandler* m_timeHandler;

    std::vector<std::shared_ptr<RigidBody>> m_rigidBodies;
    glm::dvec3 m_gravity{0.0, 0.0, 0.0}; // Default zero gravity
    uint64_t m_currentPhysicsTimeStep{0};
    uint64_t m_physicsHz{static_cast<uint64_t>(PhysicsUnits::s_tickRateHz)};

    CollisionDetector m_collisionDetector;

    // Debug visualization
    DebugRenderer* m_debugRenderer = nullptr;

    // Timestamp of the last physics step completion
    std::chrono::time_point<std::chrono::high_resolution_clock> m_lastPhysicsStepTime;
};