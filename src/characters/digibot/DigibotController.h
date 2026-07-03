// DigibotController.h
#pragma once

#include <glm/glm.hpp>
#include <memory>

class DigibotPhysics;
class PhysicsEngine;
class RigidBody;

class DigibotController {
public:
    enum class LockState { UNLOCKED, TRANSLATION_LOCK, FULL_LOCK };

public:
    DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine);
    ~DigibotController() = default;
    
    // Set the desired movement direction
    void setMovementDirection(const glm::ivec3& direction);

    // Set the view direction (world space)
    void setViewDirection(const glm::dvec3& viewDirection);

    // Per-frame update for view direction rotation when locked to grid
    void updatePerFrame(double deltaTimeRemainder);

    // Get the angular velocity of the surface we're on/locked to (per physics timestep)
    glm::dvec3 getSurfaceAngularVelocity() const { return m_surfaceAngularVelocity; }

    // Ground contact info for animation (valid only in walking mode)
    bool hasGroundContact() const { return m_hasGroundContact; }
    glm::dvec3 getGroundContactPoint() const { return m_groundContactPoint; }
    glm::dvec3 getGroundSurfaceNormal() const { return m_groundSurfaceNormal; }
    std::weak_ptr<RigidBody> getWalkingTargetRigidBody() const { return m_walkingTargetRigidBody; }

    // Set the roll input (-1 for left/Q, +1 for right/E, 0 for none)
    void setRollInput(int rollInput);

    // Jetpack control
    bool isJetpackEnabled() const { return m_jetpackEnabled; }
    void setJetpackEnabled(bool enabled);
    
    // Process physics (called during physics update)
    void physics();
    
    // Configure thrust properties
    void setThrustStrength(double strength);

    // Get the current view direction
    const glm::dvec3& getViewDirection() const { return m_viewDirection; }

    // Configure maximum roll rate
    void setMaxRollRate(double maxRate);

    // Set up direction lock state (true = locked, false = free to change)
    void setLockUpDirection(bool locked) { m_upDirectionLocked = locked; }

    // Lock target: the rigid body to match velocity/rotation with (e.g. a grid's body)
    void setTargetRigidBody(std::weak_ptr<RigidBody> rigidBody);
    void unlock();
    void setLockState(LockState state) { m_lockState = state; }
    LockState getLockState() const { return m_lockState; }
    
private:
    // Frames spent without ground contact (physics timestep units)
    uint64_t m_framesWithoutContact{0};
    
    // Physics handlers for different movement modes
    void handleFlying();
    void handleWalking();

    DigibotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;
    glm::ivec3 m_movementDirection;
    double m_thrustStrength;
    double m_angularAccelerationMax;  // Maximum angular acceleration (rad/s^2)
    double m_maxRollRate;             // Maximum roll rate (rad/s)
    int m_rollInput;                  // -1 for roll left, +1 for roll right, 0 for none
    glm::dvec3 m_viewDirection;       // Current view direction in world space

    // Lock state (target body is typically a grid's rigid body, set by the game layer)
    LockState m_lockState;
    std::weak_ptr<RigidBody> m_targetRigidBody;
    double m_translationLockStrength;

    // Jetpack mode
    bool m_jetpackEnabled;

    // Walking mode parameters
    double m_targetHoverHeight;
    double m_maxGroundAcceleration;
    double m_targetWalkSpeed;
    double m_walkingThrustStrength;
    double m_groundSelectionBias;
    double m_maxGroundAngle;
    double m_maxLockedGroundAngle;

    // Target rigid body from walking (set in handleWalking, empty if not walking or no contact)
    std::weak_ptr<RigidBody> m_walkingTargetRigidBody;

    // Angular velocity of surface we're on/locked to (zero if not on rotating surface)
    glm::dvec3 m_surfaceAngularVelocity{0.0, 0.0, 0.0};

    // Ground contact state (updated each physics step in handleWalking)
    bool m_hasGroundContact{false};
    glm::dvec3 m_groundContactPoint{0.0, 0.0, 0.0};
    // Surface normal for foot placement: normalize(bodyPos - contactPoint). This is the
    // direction toward the body, independent of the (possibly locked) orientation up.
    glm::dvec3 m_groundSurfaceNormal{0.0, 0.0, 1.0};

    // Up direction lock state
    bool m_upDirectionLocked{false};
    glm::dvec3 m_cachedModifiedUp{0.0, 0.0, 0.0};  // In rigid body local coordinates
    std::weak_ptr<RigidBody> m_cachedRigidBody;  // Rigid body the cache is relative to
};