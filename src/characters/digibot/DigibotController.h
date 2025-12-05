// DigibotController.h
#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>
#include "../../game_base/Grid.h"

class DigibotPhysics;
class PhysicsEngine;
class GridSubsystem;

class DigibotController {
public:
    enum class LockState { UNLOCKED, TRANSLATION_LOCK, FULL_LOCK };

public:
    DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine, GridSubsystem* gridSubsystem);
    ~DigibotController() = default;
    
    // Set the desired movement direction
    void setMovementDirection(const glm::ivec3& direction);

    // Set the view direction (world space)
    void setViewDirection(const glm::dvec3& viewDirection);

    // Per-frame update for view direction rotation when locked to grid
    // Returns the rotation quaternion applied this frame (for UI elements)
    void updatePerFrame(double deltaTimeRemainder);

    // Get the angular velocity of the surface we're on/locked to (per physics timestep)
    glm::dvec3 getSurfaceAngularVelocity() const { return m_surfaceAngularVelocity; }

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

    // Configure roll properties
    void setRollAcceleration(double acceleration);

    // Configure maximum roll rate
    void setMaxRollRate(double maxRate);

    // Set up direction lock state (true = locked, false = free to change)
    void setLockUpDirection(bool locked) { m_upDirectionLocked = locked; }

    // Grid locking
    void setTargetGrid(std::weak_ptr<Grid> grid);
    void unlock();
    void setLockState(LockState state) { m_lockState = state; }
    LockState getLockState() const { return m_lockState; }
    std::weak_ptr<Grid> getTargetGrid() const { return m_targetGrid; }
    
private:
    // Physics handlers for different movement modes
    void handleFlying();
    void handleWalking();

    DigibotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;
    GridSubsystem* m_gridSubsystem;
    glm::ivec3 m_movementDirection;
    double m_thrustStrength;
    double m_angularAccelerationMax;  // Maximum angular acceleration (rad/s^2)
    double m_maxRollRate;             // Maximum roll rate (rad/s)
    double m_rollAcceleration;        // Roll acceleration strength (rad/s^2)
    int m_rollInput;                  // -1 for roll left, +1 for roll right, 0 for none
    glm::dvec3 m_viewDirection;       // Current view direction in world space

    // Grid locking state
    LockState m_lockState;
    std::weak_ptr<Grid> m_targetGrid;
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
    double m_walkingSensorRadius;

    // Target rigid body from walking (set in handleWalking, empty if not walking or no contact)
    std::weak_ptr<RigidBody> m_walkingTargetRigidBody;

    // Angular velocity of surface we're on/locked to (zero if not on rotating surface)
    glm::dvec3 m_surfaceAngularVelocity{0.0, 0.0, 0.0};

    // Up direction lock state
    bool m_upDirectionLocked{false};
    glm::dvec3 m_cachedModifiedUp{0.0, 0.0, 0.0};  // In rigid body local coordinates
    std::weak_ptr<RigidBody> m_cachedRigidBody;  // Rigid body the cache is relative to

    // Last valid contact point cache (in surface local coordinates)
    glm::dvec3 m_lastValidContactPoint{0.0, 0.0, 0.0};
    std::weak_ptr<RigidBody> m_lastValidContactRigidBody;
};