// DigibotController.h
#pragma once

#include "DigibotModeTypes.h"
#include "DigibotFlyingMode.h"
#include "DigibotWalkingMode.h"
#include "DigibotDockingMode.h"
#include "DigibotInput.h"
#include <glm/glm.hpp>
#include <memory>

class DigibotPhysics;
class PhysicsEngine;
class RigidBody;

// Orchestrates the digibot movement modes (flying, walking, cockpit docking/seated),
// blends their wrenches by docking authority, applies them and the equal-opposite
// reactions, and owns the shared input/lock/docking state.
class DigibotController {
public:
    enum class DockingState { FREE, DOCKED, SEATED };

    // Docking transition this step's physics asked for. The controller never
    // applies these itself; the owner of the docking state machine (the game
    // layer's coordinator) reads and applies them after control.
    enum class DockingTransition {
        NONE,
        SEAT,     // docked and reached the seat: capture
        UNSEAT,   // seated and restraint overwhelmed: thrown into the corridor
        RELEASE,  // docked and climbed out: back to free movement
    };

public:
    DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine);
    ~DigibotController() = default;

    // Drive every input from a single value. The lock target is passed already
    // resolved (empty when unlocked); the id it came from is the caller's concern.
    // Side-effecting setters fire only on a change, so this is safe every tick.
    void applyInput(const DigibotInput& input, const std::weak_ptr<RigidBody>& lockTargetBody);

    // Snapshot the current control intent. The lock target is left as id 0 for the
    // caller to fill, since the controller holds the target as a body, not an id.
    DigibotInput captureInput() const;

    // Set the desired movement direction
    void setMovementDirection(const glm::ivec3& direction);

    // Current movement input (x right, y forward, z up). The game layer reads it
    // to drive ship thrusters while this digibot is seated in a cockpit.
    const glm::ivec3& getMovementDirection() const { return m_movementDirection; }

    // Rotation asked of the ship this digibot is seated in, as an angular
    // acceleration in the cockpit's frame (rad/s^2). Held here beside the movement
    // input; the digibot's own body ignores it, the game layer reads it to drive
    // the ship's reaction wheels.
    void setRotationCommand(const glm::dvec3& command) { m_rotationCommand = command; }
    const glm::dvec3& getRotationCommand() const { return m_rotationCommand; }

    // Set the view direction (world space)
    void setViewDirection(const glm::dvec3& viewDirection);

    // Per-frame update for view direction rotation when locked to grid
    void frameUpdate(double deltaTimeRemainder);

    // Get the angular velocity of the surface we're on/locked to (per physics timestep)
    glm::dvec3 getSurfaceAngularVelocity() const { return m_surfaceAngularVelocity; }

    // Ground contact info for animation (valid only in walking mode)
    bool hasGroundContact() const { return m_walkingMode.hasGroundContact(); }
    glm::dvec3 getGroundContactPoint() const {
        return m_walkingMode.getGroundContactPoint();
    }
    glm::dvec3 getGroundSurfaceNormal() const {
        return m_walkingMode.getGroundSurfaceNormal();
    }
    std::weak_ptr<RigidBody> getWalkingTargetRigidBody() const {
        return m_walkingMode.getWalkingTargetRigidBody();
    }

    // Set the roll input (-1 for left/Q, +1 for right/E, 0 for none)
    void setRollInput(int rollInput);

    // Jetpack control
    bool isJetpackEnabled() const { return m_isJetpackEnabled; }
    void setJetpackEnabled(bool enabled);

    // Turn current inputs into forces on the body; runs once per physics
    // step, before integration
    void stepControl();

    // Configure thrust properties
    void setThrustStrength(double strength);

    // Get the current view direction
    const glm::dvec3& getViewDirection() const { return m_viewDirection; }

    // Configure maximum roll rate
    void setMaxRollRate(double maxRate);

    // Set up direction lock state (true = locked, false = free to change)
    void setLockUpDirection(bool locked) { m_isUpDirectionLocked = locked; }

    // Lock target: the rigid body to match velocity/rotation with (e.g. a grid's body)
    void setTargetRigidBody(std::weak_ptr<RigidBody> rigidBody);
    std::weak_ptr<RigidBody> getTargetRigidBody() const { return m_targetRigidBody; }
    void unlock();
    void setLockState(DigibotLockState state) { m_lockState = state; }
    DigibotLockState getLockState() const { return m_lockState; }

    // ========== Cockpit docking (driven by the game layer) ==========
    // Engage or refresh the docking target. First call while FREE removes the solid
    // collider and enters the DOCKED state; subsequent calls refresh the target
    // (the grid moves, its centre of mass can shift).
    void setDockingTarget(const DigibotDockingMode::Target& target);
    // Cockpit destroyed or otherwise invalid: force release back to free movement.
    void clearDockingTarget();
    // Escape while seated: back to the docking corridor to climb out.
    void requestUnseat();
    // Seat without the proximity capture, for docking transitions decided elsewhere;
    // the seat restraint pulls the body in from wherever it is in the corridor.
    void forceSeat();
    DockingState getDockingState() const { return m_dockingState; }
    // Valid after stepControl, until the next stepControl overwrites it.
    DockingTransition getDesiredDockingTransition() const { return m_desiredTransition; }

private:
    void applyWrench(const std::weak_ptr<RigidBody>& bodyWeak,
                     const std::shared_ptr<RigidBody>& body,
                     const DigibotWrench& wrench, double scale);
    void releaseDocking();

    DigibotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;

    // Movement modes
    DigibotFlyingMode m_flyingMode{};
    DigibotWalkingMode m_walkingMode;
    DigibotDockingMode m_dockingMode{};

    // Shared input state
    glm::ivec3 m_movementDirection{0, 0, 0};
    glm::dvec3 m_rotationCommand{0.0, 0.0, 0.0};
    int m_rollInput{0};
    glm::dvec3 m_viewDirection{0.0, 1.0, 0.0};
    bool m_isUpDirectionLocked{false};

    // Lock state (target body is typically a grid's rigid body, set by the game layer)
    DigibotLockState m_lockState{DigibotLockState::UNLOCKED};
    std::weak_ptr<RigidBody> m_targetRigidBody{};

    // Jetpack mode
    bool m_isJetpackEnabled{true};

    // Docking state
    DockingState m_dockingState{DockingState::FREE};
    DigibotDockingMode::Target m_dockingTarget{};
    DockingTransition m_desiredTransition{DockingTransition::NONE};

    // Free-movement settings snapshotted on FREE -> DOCKED, restored on release.
    DigibotLockState m_savedLockState{DigibotLockState::UNLOCKED};
    std::weak_ptr<RigidBody> m_savedTargetRigidBody{};
    bool m_savedJetpackEnabled{true};

    // Angular velocity of surface we're on/locked to (zero if not on rotating surface)
    glm::dvec3 m_surfaceAngularVelocity{0.0, 0.0, 0.0};
};
