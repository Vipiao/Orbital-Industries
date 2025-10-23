// DigibotController.h
#pragma once

#include <glm/glm.hpp>
#include <memory>

class DigibotPhysics;
class PhysicsEngine;
class Grid;

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

    // Set the roll input (-1 for left/Q, +1 for right/E, 0 for none)
    void setRollInput(int rollInput);
    
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

    // Grid locking
    void setTargetGrid(std::weak_ptr<Grid> grid);
    void unlock();
    void setLockState(LockState state) { m_lockState = state; }
    LockState getLockState() const { return m_lockState; }
    std::weak_ptr<Grid> getTargetGrid() const { return m_targetGrid; }
    
private:
    DigibotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;
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
};