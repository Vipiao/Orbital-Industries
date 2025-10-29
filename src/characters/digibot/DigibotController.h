// DigibotController.h
#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

class DigibotPhysics;
class PhysicsEngine;
class Grid;
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

    // Walking mode state
    uint64_t m_walkingNoiseCounter;
    double m_hitRatio;
    glm::dvec3 m_averageNormal;
    double m_averageGroundDistance;
    glm::dvec3 m_lastHitPoint;
    std::vector<std::weak_ptr<Grid>> m_groundGrids;
    std::vector<double> m_gridWeights;
    
    // Walking mode parameters
    double m_runningAverageAlpha;
    double m_hitRatioThreshold;
    double m_gridWeightRemovalThreshold;
    double m_maxGroundAcceleration;
    double m_targetHoverHeight;
    double m_minGridMassFraction;
};