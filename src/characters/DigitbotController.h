// DigitbotController.h
#pragma once

#include <glm/glm.hpp>

class DigitbotPhysics;
class PhysicsEngine;

class DigitbotController {
public:
    DigitbotController(DigitbotPhysics* physics, PhysicsEngine* physicsEngine);
    ~DigitbotController() = default;
    
    // Set the desired movement direction
    void setMovementDirection(const glm::ivec3& direction);

    // Set the view direction (world space)
    void setViewDirection(const glm::dvec3& viewDirection);
    
    // Process physics (called during physics update)
    void physics();
    
    // Configure thrust properties
    void setThrustStrength(double strength);

    // Get the current view direction
    const glm::dvec3& getViewDirection() const { return m_viewDirection; }
    
private:
    DigitbotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;
    glm::ivec3 m_movementDirection;
    double m_thrustStrength;
    glm::dvec3 m_viewDirection{0.0, 1.0, 0.0}; // Default forward
};