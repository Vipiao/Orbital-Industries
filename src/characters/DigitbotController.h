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
    
    // Process physics (called during physics update)
    void physics();
    
    // Configure thrust properties
    void setThrustStrength(double strength);
    
private:
    DigitbotPhysics* m_physics;
    PhysicsEngine* m_physicsEngine;
    glm::ivec3 m_movementDirection;
    double m_thrustStrength;
};