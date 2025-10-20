// FreeCameraController.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class KeyboardHandler;
class MouseHandler;

class FreeCameraController {
public:
    FreeCameraController(GraphicsEngine* graphics);
    ~FreeCameraController() = default;
    
    // Core functionality - updates passed-in camera position and orientation
    void update(double deltaTime, glm::dvec3& position, glm::dquat& orientation);
    
    // Enable/disable control
    void enable();
    void disable();
    bool isEnabled() const { return m_enabled; }
    
    // Configuration
    void setMovementSpeed(double speed) { m_moveSpeed = speed; }
    double getMovementSpeed() const { return m_moveSpeed; }
    
private:
    GraphicsEngine* m_graphics;
    bool m_enabled;
    double m_moveSpeed;
};