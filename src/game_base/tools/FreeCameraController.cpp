// FreeCameraController.cpp
#include "FreeCameraController.h"
#include "../../graphics/GraphicsEngine.h"
#include "../../graphics/KeyboardHandler.h"
#include "../../graphics/MouseHandler.h"
#include <glm/gtc/quaternion.hpp>
#include <iostream>

FreeCameraController::FreeCameraController(GraphicsEngine* graphics)
    : m_graphics(graphics)
    , m_enabled(true)
    , m_moveSpeed(8.0) // Default movement speed
{
    if (!m_graphics) {
        throw std::runtime_error("FreeCameraController: GraphicsEngine cannot be null");
    }
}

void FreeCameraController::enable() {
    m_enabled = true;
}

void FreeCameraController::disable() {
    m_enabled = false;
}

void FreeCameraController::update(double deltaTime, glm::dvec3& position, glm::dquat& orientation) {
    if (!m_enabled || !m_graphics) {
        return;
    }

    MouseHandler* mouseHandler = m_graphics->getMouseHandler();
    KeyboardHandler* keyboard = m_graphics->getKeyboardHandler();
    
    if (!mouseHandler || !keyboard) {
        return;
    }
    
    // Mouse look (camera rotation)
    if (mouseHandler->getMouseLock()) {
        // Calculate mouse sensitivity based on framerate
        const double mouseSensitivity = 0.0008;
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        // Rotate around Z-axis for yaw (left/right)
        double yawAngle = -mouseMovement.x * mouseSensitivity;
        glm::dquat yawQuat = glm::angleAxis(yawAngle, glm::dvec3(0.0, 0.0, 1.0));
        
        // Rotate around X-axis for pitch (up/down)
        double pitchAngle = -mouseMovement.y * mouseSensitivity;
        glm::dquat pitchQuat = glm::angleAxis(pitchAngle, glm::dvec3(1.0, 0.0, 0.0));

        // Rotate around Y-axis for roll (roll right/roll left)
        const double rollSpeed = 1.65 * deltaTime;
        double rollAngle = keyboard->m_q.isDown()?
            (keyboard->m_e.isDown()?
                0.: -rollSpeed):
            (keyboard->m_e.isDown()?
                rollSpeed: 0.);
        glm::dquat rollQuat = glm::angleAxis(rollAngle, glm::dvec3(0.0, 1.0, 0.0));
        
        // Apply rotations to camera orientation
        orientation = orientation * yawQuat * pitchQuat * rollQuat;
        orientation = glm::normalize(orientation);
    }
    
    // Adjust movement speed with C and V keys
    if (keyboard->m_c.isDown()) {
        m_moveSpeed *= glm::exp(8.0 * deltaTime);
    }
    if (keyboard->m_v.isDown()) {
        m_moveSpeed /= glm::exp(8.0 * deltaTime);
    }
    
    // Calculate movement vectors based on camera orientation
    glm::dvec3 right = orientation * glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 forward = orientation * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 up = orientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Normalize the vectors
    right = glm::normalize(right);
    forward = glm::normalize(forward);
    up = glm::normalize(up);
    
    // Movement direction based on keyboard input
    glm::dvec3 moveDirection(0.0);
    
    if (keyboard->m_w.isDown()) {
        moveDirection += forward;
    }
    if (keyboard->m_s.isDown()) {
        moveDirection -= forward;
    }
    if (keyboard->m_a.isDown()) {
        moveDirection -= right;
    }
    if (keyboard->m_d.isDown()) {
        moveDirection += right;
    }
    if (keyboard->m_space.isDown()) {
        moveDirection += up;
    }
    if (keyboard->m_lShift.isDown()) {
        moveDirection -= up;
    }
    
    // Apply movement if any keys were pressed
    if (glm::length(moveDirection) > 0.0) {
        moveDirection = glm::normalize(moveDirection) * m_moveSpeed * deltaTime;
        position += moveDirection;
    }
}