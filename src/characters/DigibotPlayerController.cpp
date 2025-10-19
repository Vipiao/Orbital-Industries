// DigibotPlayerController.cpp
#include "DigibotPlayerController.h"
#include "Digibot.h"
#include "../physics/RigidBody.h"
#include "../graphics/GraphicsEngine.h"
#include "../characters/ArticulationUtils.h"
#include "../graphics/KeyboardHandler.h"
#include "../graphics/MouseHandler.h"
#include <iostream>

DigibotPlayerController::DigibotPlayerController(GraphicsEngine* graphics)
    : m_graphics(graphics)
{
    if (!m_graphics) {
        throw std::runtime_error("DigibotPlayerController: GraphicsEngine cannot be null");
    }
}

void DigibotPlayerController::setPilotableCharacter(std::weak_ptr<Digibot> character) {
    m_pilotableCharacter = character;
}

std::weak_ptr<Digibot> DigibotPlayerController::getPilotableCharacter() const {
    return m_pilotableCharacter;
}

void DigibotPlayerController::enable() {
    m_enabled = true;
}

void DigibotPlayerController::disable() {
    m_enabled = false;
}

void DigibotPlayerController::update(glm::dvec3& cameraPosition, glm::dquat& cameraOrientation, double timeRemainder) {
    if (!m_enabled || !m_graphics) {
        return;
    }
    
    // Check if we have a valid character
    auto character = m_pilotableCharacter.lock();
    if (!character) {
        return;
    }
    
    // Get input handlers
    KeyboardHandler* keyboard = m_graphics->getKeyboardHandler();
    MouseHandler* mouseHandler = m_graphics->getMouseHandler();
    
    if (!keyboard || !mouseHandler) {
        return;
    }
    
    // Process movement input and send to character
    glm::ivec3 moveDirection(0, 0, 0);
    
    // Forward/backward
    if (keyboard->m_w.isDown()) {
        moveDirection.y += 1;
    }
    if (keyboard->m_s.isDown()) {
        moveDirection.y -= 1;
    }
    
    // Left/right
    if (keyboard->m_a.isDown()) {
        moveDirection.x -= 1;
    }
    if (keyboard->m_d.isDown()) {
        moveDirection.x += 1;
    }
    
    // Up/down
    if (keyboard->m_space.isDown()) {
        moveDirection.z += 1;
    }
    if (keyboard->m_lShift.isDown()) {
        moveDirection.z -= 1;
    }
    
    // Send movement to character
    character->setMovementDirection(moveDirection);

    // Get character position and orientation
    RigidBody* rigidBody = character->getRigidBody();
    if (!rigidBody) {
        return;
    }
    
    // Get interpolated transform for the entire function
    glm::dvec3 interpolatedPos;
    glm::dquat interpolatedOrientation;
    rigidBody->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOrientation);
    
    // Get up vector from rigid body orientation
    glm::dvec3 upVector = interpolatedOrientation * glm::dvec3(0.0, 0.0, 1.0);
    
    // Process view direction from mouse input
    if (mouseHandler->getMouseLock()) {
        // Get current view direction
        glm::dvec3 currentViewDir = character->getViewDirection();
        
        // Step 1: Create orientation quaternion using look-at function
        glm::dquat viewQuat = glm::conjugate(ArticulationUtils::quatLookAtYForward(currentViewDir, upVector));
        
        // Step 2: Apply local rotations from mouse input
        const double mouseSensitivity = 0.0014;
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        double pitchAngle = -mouseMovement.y * mouseSensitivity;
        glm::dquat pitchQuat = glm::angleAxis(pitchAngle, glm::dvec3(1.0, 0.0, 0.0));
        
        double yawAngle = -mouseMovement.x * mouseSensitivity;
        glm::dquat yawQuat = glm::angleAxis(yawAngle, glm::dvec3(0.0, 0.0, 1.0));

        glm::dvec3 newViewDir = viewQuat * pitchQuat * yawQuat * glm::dvec3(0.0, 1.0, 0.0);
        
        // Set new view direction
        character->setViewDirection(newViewDir);
    }

    // Position camera in third-person view using view direction
    // Get character position and orientation
    // Get character's view direction
    glm::dvec3 viewDir = character->getViewDirection();
    
    // Create orientation quaternion from view direction and up vector
    cameraOrientation = glm::conjugate(ArticulationUtils::quatLookAtYForward(viewDir, upVector));
    
    // Set camera position - offset behind character in view direction with height
    cameraPosition = interpolatedPos + viewDir * m_cameraOffset.y + upVector * m_cameraOffset.z;
}