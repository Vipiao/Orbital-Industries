// DigitbotPlayerController.cpp
#include "DigitbotPlayerController.h"
#include "Digitbot.h"
#include "../physics/RigidBody.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/KeyboardHandler.h"
#include "../graphics/MouseHandler.h"

DigitbotPlayerController::DigitbotPlayerController(GraphicsEngine* graphics)
    : m_graphics(graphics)
{
    if (!m_graphics) {
        throw std::runtime_error("DigitbotPlayerController: GraphicsEngine cannot be null");
    }
}

void DigitbotPlayerController::setPilotableCharacter(std::weak_ptr<Digitbot> character) {
    m_pilotableCharacter = character;
}

std::weak_ptr<Digitbot> DigitbotPlayerController::getPilotableCharacter() const {
    return m_pilotableCharacter;
}

void DigitbotPlayerController::enable() {
    m_enabled = true;
}

void DigitbotPlayerController::disable() {
    m_enabled = false;
}

void DigitbotPlayerController::update(double deltaTime, glm::dvec3& cameraPosition, glm::dquat& cameraOrientation) {
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
    
    // Position camera in third-person view
    // Get character position and orientation
    RigidBody* rigidBody = character->getRigidBody();
    if (rigidBody) {
        // Calculate camera position based on character orientation
        glm::dvec3 offsetInWorld = rigidBody->m_orientation * m_cameraOffset;
        cameraPosition = rigidBody->m_position + offsetInWorld;
        
        // Set camera orientation to match character's orientation
        cameraOrientation = rigidBody->m_orientation;
    }
}