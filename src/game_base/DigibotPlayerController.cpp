// DigibotPlayerController.cpp
#include "DigibotPlayerController.h"
#include "../characters/digibot/Digibot.h"
#include "../characters/digibot/DigibotController.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../game_base/Grid.h"
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

void DigibotPlayerController::onPhysicsUpdateComplete(DigibotController* controller, const std::vector<std::weak_ptr<Grid>>& availableGrids, double interactionRange) {
    if (!m_needsRaycast || !controller || !m_graphics) {
        return;
    }
    
    // Reset flag
    m_needsRaycast = false;
    
    // Perform ray cast to find target grid
    glm::dvec3 cameraPos = m_graphics->getCamPos();
    glm::dvec3 forward = m_graphics->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 rayStart = cameraPos;
    glm::dvec3 rayEnd = cameraPos + forward * interactionRange;
    
    std::weak_ptr<Grid> closestGrid;
    bool gridFound = false;
    double closestT = -1.0;
    
    // Find closest ray intersection across available grids
    for (const auto& gridWeak : availableGrids) {
        auto gridShared = gridWeak.lock();
        if (!gridShared) continue;
        
        // Transform world ray to grid-local space
        glm::dvec3 gridLocalRayStart = gridShared->worldToGrid(rayStart);
        glm::dvec3 gridLocalRayEnd = gridShared->worldToGrid(rayEnd);
        
        // Perform ray intersection in grid-local space
        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);
        
        // Check if this is a closer hit than what we have so far
        if (result.t >= 0.0 && (!gridFound || result.t < closestT)) {
            closestT = result.t;
            gridFound = true;
            closestGrid = gridWeak;
        }
    }
    
    // Lock to the closest grid if found
    if (gridFound) {
        controller->setLockState(DigibotController::LockState::TRANSLATION_LOCK);
        controller->setTargetGrid(closestGrid);
    } else {
        std::cout << "No grid found to lock to" << std::endl;
    }
}

void DigibotPlayerController::update(DigibotController* controller, glm::dvec3& cameraPosition, glm::dquat& cameraOrientation, double timeRemainder) {
    if (!m_enabled || !m_graphics) {
        return;
    }
    
    // Handle lock input state machine
    if (controller) {
        KeyboardHandler* keyboard = m_graphics->getKeyboardHandler();
        if (keyboard) {
            // Calculate full lock threshold (0.3 seconds worth of frames)
            uint64_t fullLockThreshold = static_cast<uint64_t>(0.3 * static_cast<double>(m_graphics->getFrameRate()));
            
            // Get current lock state
            DigibotController::LockState lockState = controller->getLockState();
            
            // Handle Z button press for lock/unlock
            if (keyboard->m_z.justPressed()) {
                if (lockState == DigibotController::LockState::UNLOCKED) {
                    // Request raycast on next physics update
                    m_needsRaycast = true;
                } else {
                    // Already locked - unlock immediately
                    controller->unlock();
                }
            }
            
            // Handle transition to FULL_LOCK when holding Z
            if (keyboard->m_z.isDown() && lockState == DigibotController::LockState::TRANSLATION_LOCK) {
                if (keyboard->m_z.timeDown() > fullLockThreshold) {
                    controller->setLockState(DigibotController::LockState::FULL_LOCK);
                    std::cout << "Transitioning to FULL_LOCK (not yet implemented)" << std::endl;
                }
            }
        }
    }
    if (!m_enabled || !m_graphics) {
        return;
    }
    
    // Calculate delta time remainder since last frame
    double deltaTimeRemainder = timeRemainder - m_lastTimeRemainder;
    if (deltaTimeRemainder < 0.0) deltaTimeRemainder += 1.0; // Handle wraparound
    m_lastTimeRemainder = timeRemainder;

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

    // Roll input (Q/E keys)
    int rollInput = 0;
    if (keyboard->m_q.isDown()) {
        rollInput -= 1;  // Roll left
    }
    if (keyboard->m_e.isDown()) {
        rollInput += 1;  // Roll right
    }
    
    // Send roll input to character
    if (character) {
        character->setRollInput(rollInput);
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
    
    // Get current view direction
    glm::dvec3 currentViewDir = character->getViewDirection();
    
    // Apply grid rotation if in FULL_LOCK mode
    if (controller && controller->getLockState() == DigibotController::LockState::FULL_LOCK) {
        auto targetGrid = controller->getTargetGrid().lock();
        if (targetGrid) {
            RigidBody* targetGridRigidBody = targetGrid->getRigidBody();
            if (targetGridRigidBody) {
                // Get grid's angular velocity
                glm::dvec3 gridAngularVelocity = targetGridRigidBody->getAngularVelocityWorld();
                double angularVelocityMagnitude = glm::length(gridAngularVelocity);
                
                if (angularVelocityMagnitude > 1e-6) {
                    // Calculate rotation for this frame
                    double rotationAngle = angularVelocityMagnitude * deltaTimeRemainder;
                    glm::dvec3 rotationAxis = gridAngularVelocity / angularVelocityMagnitude;
                    
                    // Create rotation quaternion
                    glm::dquat gridRotationQuat = glm::angleAxis(rotationAngle, rotationAxis);
                    
                    // Apply grid rotation to current view direction
                    currentViewDir = gridRotationQuat * currentViewDir;
                }
            }
        }
    }
    
    // Apply mouse input to view direction
    if (mouseHandler->getMouseLock()) {
        // Step 1: Create orientation quaternion using look-at function
        glm::dquat viewQuat = glm::conjugate(ArticulationUtils::quatLookAtYForward(currentViewDir, upVector));
        
        // Step 2: Apply mouse rotations
        const double mouseSensitivity = 0.0008;
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        double pitchAngle = -mouseMovement.y * mouseSensitivity;
        glm::dquat pitchQuat = glm::angleAxis(pitchAngle, glm::dvec3(1.0, 0.0, 0.0));
        
        double yawAngle = -mouseMovement.x * mouseSensitivity;
        glm::dquat yawQuat = glm::angleAxis(yawAngle, glm::dvec3(0.0, 0.0, 1.0));

        currentViewDir = viewQuat * pitchQuat * yawQuat * glm::dvec3(0.0, 1.0, 0.0);
    }

    // Always set the final view direction (after grid rotation and/or mouse input)
    character->setViewDirection(currentViewDir);

    // Position camera in third-person view using view direction
    // Get character position and orientation
    // Get character's view direction
    glm::dvec3 viewDir = character->getViewDirection();
    
    // Create orientation quaternion from view direction and up vector
    cameraOrientation = glm::conjugate(ArticulationUtils::quatLookAtYForward(viewDir, upVector));
    
    // Set camera position - offset behind character in view direction with height
    cameraPosition = interpolatedPos + viewDir * m_cameraOffset.y + upVector * m_cameraOffset.z;
}