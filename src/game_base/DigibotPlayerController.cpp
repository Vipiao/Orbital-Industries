// DigibotPlayerController.cpp
#include "DigibotPlayerController.h"
#include "../characters/digibot/Digibot.h"
#include "../characters/digibot/DigibotController.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../game_base/Grid.h"
#include "graphics/GraphicsEngine.h"
#include "../characters/ArticulationUtils.h"
#include "graphics/KeyboardHandler.h"
#include "graphics/MouseHandler.h"
#include <iostream>
#include "utils/GridGeometry.h"
#include "InputSettings.h"
#include "GridRaycast.h"
#include <optional>

DigibotPlayerController::DigibotPlayerController(GraphicsEngine* graphics)
    : m_graphics(graphics)
{
    if (!m_graphics) {
        throw std::runtime_error("DigibotPlayerController: GraphicsEngine cannot be null");
    }
}

void DigibotPlayerController::setPilotableCharacter(std::weak_ptr<Digibot> character) {
    auto previous = m_pilotableCharacter.lock();
    auto next = character.lock();
    if (previous == next) {
        return;
    }

    // Never leave a character we no longer control headless
    if (previous) {
        previous->setHeadVisible(true);
    }

    m_pilotableCharacter = character;
    applyHeadVisibility();
}

std::weak_ptr<Digibot> DigibotPlayerController::getPilotableCharacter() const {
    return m_pilotableCharacter;
}

void DigibotPlayerController::enable() {
    if (m_enabled) {
        return;
    }
    m_enabled = true;
    applyHeadVisibility();
}

void DigibotPlayerController::disable() {
    if (!m_enabled) {
        return;
    }
    m_enabled = false;
    applyHeadVisibility();
}

void DigibotPlayerController::applyHeadVisibility() {
    auto character = m_pilotableCharacter.lock();
    if (character) {
        // The head is hidden only while we are looking out of it
        character->setHeadVisible(!(m_enabled && m_firstPerson));
    }
}

glm::dvec3 DigibotPlayerController::getSurfaceAngularVelocity() const {
    auto character = m_pilotableCharacter.lock();
    if (!character) {
        return glm::dvec3(0.0, 0.0, 0.0);
    }
    DigibotController* controller = character->getController();
    if (!controller) {
        return glm::dvec3(0.0, 0.0, 0.0);
    }
    return controller->getSurfaceAngularVelocity();
}

void DigibotPlayerController::stepControl(DigibotController* controller, const std::vector<std::weak_ptr<Grid>>& availableGrids, double interactionRange) {
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
    
    // Get interpolation time for accurate raycasting
    auto [_, timeRemainder] = m_graphics->getRenderParameters();

    std::optional<GridRayHit> hit{
        GridRaycast::closestHit(availableGrids, rayStart, rayEnd, timeRemainder)};

    // Lock to the closest grid if found (the controller only needs its rigid body)
    if (hit) {
        std::weak_ptr<RigidBody> targetBody;
        if (auto lockedGrid = hit->m_grid.lock()) {
            targetBody = lockedGrid->getRigidBody();
        }
        if (m_needsFullLockRaycast) {
            controller->setLockState(DigibotLockState::FULL_LOCK);
            controller->setTargetRigidBody(targetBody);
        } else {
            controller->setLockState(DigibotLockState::TRANSLATION_LOCK);
            controller->setTargetRigidBody(targetBody);
        }
    } else {
        std::cout << "No grid found to lock to" << std::endl;
    }

    m_needsRaycast = false;
    m_needsFullLockRaycast = false;
}

void DigibotPlayerController::update(DigibotController* controller, glm::dvec3& cameraPosition, glm::dquat& cameraOrientation, double timeRemainder) {
    if (!m_enabled || !m_graphics) {
        return;
    }
    
    // Handle lock input state machine (ignored while docked/seated so the lock
    // settings survive the cockpit episode untouched)
    if (controller && controller->getDockingState() == DigibotController::DockingState::FREE) {
        KeyboardHandler* keyboard = m_graphics->getKeyboardHandler();
        if (keyboard) {
            // Calculate full lock threshold (0.3 seconds worth of frames)
            uint64_t fullLockThreshold = static_cast<uint64_t>(0.3 * m_graphics->getFrameRate());
            
            // Get current lock state
            DigibotLockState lockState = controller->getLockState();
            bool longPress = keyboard->m_z.isDown() && keyboard->m_z.timeDown() == fullLockThreshold;
            bool shortRelease = keyboard->m_z.justReleased() && keyboard->m_z.timeDown() < fullLockThreshold;
            
            if (lockState == DigibotLockState::UNLOCKED) {
                // Quick press -> translation lock
                if (shortRelease) {
                    m_needsRaycast = true;
                }
                // Long press -> full lock
                if (longPress) {
                    m_needsRaycast = true;
                    m_needsFullLockRaycast = true;
                }
            } else if (lockState == DigibotLockState::TRANSLATION_LOCK) {
                // Quick press -> unlock
                if (shortRelease) {
                    controller->unlock();
                }
                // Long press -> full lock (no raycast needed, keep current target)
                if (longPress) {
                    controller->setLockState(DigibotLockState::FULL_LOCK);
                }
            } else if (lockState == DigibotLockState::FULL_LOCK) {
                // Quick press -> unlock
                if (shortRelease) {
                    controller->unlock();
                }
                // Long press -> raycast for new target, keep lock regardless
                if (longPress) {
                    m_needsRaycast = true;
                    m_needsFullLockRaycast = true;
                }
            }
        }
    }
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

    // Send up direction lock state to controller
    // When caps lock is DOWN, direction is FREE (not locked)
    if (controller) {
        controller->setLockUpDirection(!keyboard->m_capsLock.isDown());
    }

    // Toggle jetpack with X key (ignored while docked/seated)
    if (keyboard->m_x.justPressed() &&
        controller->getDockingState() == DigibotController::DockingState::FREE) {
        controller->setJetpackEnabled(!controller->isJetpackEnabled());
    }

    // Escape leaves the cockpit seat (back into the docking corridor to climb out)
    if (keyboard->m_esc.justPressed() &&
        controller->getDockingState() == DigibotController::DockingState::SEATED) {
        controller->requestUnseat();
    }

    // Toggle first-person view with T key
    if (keyboard->m_t.justPressed()) {
        m_firstPerson = !m_firstPerson;
        applyHeadVisibility();
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
    auto rigidBodyWeak = character->getRigidBody();
    auto rigidBody = rigidBodyWeak.lock();
    if (!rigidBody) {
        return;
    }
    
    // Get interpolated transform for the entire function
    glm::dvec3 interpolatedPos;
    glm::dquat interpolatedOrientation;
    rigidBody->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOrientation);
    
    // Get up vector from rigid body orientation
    glm::dvec3 upVector = interpolatedOrientation * glm::dvec3(0.0, 0.0, 1.0);
    glm::dvec3 rightVector = interpolatedOrientation * glm::dvec3(1.0, 0.0, 0.0);
    
    // Get current view direction (already includes surface rotation from framePreRender)
    glm::dvec3 currentViewDir = controller->getViewDirection();

    // Apply mouse input to view direction
    if (mouseHandler->getMouseLock()) {
        // Step 1: Create orientation quaternion using look-at function
        glm::dquat viewQuat = glm::conjugate(ArticulationUtils::quatLookAtYForward(currentViewDir, upVector));
        
        // Step 2: Apply mouse rotations
        const double mouseSensitivity = InputSettings::MOUSE_SENSITIVITY;
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        double pitchAngle = -mouseMovement.y * mouseSensitivity;
        glm::dquat pitchQuat = glm::angleAxis(pitchAngle, glm::dvec3(1.0, 0.0, 0.0));
        
        // Transform upVector into viewQuat's coordinate frame for yaw axis
        glm::dvec3 yawAxis = glm::conjugate(viewQuat) * upVector;
        double yawAngle = -mouseMovement.x * mouseSensitivity;
        glm::dquat yawQuat = glm::angleAxis(yawAngle, yawAxis);

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

    if (m_firstPerson) {
        // First person: camera sits where the (hidden) head is
        cameraPosition = interpolatedPos +
            interpolatedOrientation * character->getHeadLocalPosition();

        // Lean forward as the view pitches down, so we look past the body instead of
        // into the neck. Lean direction is the horizontal component of the view.
        double downFactor = glm::max(0.0, -glm::dot(viewDir, upVector));
        glm::dvec3 leanDir = viewDir - glm::dot(viewDir, upVector) * upVector;
        double leanDirLength = glm::length(leanDir);
        if (leanDirLength > 1e-6) {
            leanDir /= leanDirLength;
        } else {
            // Looking straight up/down: horizontal view is undefined, use body forward
            leanDir = interpolatedOrientation * glm::dvec3{0.0, 1.0, 0.0};
        }
        cameraPosition += leanDir * (m_firstPersonLookDownLean * downFactor);
    } else {
        // Third person: offset behind character in view direction with height
        cameraPosition = interpolatedPos +
            m_cameraOffset.x * rightVector +
            viewDir * m_cameraOffset.y +
            upVector * m_cameraOffset.z;
    }
}