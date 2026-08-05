// DigibotPlayerController.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

class Digibot;
class DigibotController;
class GraphicsEngine;
class KeyboardHandler;
class MouseHandler;
class Grid;

/**
 * @brief Controls player-driven Digibot characters
 * 
 * Translates keyboard/mouse input into character movement commands
 * and positions the camera for third-person view.
 */
class DigibotPlayerController {
public:
    DigibotPlayerController(GraphicsEngine* graphics);
    ~DigibotPlayerController() = default;
    
    // Core functionality - updates camera and sends inputs to character
    void update(DigibotController* controller, glm::dvec3& cameraPosition, glm::dquat& cameraOrientation, double timeRemainder);
    
    // Character selection
    void setPilotableCharacter(std::weak_ptr<Digibot> character);
    std::weak_ptr<Digibot> getPilotableCharacter() const;
    
    // Enable/disable
    void enable();
    void disable();
    bool isEnabled() const { return m_enabled; }

    // First-person view state (toggled with the T key while enabled)
    bool isFirstPerson() const { return m_firstPerson; }

    // Once per physics step: lock raycasting and movement input mapping
    void stepControl(DigibotController* controller, const std::vector<std::weak_ptr<Grid>>& availableGrids, double interactionRange);

    // Get the angular velocity of the surface we're on/locked to (per physics timestep)
    glm::dvec3 getSurfaceAngularVelocity() const;

private:
    // References
    GraphicsEngine* m_graphics;
    std::weak_ptr<Digibot> m_pilotableCharacter;
    
    // Sync the character's head visibility with the current view state
    void applyHeadVisibility();

    // State
    bool m_enabled{false};
    bool m_firstPerson{false};

    // Third-person camera offset (in local character space)
    glm::dvec3 m_cameraOffset{0., -2.0, 0.8}; // Behind and above character

    // First-person forward lean when looking straight down (avoids staring down the neck)
    double m_firstPersonLookDownLean{0.2};

    // Lock state
    bool m_needsRaycast{false};
    bool m_needsFullLockRaycast{false};

    // Mouse movement gathered while seated in a cockpit, in pixels. Frames and
    // physics steps run at their own rates, so the movement is summed per frame
    // and drained once per step: no sample is dropped and the ship answers to the
    // distance the mouse travelled rather than to the frame rate it travelled at.
    glm::dvec2 m_pilotMouseTravel{0.0, 0.0};

    // The pilot's rotation intent for the ship, from the mouse travel above and
    // the roll keys. Zero whenever the character is not seated.
    glm::dvec3 pilotRotationCommand(bool seated);
};