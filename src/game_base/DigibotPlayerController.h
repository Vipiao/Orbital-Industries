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

    // Physics callback for lock raycasting
    void onPhysicsUpdateComplete(DigibotController* controller, const std::vector<std::weak_ptr<Grid>>& availableGrids, double interactionRange);

    // Get the angular velocity of the surface we're on/locked to (per physics timestep)
    glm::dvec3 getSurfaceAngularVelocity() const;

private:
    // References
    GraphicsEngine* m_graphics;
    std::weak_ptr<Digibot> m_pilotableCharacter;
    
    // State
    bool m_enabled{false};
    
    // Third-person camera offset (in local character space)
    glm::dvec3 m_cameraOffset{0., -2.0, 0.8}; // Behind and above character

    // Lock state
    bool m_needsRaycast{false};

    // Frame timing for view rotation
    double m_lastTimeRemainder{0.0};
};