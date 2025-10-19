// DigitbotPlayerController.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class KeyboardHandler;
class MouseHandler;
class Digitbot;

/**
 * @brief Controls player-driven Digitbot characters
 * 
 * Translates keyboard/mouse input into character movement commands
 * and positions the camera for third-person view.
 */
class DigitbotPlayerController {
public:
    DigitbotPlayerController(GraphicsEngine* graphics);
    ~DigitbotPlayerController() = default;
    
    // Core functionality - updates camera and sends inputs to character (timeRemainder for interpolation)
    void update(glm::dvec3& cameraPosition, glm::dquat& cameraOrientation, double timeRemainder);
    
    // Character selection
    void setPilotableCharacter(std::weak_ptr<Digitbot> character);
    std::weak_ptr<Digitbot> getPilotableCharacter() const;
    
    // Enable/disable
    void enable();
    void disable();
    bool isEnabled() const { return m_enabled; }
    
private:
    // References
    GraphicsEngine* m_graphics;
    std::weak_ptr<Digitbot> m_pilotableCharacter;
    
    // State
    bool m_enabled{false};
    
    // Third-person camera offset (in local character space)
    glm::dvec3 m_cameraOffset{0.0, -3.0, 1.0}; // Behind and above character
};