// CharacterControlTool.h
#pragma once

#include <memory>
#include <glm/glm.hpp>

class GameBase;
class RadialMenu;

class CharacterControlTool {
public:
    CharacterControlTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange);
    ~CharacterControlTool();
    
    // Activation control
    void activate();
    void deactivate();
    bool isActive() const { return m_active; }
    void toggle(); // Toggle active state

    // Callback hooks - will be used for character control later
    void preRenderCallback(bool doToggle);
    void onPhysicsUpdateComplete();
    
private:
    double m_interactionRange;
    GameBase* m_gameBase;
    RadialMenu* m_radialMenu;
    
    // Internal state
    bool m_active{false};
    int64_t m_characterControlNodeId{-1};
    int m_playIconTextureIndex{-1};
    
    // Colors for active/inactive states
    glm::dvec4 m_activeColor{0.0, 0.8, 0.2, 0.8};   // Bright green
    glm::dvec4 m_inactiveColor{0.3, 0.3, 0.3, 0.5}; // Gray
};