// ColorTool.h
#pragma once

#include <memory>
#include <functional>
#include <glm/glm.hpp>

// Forward declarations
class GameBase;
class RadialMenu;

class ColorTool {
public:
    ColorTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId);
    ~ColorTool();
    
    // Activation control
    void activate();
    void deactivate();
    bool isActive() const { return m_active; }
    
    // Callback hooks
    void preRenderCallback();
    void onPhysicsUpdateComplete();
    
private:
    GameBase* m_gameBase;
    RadialMenu* m_radialMenu;
    
    // Internal state
    glm::dvec4 m_currentColor{1.0, 1.0, 1.0, 1.0};
    bool m_active{false};
    
    // Menu structure node IDs
    int64_t m_colorToolParentId{-1};
    int64_t m_hueNodeId{-1};
    int64_t m_saturationValueNodeId{-1};
    int64_t m_keyNodeId{-1};
    
    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void createHueSubmenus();
    void createSaturationValueSubmenus();
    void createKeySubmenus();
    
    // Color modification callbacks
    void onHueSelected(int value);
    void onSaturationValueSelected(int value);
    void onKeySelected(int value);
};