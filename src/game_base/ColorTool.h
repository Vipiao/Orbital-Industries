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

    // Color getters
    glm::dvec4 getCurrentColorRGBA() const;
    glm::dvec4 getCurrentColorHSVA() const;
    
    // Callback hooks
    void preRenderCallback(bool doTryCopy, bool doTryPaste);
    void onPhysicsUpdateComplete();
    
private:
    GameBase* m_gameBase;
    RadialMenu* m_radialMenu;
    
    // Internal state
    glm::dvec4 m_currentColor{0.0, 1.0, 1.0, 1.0}; // HSVA: hue=0, sat=0, val=1, alpha=1 (white)
    bool m_active{false};

    // Copy/paste state
    bool m_doCopy{false};
    bool m_doPaste{false};
    
    // Menu structure node IDs
    int64_t m_colorToolParentId{-1};
    int64_t m_hueNodeId{-1};
    int64_t m_saturationValueNodeId{-1};
    int64_t m_keyNodeId{-1};
    int64_t m_centerNodeId{-1};

    // Symbol texture indices
    int m_hueTextureIndex{-1};
    int m_saturationTextureIndex{-1};
    int m_valueTextureIndex{-1};
    
    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void createHueSubmenus();
    void createSaturationValueSubmenus();
    void createKeySubmenus();

    // Color preview updates
    void updateColorPreviews();
    
    // Color modification callbacks
    void onHueSelected(int value);
    void onSaturationValueSelected(int value);
    void onKeySelected(int value);
};