// CharacterSelectionTool.cpp
#include "CharacterSelectionTool.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include "../GameBase.h"
#include "graphics/GraphicsEngine.h"
#include "../RadialMenu.h"
#include <iostream>

CharacterSelectionTool::CharacterSelectionTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
    : m_gameBase(gameBase)
    , m_radialMenu(radialMenu)
    , m_interactionRange(interactionRange)
{
    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }

    // Load icon texture
    m_playIconTextureIndex = m_gameBase->m_graphicsEngine->createInstanceTexture(m_radialMenu->getGeometry(), "../media/2d_graphics/08_play_icon.png");

    // Create menu node with toggle callback
    auto toggleCallback = [this]() {
        toggle();
    };
    
    // For character control, we use the same callback for activation and deactivation
    // This creates a toggle behavior instead of a submenu navigation
    m_characterSelectionNodeId = m_radialMenu->createNode(
        parentNodeId, 
        m_playIconTextureIndex, 
        toggleCallback,    // Activation callback
        toggleCallback     // Deactivation callback (same)
    );
    
    // Set initial node colors
    RadialMenuNode* node = m_radialMenu->getNode(m_characterSelectionNodeId);
    if (node) {
        node->m_selectColor = m_inactiveColor;    // Initial color when selected
        node->m_unSelectColor = m_inactiveColor;  // Initial color when unselected
        m_radialMenu->updateRendering();
    }
}

CharacterSelectionTool::~CharacterSelectionTool() {
    // Nothing special to clean up
}

void CharacterSelectionTool::activate() {
    m_active = true;
    
    // Update node colors to reflect active state
    RadialMenuNode* node = m_radialMenu->getNode(m_characterSelectionNodeId);
    if (node) {
        node->m_selectColor = m_activeColor;
        node->m_unSelectColor = m_activeColor;
        m_radialMenu->updateRendering();
    }
    
    std::cout << "Character control mode activated" << std::endl;
}

void CharacterSelectionTool::deactivate() {
    m_active = false;
    
    // Update node colors to reflect inactive state
    RadialMenuNode* node = m_radialMenu->getNode(m_characterSelectionNodeId);
    if (node) {
        node->m_selectColor = m_inactiveColor;
        node->m_unSelectColor = m_inactiveColor;
        m_radialMenu->updateRendering();
    }
    
    std::cout << "Character control mode deactivated" << std::endl;
}

void CharacterSelectionTool::toggle() {
    if (m_active) {
        deactivate();
    } else {
        activate();
    }
}

void CharacterSelectionTool::framePreRender(bool doToggle) {
    // Will be used for handling control input in the future
}

void CharacterSelectionTool::stepControl() {
    // Will be used for character control logic in the future
}