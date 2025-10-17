// CharacterControlTool.cpp
#include "CharacterControlTool.h"
#include "../game_base/GameBase.h"
#include "RadialMenu.h"
#include <iostream>

CharacterControlTool::CharacterControlTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
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
    m_playIconTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/08_play_icon.png");

    // Create menu node with toggle callback
    auto toggleCallback = [this]() {
        toggle();
    };
    
    // For character control, we use the same callback for activation and deactivation
    // This creates a toggle behavior instead of a submenu navigation
    m_characterControlNodeId = m_radialMenu->createNode(
        parentNodeId, 
        m_playIconTextureIndex, 
        toggleCallback,    // Activation callback
        toggleCallback     // Deactivation callback (same)
    );
    
    // Set initial node colors
    RadialMenuNode* node = m_radialMenu->getNode(m_characterControlNodeId);
    if (node) {
        node->m_selectColor = m_inactiveColor;    // Initial color when selected
        node->m_unSelectColor = m_inactiveColor;  // Initial color when unselected
        m_radialMenu->updateRendering();
    }
}

CharacterControlTool::~CharacterControlTool() {
    // Nothing special to clean up
}

void CharacterControlTool::activate() {
    m_active = true;
    
    // Update node colors to reflect active state
    RadialMenuNode* node = m_radialMenu->getNode(m_characterControlNodeId);
    if (node) {
        node->m_selectColor = m_activeColor;
        node->m_unSelectColor = m_activeColor;
        m_radialMenu->updateRendering();
    }
    
    std::cout << "Character control mode activated" << std::endl;
}

void CharacterControlTool::deactivate() {
    m_active = false;
    
    // Update node colors to reflect inactive state
    RadialMenuNode* node = m_radialMenu->getNode(m_characterControlNodeId);
    if (node) {
        node->m_selectColor = m_inactiveColor;
        node->m_unSelectColor = m_inactiveColor;
        m_radialMenu->updateRendering();
    }
    
    std::cout << "Character control mode deactivated" << std::endl;
}

void CharacterControlTool::toggle() {
    if (m_active) {
        deactivate();
    } else {
        activate();
    }
}

void CharacterControlTool::preRenderCallback(bool doToggle) {
    // Will be used for handling control input in the future
}

void CharacterControlTool::onPhysicsUpdateComplete() {
    // Will be used for character control logic in the future
}