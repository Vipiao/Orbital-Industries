// ColorTool.cpp
#include "ColorTool.h"
#include "../game_base/GameBase.h"
#include "RadialMenu.h"
#include "../utils/ColorUtils.h"
#include <iostream>

ColorTool::ColorTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId)
    : m_gameBase(gameBase), m_radialMenu(radialMenu) {
    
    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }
    
    createMenuStructure(parentNodeId);
    
    std::cout << "ColorTool: Created with menu structure" << std::endl;
}

ColorTool::~ColorTool() {
    // Cleanup handled by RadialMenu
}

void ColorTool::activate() {
    m_active = true;
    std::cout << "ColorTool: Activated" << std::endl;
}

void ColorTool::deactivate() {
    m_active = false;
    std::cout << "ColorTool: Deactivated" << std::endl;
}

void ColorTool::preRenderCallback() {
    // TODO: Implement input processing
}

void ColorTool::onPhysicsUpdateComplete() {
    // TODO: Implement color application to world
}

void ColorTool::createMenuStructure(int64_t parentNodeId) {
    // Level 1: Create Color Tool Parent
    m_colorToolParentId = m_radialMenu->createNode(parentNodeId);
    
    // Add fake center node so the 3 categories appear as segments
    m_radialMenu->createNode(m_colorToolParentId);
    
    // Level 2: Create 3 main categories
    m_hueNodeId = m_radialMenu->createNode(m_colorToolParentId);
    m_saturationValueNodeId = m_radialMenu->createNode(m_colorToolParentId);
    m_keyNodeId = m_radialMenu->createNode(m_colorToolParentId);

    // Level 3: Create leaf nodes directly under each category
    createHueSubmenus();
    createSaturationValueSubmenus();
    createKeySubmenus();
}

void ColorTool::createHueSubmenus() {
    // Add fake center node that does nothing
    m_radialMenu->createNode(m_hueNodeId);

    // Create 8 leaf nodes directly under Hue (center node unused)
    for (int value = 0; value < 8; ++value) {
        auto callback = [this, value]() {
            onHueSelected(value);
        };
        m_radialMenu->createNode(m_hueNodeId, -1, callback);
    }
}

void ColorTool::createSaturationValueSubmenus() {
    // Add fake center node that does nothing
    m_radialMenu->createNode(m_saturationValueNodeId);

    // Create 8 leaf nodes directly under Saturation/Value (center node unused)
    for (int value = 0; value < 8; ++value) {
        auto callback = [this, value]() {
            onSaturationValueSelected(value);
        };
        m_radialMenu->createNode(m_saturationValueNodeId, -1, callback);
    }
}

void ColorTool::createKeySubmenus() {
    // Add fake center node that does nothing
    m_radialMenu->createNode(m_keyNodeId);

    // Create 8 leaf nodes directly under Key (center node unused)
    for (int value = 0; value < 8; ++value) {
        auto callback = [this, value]() {
            onKeySelected(value);
        };
        m_radialMenu->createNode(m_keyNodeId, -1, callback);
    }
}

void ColorTool::onHueSelected(int value) {
    // Calculate hue value based on value (0-7 maps to 0.0-1.0)
    double hue = static_cast<double>(value) / 7.0;
    
    // Convert current color to HSV, modify hue, convert back
    glm::dvec3 rgb = glm::dvec3(m_currentColor.r, m_currentColor.g, m_currentColor.b);
    glm::dvec3 hsv = ColorUtils::rgbToHsv(rgb);
    hsv.x = hue;
    rgb = ColorUtils::hsvToRgb(hsv);
    m_currentColor = glm::dvec4(rgb.r, rgb.g, rgb.b, m_currentColor.a);
    
    std::cout << "ColorTool: Hue selected - value: " << value << std::endl;
}

void ColorTool::onSaturationValueSelected(int value) {
    // Calculate saturation based on value (0-7 maps to 0.0-1.0)
    double saturation = static_cast<double>(value) / 7.0;
    
    // Convert current color to HSV, modify saturation, convert back
    glm::dvec3 rgb = glm::dvec3(m_currentColor.r, m_currentColor.g, m_currentColor.b);
    glm::dvec3 hsv = ColorUtils::rgbToHsv(rgb);
    hsv.y = saturation;
    rgb = ColorUtils::hsvToRgb(hsv);
    m_currentColor = glm::dvec4(rgb.r, rgb.g, rgb.b, m_currentColor.a);
    
    std::cout << "ColorTool: Saturation selected - value: " << value << std::endl;
}

void ColorTool::onKeySelected(int value) {
    // Calculate key/brightness value based on value (0-7 maps to 0.0-1.0)
    double key = static_cast<double>(value) / 7.0;
    
    // Apply key adjustment to current color
    glm::dvec3 rgb = glm::dvec3(m_currentColor.r, m_currentColor.g, m_currentColor.b);
    glm::dvec3 hsv = ColorUtils::rgbToHsv(rgb);
    hsv.z = key;
    rgb = ColorUtils::hsvToRgb(hsv);
    m_currentColor = glm::dvec4(rgb.r, rgb.g, rgb.b, m_currentColor.a);
    
    std::cout << "ColorTool: Key selected - value: " << value << std::endl;
}