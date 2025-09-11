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

    // Load symbol textures
    m_hueTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/radial_menu_graphics_generator/icon_hue_v2.png");
    m_saturationTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/radial_menu_graphics_generator/icon_saturation_v2.png");
    m_valueTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/radial_menu_graphics_generator/icon_value_v2.png");
    
    // Create menu structure with loaded textures
    
    createMenuStructure(parentNodeId);
    
    std::cout << "ColorTool: Created with menu structure" << std::endl;
}

ColorTool::~ColorTool() {
    // Release symbol textures
    if (m_hueTextureIndex >= 0) {
        m_gameBase->m_graphicsEngine->getInstanceHandler()->releaseTexture(m_hueTextureIndex);
    }
    if (m_saturationTextureIndex >= 0) {
        m_gameBase->m_graphicsEngine->getInstanceHandler()->releaseTexture(m_saturationTextureIndex);
    }
    if (m_valueTextureIndex >= 0) {
        m_gameBase->m_graphicsEngine->getInstanceHandler()->releaseTexture(m_valueTextureIndex);
    }
}

void ColorTool::activate() {
    m_active = true;
    std::cout << "ColorTool: Activated" << std::endl;
}

void ColorTool::deactivate() {
    m_active = false;
    std::cout << "ColorTool: Deactivated" << std::endl;
}

glm::dvec4 ColorTool::getCurrentColorRGBA() const {
    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);
    glm::dvec3 rgb = ColorUtils::hsvToRgb(hsv);
    return glm::dvec4(rgb.r, rgb.g, rgb.b, m_currentColor.w);
}

glm::dvec4 ColorTool::getCurrentColorHSVA() const {
    return m_currentColor;
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
    glm::dvec4 currentColor = getCurrentColorRGBA();
    currentColor.a = 1.0; // Ensure full opacity
    m_centerNodeId = m_radialMenu->createNode(
        m_colorToolParentId, -1, nullptr, currentColor, currentColor);
    
    // Level 2: Create 3 main categories
    glm::dvec4 graySelectColor = glm::dvec4(0.6, 0.6, 0.6, 0.5);
    glm::dvec4 grayUnSelectColor = glm::dvec4(0.3, 0.3, 0.3, 0.5);
    
    m_hueNodeId = m_radialMenu->createNode(m_colorToolParentId, m_hueTextureIndex, nullptr, 
                                          graySelectColor, grayUnSelectColor);
    m_saturationValueNodeId = m_radialMenu->createNode(m_colorToolParentId, m_saturationTextureIndex, nullptr,
                                                      graySelectColor, grayUnSelectColor);
    m_keyNodeId = m_radialMenu->createNode(m_colorToolParentId, m_valueTextureIndex, nullptr,
                                          graySelectColor, grayUnSelectColor);

    // Level 3: Create leaf nodes directly under each category
    createHueSubmenus();
    createSaturationValueSubmenus();
    createKeySubmenus();
}

void ColorTool::createHueSubmenus() {
    // Use pure colors for hue selection (saturation = 1.0, value = 1.0)
    
    // Add center node with current color (non-transparent)
    glm::dvec4 centerColor = getCurrentColorRGBA();
    centerColor.a = 1.0;
    m_radialMenu->createNode(m_hueNodeId, -1, nullptr, centerColor, centerColor);

    // Create 8 leaf nodes directly under Hue (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate hue preview color
        // Use pure colors: saturation = 1.0, value = 1.0
        glm::dvec3 previewHsv = glm::dvec3(static_cast<double>(value) / 8.0, 1.0, 1.0);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onHueSelected(value);
        };
        m_radialMenu->createNode(m_hueNodeId, -1, callback, selectColor, unSelectColor);
    }
}

void ColorTool::createSaturationValueSubmenus() {
    // Current color is already in HSV format
    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);

    // Add center node with current color (non-transparent)
    glm::dvec4 centerColor = getCurrentColorRGBA();
    centerColor.a = 1.0;
    m_radialMenu->createNode(m_saturationValueNodeId, -1, nullptr, centerColor, centerColor);

    // Create 8 leaf nodes directly under Saturation/Value (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate saturation preview color
        // Use current hue but pure value (1.0) to show saturation clearly
        glm::dvec3 previewHsv = glm::dvec3(hsv.x, static_cast<double>(value) / 7.0, 1.0);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onSaturationValueSelected(value);
        };
        m_radialMenu->createNode(m_saturationValueNodeId, -1, callback, selectColor, unSelectColor);
    }
}

void ColorTool::createKeySubmenus() {
    // Current color is already in HSV format
    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);
    
    // Add center node with current color (non-transparent)
    glm::dvec4 centerColor = getCurrentColorRGBA();
    centerColor.a = 1.0;
    m_radialMenu->createNode(m_keyNodeId, -1, nullptr, centerColor, centerColor);

    // Create 8 leaf nodes directly under Key (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate key/value preview color
        // Use current hue but pure saturation (1.0) to show value clearly
        glm::dvec3 previewHsv = glm::dvec3(hsv.x, 1.0, static_cast<double>(value) / 7.0);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onKeySelected(value);
        };
        m_radialMenu->createNode(m_keyNodeId, -1, callback, selectColor, unSelectColor);
    }
}

void ColorTool::updateColorPreviews() {
    // Remove all existing children from each submenu
    m_radialMenu->removeAllChildren(m_hueNodeId);
    m_radialMenu->removeAllChildren(m_saturationValueNodeId);
    m_radialMenu->removeAllChildren(m_keyNodeId);

    // Update the center node color
    glm::dvec4 currentColor = getCurrentColorRGBA();
    currentColor.a = 1.0;
    RadialMenuNode* centerNode = m_radialMenu->getNode(m_centerNodeId);
    if (centerNode) {
        centerNode->m_selectColor = currentColor;
        centerNode->m_unSelectColor = currentColor;
    }
    
    createHueSubmenus();
    createSaturationValueSubmenus();
    createKeySubmenus();
    m_radialMenu->updateRendering();
}

void ColorTool::onHueSelected(int value) {
    // Update hue directly in HSV storage
    double hue = static_cast<double>(value) / 8.0;
    
    m_currentColor.x = hue; // Update hue component

    updateColorPreviews();
    
    std::cout << "ColorTool: Hue selected - value: " << value << std::endl;
}

void ColorTool::onSaturationValueSelected(int value) {
    // Update saturation directly in HSV storage
    double saturation = static_cast<double>(value) / 7.0;
    
    m_currentColor.y = saturation; // Update saturation component

    updateColorPreviews();
    
    std::cout << "ColorTool: Saturation selected - value: " << value << std::endl;
}

void ColorTool::onKeySelected(int value) {
    // Update value/brightness directly in HSV storage
    double key = static_cast<double>(value) / 7.0;
    
    m_currentColor.z = key; // Update value component

    updateColorPreviews();
    
    std::cout << "ColorTool: Key selected - value: " << value << std::endl;
}