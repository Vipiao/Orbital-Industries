// ColorTool.cpp
#include "ColorTool.h"
#include "../game_base/GameBase.h"
#include "RadialMenu.h"
#include "../game_base/Grid.h"
#include "StructuralBlock.h"
#include "../utils/ColorUtils.h"
#include <iostream>
#include "../graphics/MeshManager2D/MeshManager2D.h"
#include "../graphics/MeshManager2D/GeometryInstance.h"

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

    // Create paint crosshair using 2D mesh manager
    m_paintCrosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/03_crosshair_paint_v3.png", -1, true);
    if (auto geometry = m_paintCrosshairGeometry.lock()) {
        // Don't create instance yet - will be created when activated
    }
    
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
    // Create and show paint crosshair
    if (!m_paintCrosshairInstance.lock() && m_paintCrosshairGeometry.lock()) {
        m_paintCrosshairInstance = m_paintCrosshairGeometry.lock()->createInstance();
        if (auto instance = m_paintCrosshairInstance.lock()) {
            glm::vec2 scale(0.05f, 0.05f);
            // 11x11 pixels of a 64x64 image top right is the spray centre.
            double offset = 2.0 * (0.5 - 11.0/64.0) * static_cast<double>(scale.x);
            glm::vec2 position(static_cast<float>(offset), static_cast<float>(-offset)); // down-right direction
            instance->setPosition(position);
            instance->setScale(scale);
            instance->setColor(getCurrentColorRGBA());
        }
    }
    std::cout << "ColorTool: Activated" << std::endl;
}

void ColorTool::deactivate() {
    m_active = false;

    // Remove paint crosshair
    if (auto instance = m_paintCrosshairInstance.lock()) {
        if (auto geometry = m_paintCrosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
            m_paintCrosshairInstance.reset();
        }
    }
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

void ColorTool::preRenderCallback(bool doTryCopy, bool doTryPaste) {
    if (!m_active) {
        return;
    }
    
    if (!doTryCopy && !doTryPaste) {
        return;
    }
    
    m_doCopy = doTryCopy;
    m_doPaste = doTryPaste;
}

void ColorTool::onPhysicsUpdateComplete() {
    if (!m_active) {
        return;
    }
    
    if (!m_doCopy && !m_doPaste) {
        return;
    }
    
    // Perform ray casting against all grids
    std::weak_ptr<Grid> targetGridWeak;
    glm::ivec3 hitPos;
    bool blockFound = false;
    double closestT = -1.0;
    
    // Camera position and direction
    glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos = startPos + forward * 20.0; // Cast ray 20 units forward
    
    // Find closest ray intersection across all grids
    for (const auto& gridShared : m_gameBase->m_grids) {
        if (!gridShared) continue; // Safety check
        
        // Transform world ray to grid-local space
        glm::dvec3 gridLocalRayStart = gridShared->worldToGrid(startPos);
        glm::dvec3 gridLocalRayEnd = gridShared->worldToGrid(endPos);
        
        // Perform ray intersection in grid-local space
        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);
        
        // Check if this is a closer hit than what we have so far
        if (result.t >= 0.0 && (!blockFound || result.t < closestT)) {
            closestT = result.t;
            blockFound = true;
            targetGridWeak = gridShared;
            
            // Calculate intersection point with small epsilon to ensure we're inside the hit cell
            const double epsilon = 1e-6;
            double adjustedT = result.t + epsilon;
            glm::dvec3 gridLocalIntersectionPoint = gridLocalRayStart + adjustedT * (gridLocalRayEnd - gridLocalRayStart);
            
            // Floor to get hit cell (already in grid coordinates)
            hitPos = glm::ivec3(glm::floor(gridLocalIntersectionPoint));
        }
    }
    
    // Handle copy/paste operations with found target
    if (blockFound) {
        auto targetGrid = targetGridWeak.lock();
        if (targetGrid) {
            StructuralBlock* targetBlock = targetGrid->getCell(hitPos);
            if (targetBlock) {
                if (m_doCopy) {
                    // Convert RGB color to HSV and store in current color
                    glm::dvec3 rgb = glm::dvec3(targetBlock->m_color.r, targetBlock->m_color.g, targetBlock->m_color.b);
                    glm::dvec3 hsv = ColorUtils::rgbToHsv(rgb);
                    m_currentColor = glm::dvec4(hsv.x, hsv.y, hsv.z, targetBlock->m_color.a);
                    updateColorPreviews();
                }
                if (m_doPaste) {
                    // Convert current HSV color to RGB and apply to block
                    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);
                    glm::dvec3 rgb = ColorUtils::hsvToRgb(hsv);
                    targetGrid->setColor(hitPos, glm::dvec4(rgb.r, rgb.g, rgb.b, m_currentColor.w));
                }
            }
        }
    }
    
    // Reset flags
    m_doCopy = false;
    m_doPaste = false;
}

void ColorTool::createMenuStructure(int64_t parentNodeId) {
    // Level 1: Create Color Tool Parent
    auto activateCallback = [this]() { activate(); };
    auto deactivateCallback = [this]() { deactivate(); };
    
    m_colorToolParentId = m_radialMenu->createNode(parentNodeId, -1, activateCallback, deactivateCallback);
    
    // Add fake center node so the 3 categories appear as segments
    glm::dvec4 currentColor = getCurrentColorRGBA();
    currentColor.a = 1.0; // Ensure full opacity
    m_centerNodeId = m_radialMenu->createNode(
        m_colorToolParentId, -1, activateCallback, deactivateCallback, currentColor, currentColor);
    
    // Level 2: Create 3 main categories
    glm::dvec4 graySelectColor = glm::dvec4(0.6, 0.6, 0.6, 0.5);
    glm::dvec4 grayUnSelectColor = glm::dvec4(0.3, 0.3, 0.3, 0.5);
    
    m_hueNodeId = m_radialMenu->createNode(m_colorToolParentId, m_hueTextureIndex, activateCallback, deactivateCallback,
                                          graySelectColor, grayUnSelectColor);
    m_saturationValueNodeId = m_radialMenu->createNode(m_colorToolParentId, m_saturationTextureIndex, activateCallback, deactivateCallback,
                                                      graySelectColor, grayUnSelectColor);
    m_keyNodeId = m_radialMenu->createNode(m_colorToolParentId, m_valueTextureIndex, activateCallback, deactivateCallback,
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
    m_radialMenu->createNode(m_hueNodeId, -1, [this]() { activate(); }, [this]() { deactivate(); }, centerColor, centerColor);

    // Create 8 leaf nodes directly under Hue (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate hue preview color
        //// Use pure colors: saturation = 1.0, value = 1.0
        //glm::dvec3 previewHsv = glm::dvec3(static_cast<double>(value) / 8.0, 1.0, 1.0);
        // Use modified color
        glm::dvec3 previewHsv = glm::dvec3(static_cast<double>(value) / 8.0, m_currentColor.y, m_currentColor.z);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onHueSelected(value);
        };
        m_radialMenu->createNode(m_hueNodeId, -1, callback, [this]() { deactivate(); }, selectColor, unSelectColor);
    }
}

void ColorTool::createSaturationValueSubmenus() {
    // Current color is already in HSV format
    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);

    // Add center node with current color (non-transparent)
    glm::dvec4 centerColor = getCurrentColorRGBA();
    centerColor.a = 1.0;
    m_radialMenu->createNode(m_saturationValueNodeId, -1, [this]() { activate(); }, [this]() { deactivate(); }, centerColor, centerColor);

    // Create 8 leaf nodes directly under Saturation/Value (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate saturation preview color
        //// Use current hue but pure value (1.0) to show saturation clearly
        //glm::dvec3 previewHsv = glm::dvec3(hsv.x, static_cast<double>(value) / 7.0, 1.0);
        // Use modified color
        glm::dvec3 previewHsv = glm::dvec3(hsv.x, static_cast<double>(value) / 7.0, hsv.z);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onSaturationValueSelected(value);
        };
        m_radialMenu->createNode(m_saturationValueNodeId, -1, callback, [this]() { deactivate(); }, selectColor, unSelectColor);
    }
}

void ColorTool::createKeySubmenus() {
    // Current color is already in HSV format
    glm::dvec3 hsv = glm::dvec3(m_currentColor.x, m_currentColor.y, m_currentColor.z);
    
    // Add center node with current color (non-transparent)
    glm::dvec4 centerColor = getCurrentColorRGBA();
    centerColor.a = 1.0;
    m_radialMenu->createNode(m_keyNodeId, -1, [this]() { activate(); }, [this]() { deactivate(); }, centerColor, centerColor);

    // Create 8 leaf nodes directly under Key (center node unused)
    for (int value = 0; value < 8; ++value) {
        // Calculate key/value preview color
        //// Use current hue but pure saturation (1.0) to show value clearly
        //glm::dvec3 previewHsv = glm::dvec3(hsv.x, 1.0, static_cast<double>(value) / 7.0);
        // Use modified color
        glm::dvec3 previewHsv = glm::dvec3(hsv.x, hsv.y, static_cast<double>(value) / 7.0);
        glm::dvec3 previewRgb = ColorUtils::hsvToRgb(previewHsv);
        glm::dvec4 unSelectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);
        glm::dvec4 selectColor = glm::dvec4(previewRgb.r, previewRgb.g, previewRgb.b, 1.0);

        auto callback = [this, value]() {
            onKeySelected(value);
        };
        m_radialMenu->createNode(m_keyNodeId, -1, callback, [this]() { deactivate(); }, selectColor, unSelectColor);
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

    // Update paint crosshair color if it exists
    if (auto instance = m_paintCrosshairInstance.lock()) {
        glm::dvec4 currentColor = getCurrentColorRGBA();
        currentColor.a = 1.0; // Ensure full opacity for crosshair
        instance->setColor(currentColor);
    }
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