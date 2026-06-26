// BuildTool.cpp
#include "BuildTool.h"
#include "../GameBase.h"
#include "../RadialMenu.h"
#include "../Grid.h"
#include "../StructuralBlock.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/MeshManager2D/GeometryInstance.h"
#include <iostream>
#include "../../utils/GridGeometry.h"

BuildTool::BuildTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
    : m_gameBase(gameBase), m_radialMenu(radialMenu), m_interactionRange(interactionRange) {
    
    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }

    // Load construction icon texture
    m_constructionIconTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/2d_graphics/07_construction_icon.png");

    // Calculate crosshair offset and scale once in constructor
    m_crosshairScale = glm::dvec2(0.1, 0.1);
    // 16x16 pixels of a 64x64 image where the construction icon center is located
    m_crosshairOffset.x = 2.0 * (0.5 - 16.0/64.0) * m_crosshairScale.x;
    m_crosshairOffset.y = 2.0 * (0.5 - 16.0/64.0) * m_crosshairScale.y;

    // Create menu structure
    createMenuStructure(parentNodeId);

    // Create build crosshair using 2D mesh manager
    m_buildCrosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/2d_graphics/07_construction_icon.png", -1, true);
    if (auto geometry = m_buildCrosshairGeometry.lock()) {
        // Don't create instance yet - will be created when activated
    }
}

BuildTool::~BuildTool() {
    // Remove crosshair instance if it exists
    if (auto instance = m_buildCrosshairInstance.lock()) {
        if (auto geometry = m_buildCrosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
        }
    }
    
    // Textures are automatically cleaned up by TextureManagerBase destructor
}

void BuildTool::activate() {
    m_active = true;
    
    // Create and show build crosshair
    if (!m_buildCrosshairInstance.lock() && m_buildCrosshairGeometry.lock()) {
        m_buildCrosshairInstance = m_buildCrosshairGeometry.lock()->createInstance();
        if (auto instance = m_buildCrosshairInstance.lock()) {
            glm::vec2 position(static_cast<float>(m_crosshairOffset.x), static_cast<float>(-m_crosshairOffset.y)); // down-right direction
            instance->setPosition(position);
            instance->setScale(glm::vec2(static_cast<float>(m_crosshairScale.x), static_cast<float>(m_crosshairScale.y)));
            glm::dvec4 color(1.0, 1.0, 1.0, m_buildCrosshairTransparency);
            instance->setColor(color);
        }
    }
}

void BuildTool::deactivate() {
    m_active = false;

    // Remove build crosshair
    if (auto instance = m_buildCrosshairInstance.lock()) {
        if (auto geometry = m_buildCrosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
            m_buildCrosshairInstance.reset();
        }
    }
}

void BuildTool::preRenderCallback(bool doCreate, bool doRemove) {
    if (!m_active) {
        return;
    }
    
    if (doCreate) m_doCreate = true;
    if (doRemove) m_doRemove = true;
}

void BuildTool::onPhysicsUpdateComplete(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    if (!m_active) {
        return;
    }
    
    if (!m_doCreate && !m_doRemove) {
        return;
    }
    
    // Perform ray casting against all grids (copied from Creative::physics)
    std::weak_ptr<Grid> targetGridWeak;
    glm::ivec3 targetPos;
    glm::ivec3 hitPos;
    bool blockFound = false;
    double closestT = -1.0;
    
    // Camera position and direction
    glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 endPos = startPos + forward * m_interactionRange;

    // Get interpolation time for accurate raycasting
    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
    
    // Find closest ray intersection across all grids
    for (const auto& gridWeak : availableGrids) {
        auto gridShared = gridWeak.lock();
        if (!gridShared) continue;
        
        // Get interpolated transform once per grid
        glm::dvec3 interpolatedPos;
        glm::dquat interpolatedOri;
        gridShared->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);
        
        // Transform world ray to interpolated grid-local space
        glm::dvec3 gridLocalRayStart = GridGeometry::worldToGrid(startPos, interpolatedPos, interpolatedOri, gridShared->m_centerOfMass);
        glm::dvec3 gridLocalRayEnd = GridGeometry::worldToGrid(endPos, interpolatedPos, interpolatedOri, gridShared->m_centerOfMass);
        
        // Perform ray intersection in grid-local space
        RayIntersectionResult result = gridShared->intersectRay(gridLocalRayStart, gridLocalRayEnd);
        
        // Check if this is a closer hit than what we have so far
        if (result.t >= 0.0 && (!blockFound || result.t < closestT)) {
            closestT = result.t;
            blockFound = true;
            targetGridWeak = gridWeak;
            
            // Calculate intersection point with small epsilon to ensure we're inside the hit cell
            const double epsilon = 1e-6;
            double adjustedT = result.t + epsilon;
            glm::dvec3 gridLocalIntersectionPoint = gridLocalRayStart + adjustedT * (gridLocalRayEnd - gridLocalRayStart);
            
            // Floor to get hit cell (already in grid coordinates)
            hitPos = glm::ivec3(glm::floor(gridLocalIntersectionPoint));
            
            // Surface normal is already in grid-local space from the ray intersection
            glm::dvec3 gridNormal = result.surfaceNormal;
            
            // Find dominant axis direction
            glm::dvec3 absNormal = glm::abs(gridNormal);
            glm::ivec3 dominantAxis;
            if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z) {
                dominantAxis = glm::ivec3(gridNormal.x > 0 ? 1 : -1, 0, 0);
            } else if (absNormal.y >= absNormal.z) {
                dominantAxis = glm::ivec3(0, gridNormal.y > 0 ? 1 : -1, 0);
            } else {
                dominantAxis = glm::ivec3(0, 0, gridNormal.z > 0 ? 1 : -1);
            }

            // Calculate target position (one cell forward along dominant axis)
            targetPos = hitPos + dominantAxis;
        }
    }
    
    // Handle block creation
    if (m_doCreate) {
        if (blockFound) {
            auto targetGrid = targetGridWeak.lock();
            if (targetGrid) {
                // Place block at the position before the hit
                addGridBlock(targetGrid.get(), targetPos.x, targetPos.y, targetPos.z);
            }
        } else {
            // No block found, create a new grid 2 units ahead
            glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
            auto newGridWeak = m_gameBase->createGrid(newGridPos);
            Grid* newGrid = newGridWeak.lock().get();
            addGridBlock(newGrid, 0, 0, 0);  // Add initial block at grid center
        }
    }
    
    // Handle block removal
    if (m_doRemove) {
        if (blockFound) {
            auto targetGrid = targetGridWeak.lock();
            if (targetGrid) {
                // Remove the hit block
                removeGridBlock(targetGrid.get(), hitPos.x, hitPos.y, hitPos.z);
                
                // Handle grid splitting
                handleGridSplitting(targetGridWeak, hitPos);
                
                // Remove empty grids
                if (targetGrid->isEmpty()) {
                    m_gameBase->removeGrid(targetGridWeak);
                }
            }
        }
    }
    
    // Reset flags
    m_doCreate = false;
    m_doRemove = false;
}

void BuildTool::createMenuStructure(int64_t parentNodeId) {
    // Simple single node structure like ModifyTool
    auto activateCallback = [this]() { activate(); };
    auto deactivateCallback = [this]() { deactivate(); };
    
    // Create build tool parent node
    //glm::dvec4 selectColor = glm::dvec4(0.8, 0.6, 0.2, 0.5);    // Construction orange
    //glm::dvec4 unSelectColor = glm::dvec4(0.4, 0.3, 0.1, 0.5);  // Dark orange
    
    m_buildToolParentId = m_radialMenu->createNode(parentNodeId, m_constructionIconTextureIndex, activateCallback, deactivateCallback);
    
    // Add center node so you can navigate into the build tool
    m_centerNodeId = m_radialMenu->createNode(
        m_buildToolParentId, -1, activateCallback, deactivateCallback);
        
    m_radialMenu->createNode(
        m_buildToolParentId, m_constructionIconTextureIndex, activateCallback, deactivateCallback);
}

void BuildTool::addGridBlock(Grid* grid, int x, int y, int z) {
    if (grid) grid->addCell(glm::ivec3(x, y, z));
}

void BuildTool::removeGridBlock(Grid* grid, int x, int y, int z) {
    if (grid) grid->removeCell(glm::ivec3(x, y, z));
}

void BuildTool::handleGridSplitting(std::weak_ptr<Grid> targetGrid, const glm::ivec3& removedPos) {
    // Check for grid splits by testing connectivity of neighboring blocks
    std::vector<glm::ivec3> edgeCoords = {
        glm::ivec3(removedPos.x + 1, removedPos.y, removedPos.z),  // +X
        glm::ivec3(removedPos.x - 1, removedPos.y, removedPos.z),  // -X
        glm::ivec3(removedPos.x, removedPos.y + 1, removedPos.z),  // +Y
        glm::ivec3(removedPos.x, removedPos.y - 1, removedPos.z),  // -Y
        glm::ivec3(removedPos.x, removedPos.y, removedPos.z + 1),  // +Z
        glm::ivec3(removedPos.x, removedPos.y, removedPos.z - 1)   // -Z
    };
    
    // Schedule the grid split check for later processing
    m_gameBase->scheduleGridSplitCheck(targetGrid, edgeCoords);
}