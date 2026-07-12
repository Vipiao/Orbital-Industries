// ModifyTool.cpp
#include "ModifyTool.h"
#include "graphics/GraphicsEngine.h"
#include "../GameBase.h"
#include "../RadialMenu.h"
#include "../Grid.h"
#include "../StructuralBlock.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/MeshManager2D/Instance2D.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include "utils/PositionSelector.h"
#include <cmath>
#include <iostream>
#include "utils/PolyhedronProcessor.h"
#include <algorithm>
#include "utils/GridGeometry.h"

ModifyTool::ModifyTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange)
    : m_gameBase(gameBase), m_radialMenu(radialMenu), m_interactionRange(interactionRange) {
    
    if (!m_gameBase) {
        throw std::runtime_error("GameBase cannot be null");
    }
    
    if (!m_radialMenu) {
        throw std::runtime_error("RadialMenu cannot be null");
    }
    
    // Load marker geometry using graphics engine's 2D mesh manager
    m_marker = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/2d_graphics/05_marker_v2.png", -1, true);

    // Load 3D arrow geometry; overlay so the markers draw on top of the scene
    m_arrowGeometry = m_gameBase->m_graphicsEngine->createInstanceGeometry(
        "../media/blender/04_arrow.obj", RenderLayer::Overlay);

    // Create modify crosshair using 2D mesh manager
    m_modifyCrosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/2d_graphics/04_crosshair_wrench.png", -1, true);
    
    // Calculate crosshair offset and scale once in constructor
    m_crosshairScale = glm::dvec2(0.05, 0.05);
    // 9x12 pixels of a 64x64 image where the wrench center is located

    // Load modify icon texture
    m_modifyIconTextureIndex = m_gameBase->m_graphicsEngine->createInstanceTexture("../media/2d_graphics/06_modify_icon.png");
    m_crosshairOffset.x = 2.0 * (0.5 - 9.0/64.0) * m_crosshairScale.x;
    m_crosshairOffset.y = 2.0 * (0.5 - 12.0/64.0) * m_crosshairScale.y;

    // Create menu structure
    createMenuStructure(parentNodeId);
}

ModifyTool::~ModifyTool() {
    // Textures are automatically cleaned up by TextureManagerBase destructor
}

void ModifyTool::activate() {
    m_active = true;
    std::cout << "ModifyTool activate" << std::endl;

    // Create and show modify crosshair
    if (!m_modifyCrosshairInstance.lock() && m_modifyCrosshairGeometry.lock()) {
        auto geometry = m_modifyCrosshairGeometry.lock();
        m_modifyCrosshairInstance = geometry->addInstance();
        if (auto instance = m_modifyCrosshairInstance.lock()) {
            instance->m_position = glm::dvec2(m_crosshairOffset.x, -m_crosshairOffset.y); // down-right direction
            instance->m_scale = m_crosshairScale;
            instance->m_color = glm::dvec4(1.0, 0.8, 0.2, m_modifyCrosshairTransparency); // Orange color for modify tool
            geometry->updateInstanceInBuffer(instance.get());
        }
    }
}

void ModifyTool::deactivate() {
    m_active = false;
    
    // Clear selection state when deactivating
    m_hasSelectedBlock = false;
    m_selectedGrid.reset();
    m_cursorNearMarker = false;
    m_nearestMarkerIndex = -1;
    
    // Clear marker instances
    if (auto geometry = m_marker.lock()) {
        for (auto& instance : m_markerInstances) {
            if (auto inst = instance.lock()) {
                geometry->removeInstance(inst);
            }
        }
    }
    m_markerInstances.clear();
    
    // Clear arrow instances
    if (auto geometry = m_arrowGeometry.lock()) {
        for (auto& instance : m_arrowInstances) {
            if (auto inst = instance.lock()) {
                geometry->removeInstance(inst);
            }
        }
    }
    m_arrowInstances.clear();
    m_currentSelectedGridSsboIndex = -1;

    // Remove modify crosshair
    if (auto instance = m_modifyCrosshairInstance.lock()) {
        if (auto geometry = m_modifyCrosshairGeometry.lock()) {
            geometry->removeInstance(instance);
            m_modifyCrosshairInstance.reset();
        }
    }

    std::cout << "ModifyTool deactivate" << std::endl;
}

void ModifyTool::framePreRender(bool doModify, bool doCancel) {
    if (!m_active) {
        return;
    }
    
    if (doModify) m_doModify = true;
    if (doCancel) m_doCancel = true;
    
    // Handle cancel - unselect block
    if (m_doCancel && m_hasSelectedBlock) {
        m_hasSelectedBlock = false;
        m_selectedGrid.reset();
        m_cursorNearMarker = false;
        m_nearestMarkerIndex = -1;
        
        // Clear marker instances
        if (auto geometry = m_marker.lock()) {
            for (auto& instance : m_markerInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_markerInstances.clear();
        
        // Clear arrow instances
        if (auto geometry = m_arrowGeometry.lock()) {
            for (auto& instance : m_arrowInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_arrowInstances.clear();
        m_currentSelectedGridSsboIndex = -1;
        return;
    }
    
    // Update marker positions (same logic as Creative::updateMarkerPositions)
    updateMarkerPositions();

    // Update wrench animation angles
    int frameRate = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime = 1.0 / static_cast<double>(frameRate);
    // Target angle lerps toward 0 at 2% per frame (small decay)
    //m_targetAngle = m_targetAngle * 0.95;
    m_targetAngle = m_targetAngle * glm::exp(-8 * deltaTime);
    
    // Current angle lerps toward target angle at 5% per frame (faster follow)
    //m_currentAngle = m_currentAngle + (m_targetAngle - m_currentAngle) * 0.2;
    m_currentAngle = m_targetAngle + (m_currentAngle - m_targetAngle) *
        glm::exp(-36 * deltaTime);
    
    // Update wrench crosshair rotation if active
    if (auto instance = m_modifyCrosshairInstance.lock()) {
        instance->m_orientation = m_currentAngle;

        // Rotate offset vector around tip point
        double cosAngle = glm::cos(m_currentAngle);
        double sinAngle = glm::sin(m_currentAngle);
        double rotatedOffsetX = m_crosshairOffset.x * cosAngle - (-m_crosshairOffset.y) * sinAngle;
        double rotatedOffsetY = m_crosshairOffset.x * sinAngle + (-m_crosshairOffset.y) * cosAngle;

        instance->m_position = glm::dvec2(rotatedOffsetX, rotatedOffsetY);

        if (auto geometry = m_modifyCrosshairGeometry.lock()) {
            geometry->updateInstanceInBuffer(instance.get());
        }
    }
}

void ModifyTool::stepControl(const std::vector<std::weak_ptr<Grid>>& availableGrids) {
    if (!m_active || !m_doModify) {
        m_doModify = false;
        m_doCancel = false;
        return;
    }
    
    
    // Handle configure mode logic (similar to Creative::handleConfigureMode)
    // Perform ray casting against all grids
    std::weak_ptr<Grid> targetGridWeak;
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
        }
    }
    
    if (!blockFound && !m_cursorNearMarker) {
        // No block found and not near marker, but don't unselect unless cancelled
        m_doModify = false;
        m_doCancel = false;
        return;
    }
    
    auto targetGrid = targetGridWeak.lock();
    if (!targetGrid && !m_cursorNearMarker) {
        m_doModify = false;
        m_doCancel = false;
        return;
    }
    
    // Check if this is the same block we already have selected
    bool isSameBlock = m_hasSelectedBlock && 
                      !m_selectedGrid.expired() &&
                      m_selectedGrid.lock().get() == targetGrid.get() &&
                      m_selectedBlockCoord == hitPos;
    
    if (!isSameBlock && !m_cursorNearMarker && blockFound) {
        // New block selected
        m_hasSelectedBlock = true;
        m_selectedGrid = targetGridWeak;
        m_selectedBlockCoord = hitPos;
        m_doModify = false;
        m_doCancel = false;
        return;
    }
    
    // Handle modification when doModify is true and cursor is near marker
    if (m_doModify && m_cursorNearMarker && m_hasSelectedBlock) {
        auto selectedGrid = m_selectedGrid.lock();
        if (selectedGrid && selectedGrid->hasCell(m_selectedBlockCoord)) {
            // Get current vertices from the selected block
            const auto& cells = selectedGrid->getCells();
            auto cellIt = cells.find(m_selectedBlockCoord);
            if (cellIt != cells.end()) {
                std::array<glm::ivec3, 8> currentVertices = cellIt->second.m_localVertices;
                
                // Calculate modification based on nearest marker
                // This logic should match what was in Creative::updateMarkerPositions
                std::vector<glm::dvec3> cornerPositions;
                std::vector<int> cornerIndexData;
                std::vector<glm::ivec3> directionData;
                
                // Generate corner positions and directions (same as Creative logic)
                const double offset = 0.3;
                for (int cornerIndex = 0; cornerIndex < 8; ++cornerIndex) {
                    glm::ivec3 defaultVertex = PolyhedronProcessor::DEFAULT_VERTICES[cornerIndex];
                    glm::dvec3 normalizedVertex = glm::dvec3(defaultVertex) / double(StructuralBlock::MAX_SIZE);
                    
                    glm::ivec3 unitDirections[6] = {
                        glm::ivec3(1, 0, 0), glm::ivec3(-1, 0, 0),
                        glm::ivec3(0, 1, 0), glm::ivec3(0, -1, 0),
                        glm::ivec3(0, 0, 1), glm::ivec3(0, 0, -1)
                    };
                    
                    for (int i = 0; i < 6; ++i) {
                        glm::ivec3 currentVertex = currentVertices[cornerIndex];
                        glm::ivec3 newVertex = currentVertex + unitDirections[i];
                        
                        if (newVertex.x >= 0 && newVertex.x <= StructuralBlock::MAX_SIZE &&
                            newVertex.y >= 0 && newVertex.y <= StructuralBlock::MAX_SIZE &&
                            newVertex.z >= 0 && newVertex.z <= StructuralBlock::MAX_SIZE) {
                            glm::dvec3 scaledDirection = glm::dvec3(unitDirections[i]) * offset;
                            glm::dvec3 corner = glm::dvec3(m_selectedBlockCoord) + normalizedVertex + scaledDirection;
                            cornerPositions.push_back(selectedGrid->gridToWorld(corner));
                            cornerIndexData.push_back(cornerIndex);
                            directionData.push_back(unitDirections[i]);
                        }
                    }
                }
                
                // Apply the modification if we have a valid nearest marker
                if (m_nearestMarkerIndex >= 0 && m_nearestMarkerIndex < static_cast<int>(cornerIndexData.size())) {
                    int cornerIndex = cornerIndexData[m_nearestMarkerIndex];
                    glm::ivec3 direction = directionData[m_nearestMarkerIndex];
                    
                    std::array<glm::ivec3, 8> newVertices = currentVertices;
                    newVertices[cornerIndex] += direction;
                    
                    // Store modification data for execution
                    m_modificationGrid = selectedGrid;
                    m_modificationCoord = m_selectedBlockCoord;
                    m_modificationVertices = newVertices;
                    
                    // Execute modification
                    if (selectedGrid->canModifyCell(m_modificationCoord, m_modificationVertices)) {
                        m_targetAngle = glm::radians(90.);
                        if (selectedGrid->modifyCell(m_modificationCoord, m_modificationVertices)) {
                            // Schedule grid split check
                            std::vector<glm::ivec3> edgeCoords = {
                                glm::ivec3(m_modificationCoord.x, m_modificationCoord.y, m_modificationCoord.z),
                                glm::ivec3(m_modificationCoord.x + 1, m_modificationCoord.y, m_modificationCoord.z),
                                glm::ivec3(m_modificationCoord.x - 1, m_modificationCoord.y, m_modificationCoord.z),
                                glm::ivec3(m_modificationCoord.x, m_modificationCoord.y + 1, m_modificationCoord.z),
                                glm::ivec3(m_modificationCoord.x, m_modificationCoord.y - 1, m_modificationCoord.z),
                                glm::ivec3(m_modificationCoord.x, m_modificationCoord.y, m_modificationCoord.z + 1),
                                glm::ivec3(m_modificationCoord.x, m_modificationCoord.y, m_modificationCoord.z - 1)
                            };
                            
                            m_gameBase->scheduleGridSplitCheck(selectedGrid, edgeCoords);
                            
                            std::cout << "Successfully modified cell at (" << m_modificationCoord.x 
                                      << ", " << m_modificationCoord.y << ", " << m_modificationCoord.z << ")" << std::endl;
                        } else {
                            std::cout << "Failed to modify cell" << std::endl;
                        }
                    }
                }
            }
        }
    }
    m_doModify = false;
    m_doCancel = false;
}

void ModifyTool::createMenuStructure(int64_t parentNodeId) {
    // Simple single node structure
    auto activateCallback = [this]() { activate(); };
    auto deactivateCallback = [this]() { deactivate(); };
    
    // Create modify tool parent node
    m_modifyToolParentId = m_radialMenu->createNode(parentNodeId, m_modifyIconTextureIndex, activateCallback, deactivateCallback);
    
    // Add center node so you can navigate into the modify tool
    // Use orange color scheme to indicate modify tool
    //glm::dvec4 centerSelectColor = glm::dvec4(0.9, 0.5, 0.3, 1.0);   // Bright orange, opaque
    //glm::dvec4 centerUnSelectColor = glm::dvec4(0.8, 0.4, 0.2, 1.0); // Orange, opaque
    
    m_centerNodeId = m_radialMenu->createNode(
        m_modifyToolParentId, -1, activateCallback, deactivateCallback);
    //    centerSelectColor, centerUnSelectColor);
    
    m_radialMenu->createNode(
        m_modifyToolParentId, m_modifyIconTextureIndex, activateCallback, deactivateCallback);
    //    centerSelectColor, centerUnSelectColor);
}

glm::dquat ModifyTool::getArrowOrientation(const glm::ivec3& direction) {
    // Default arrow points in +X direction, calculate rotation to target direction
    if (direction == glm::ivec3(1, 0, 0)) {
        return glm::dquat(1.0, 0.0, 0.0, 0.0); // +X: identity
    } else if (direction == glm::ivec3(-1, 0, 0)) {
        return glm::angleAxis(glm::pi<double>(), glm::dvec3(0.0, 0.0, 1.0)); // -X: 180° around Z
    } else if (direction == glm::ivec3(0, 1, 0)) {
        return glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0)); // +Y: 90° around Z
    } else if (direction == glm::ivec3(0, -1, 0)) {
        return glm::angleAxis(-glm::half_pi<double>(), glm::dvec3(0.0, 0.0, 1.0)); // -Y: -90° around Z
    } else if (direction == glm::ivec3(0, 0, 1)) {
        return glm::angleAxis(-glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0)); // +Z: -90° around Y
    } else if (direction == glm::ivec3(0, 0, -1)) {
        return glm::angleAxis(glm::half_pi<double>(), glm::dvec3(0.0, 1.0, 0.0)); // -Z: 90° around Y
    } else {
        return glm::dquat(1.0, 0.0, 0.0, 0.0); // Fallback: identity
    }
}

void ModifyTool::updateMarkerPositions() {
    // This method contains the core logic from Creative::updateMarkerPositions
    // but adapted for the tool pattern
    
    // Reset cursor proximity state
    m_cursorNearMarker = false;
    m_nearestMarkerIndex = -1;

    // Get interpolation parameters for accurate rendering
    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
    
    // Helper lambda to clear markers
    auto clearMarkers = [this]() {
        // Clear 3D arrow instances
        if (auto geometry = m_arrowGeometry.lock()) {
            for (auto& instance : m_arrowInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_arrowInstances.clear();

        if (auto geometry = m_marker.lock()) {
            for (auto& instance : m_markerInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_markerInstances.clear();
        m_currentSelectedGridSsboIndex = -1;
    };
    
    if (!m_hasSelectedBlock) {
        clearMarkers();
        return;
    }

    auto selectedGrid = m_selectedGrid.lock();
    if (!selectedGrid || !selectedGrid->hasCell(m_selectedBlockCoord)) {
        m_hasSelectedBlock = false;
        clearMarkers();
        return;
    }

    // Get interpolated transform for accurate marker positioning
    glm::dvec3 interpolatedPos;
    glm::dquat interpolatedOri;
    selectedGrid->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);
    
    // Check if camera is too far from the selected block
    glm::dvec3 blockGridCenter = glm::dvec3(m_selectedBlockCoord) + glm::dvec3(0.5, 0.5, 0.5);
    glm::dvec3 blockWorldPos = GridGeometry::gridToWorld(blockGridCenter, interpolatedPos, 
                                                         interpolatedOri, selectedGrid->m_centerOfMass);
    glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
    double distanceToBlock = glm::length(blockWorldPos - cameraPos);
    if (distanceToBlock > m_interactionRange) {
        m_hasSelectedBlock = false;
        clearMarkers();
        return;
    }
    
    // Calculate corner positions using static default vertices (6 per corner = 48 total)
    std::vector<glm::dvec3> cornerPositions;
    std::vector<int> cornerIndexData;
    std::vector<glm::ivec3> directionData;

    // Check if grid SSBO index changed and recreate arrow instances if needed
    int ssboIndex = selectedGrid->getGridSSBOIndex();

    if (ssboIndex != m_currentSelectedGridSsboIndex) {
        // Clear existing arrow instances since SSBO index changed
        if (auto geometry = m_arrowGeometry.lock()) {
            for (auto& instance : m_arrowInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_arrowInstances.clear();
        m_currentSelectedGridSsboIndex = ssboIndex;
    }
    
    // Get current vertices from the selected block for bounds checking
    std::array<glm::ivec3, 8> currentVertices;
    const auto& cells = selectedGrid->getCells();
    auto cellIt = cells.find(m_selectedBlockCoord);
    if (cellIt == cells.end()) {
        clearMarkers();
        return;
    }
    currentVertices = cellIt->second.m_localVertices;
    const double offset = 0.3;
    
    // Iterate through the 8 default vertices
    for (int cornerIndex = 0; cornerIndex < 8; ++cornerIndex) {
        glm::ivec3 defaultVertex = PolyhedronProcessor::DEFAULT_VERTICES[cornerIndex];
        glm::dvec3 normalizedVertex = glm::dvec3(defaultVertex) / double(StructuralBlock::MAX_SIZE);
        
        // Generate all 6 cardinal directions for this corner
        glm::ivec3 unitDirections[6] = {
            glm::ivec3(1, 0, 0),   // +X
            glm::ivec3(-1, 0, 0),  // -X
            glm::ivec3(0, 1, 0),   // +Y
            glm::ivec3(0, -1, 0),  // -Y
            glm::ivec3(0, 0, 1),   // +Z
            glm::ivec3(0, 0, -1)   // -Z
        };
        
        // Generate 6 positions per corner
        for (int i = 0; i < 6; ++i) {
            // Check if moving this vertex in this direction would be within bounds
            glm::ivec3 currentVertex = currentVertices[cornerIndex];
            glm::ivec3 newVertex = currentVertex + unitDirections[i];
            
            // Only create marker if the movement would be valid
            if (newVertex.x >= 0 && newVertex.x <= StructuralBlock::MAX_SIZE &&
                newVertex.y >= 0 && newVertex.y <= StructuralBlock::MAX_SIZE &&
                newVertex.z >= 0 && newVertex.z <= StructuralBlock::MAX_SIZE) {
                glm::dvec3 scaledDirection = glm::dvec3(unitDirections[i]) * offset;
                glm::dvec3 corner = glm::dvec3(m_selectedBlockCoord) + normalizedVertex + scaledDirection;
                cornerPositions.push_back(GridGeometry::gridToWorld(corner, interpolatedPos, 
                                                                    interpolatedOri, selectedGrid->m_centerOfMass));
                cornerIndexData.push_back(cornerIndex);
                directionData.push_back(unitDirections[i]);
            }
        }
    }

    // Calculate 3D arrow positions in grid local space
    std::vector<glm::dvec3> arrowLocalPositions;
    std::vector<glm::dquat> arrowOrientations;
    for (size_t i = 0; i < cornerPositions.size(); ++i) {
        glm::dvec3 localPos = glm::dvec3(m_selectedBlockCoord) + glm::dvec3(PolyhedronProcessor::DEFAULT_VERTICES[cornerIndexData[i]]) / double(StructuralBlock::MAX_SIZE);
        arrowLocalPositions.push_back(localPos);
        arrowOrientations.push_back(getArrowOrientation(directionData[i]));
    }
    
    // Get positions and scales of all markers (now 48 total)
    auto selectorResult = PositionSelector::selectFromPositions(
        cornerPositions,
        0.004, // Small projected radius. (How far is minimum distance)
        m_gameBase->m_graphicsEngine->getCamPos(),
        m_gameBase->m_graphicsEngine->getCamOri(),
        m_gameBase->m_graphicsEngine->getFieldOfView(),
        static_cast<double>(m_gameBase->m_graphicsEngine->getScreenWidth()) /
        static_cast<double>(m_gameBase->m_graphicsEngine->getScreenHeight()),
        glm::dvec2(0.0, 0.0), // Screen center as cursor position
        5, // Separation iterations
        m_gameBase->m_graphicsEngine->getPaniniHorizontal(),
        m_gameBase->m_graphicsEngine->getPaniniVertical(),
        m_gameBase->m_graphicsEngine->getPaniniFitScale()
    );
    
    // Calculate distances for dynamic transparency and track min/max
    std::vector<double> markerDistances;
    std::vector<double> arrowDistances;
    double minDist = 1e9;  // Start with large value
    double maxDist = -1e9; // Start with small value
    
    // Calculate distances for 2D markers (only for visible ones)
    for (size_t i = 0; i < selectorResult.projectedPositions.size(); ++i) {
        glm::dvec2 screenPos = selectorResult.projectedPositions[i];
        if (screenPos.x > -1.9 && screenPos.y > -1.9) {
            double dist = glm::length(cornerPositions[i] - cameraPos);
            markerDistances.push_back(dist);
            minDist = glm::min(minDist, dist);
            maxDist = glm::max(maxDist, dist);
        }
    }
    
    // Calculate distances for 3D arrows (convert local to world space)
    for (size_t i = 0; i < arrowLocalPositions.size(); ++i) {
        glm::dvec3 worldPos = GridGeometry::gridToWorld(arrowLocalPositions[i], interpolatedPos,
                                                        interpolatedOri, selectedGrid->m_centerOfMass);
        double dist = glm::length(worldPos - cameraPos);
        arrowDistances.push_back(dist);
        minDist = glm::min(minDist, dist);
        maxDist = glm::max(maxDist, dist);
    }
    
    // Handle edge case where all distances are the same or no elements
    if (maxDist - minDist < 1e-6) { minDist = maxDist - 1e-6; }

    struct MarkerData {
        glm::dvec2 position;
        glm::dvec2 scale;
    };
    std::vector<MarkerData> markerData;
    
    if (selectorResult.closestIndex >= 0 && selectorResult.distanceToClosest < 0.04) {
        m_cursorNearMarker = true;
        m_nearestMarkerIndex = selectorResult.closestIndex;
        
        // Store selected marker data for potential modification
        m_selectedMarkerCoordinate = glm::ivec3(cornerIndexData[m_nearestMarkerIndex], 0, 0); // Store corner index in x component
        m_selectedMarkerDirection = glm::dvec3(directionData[m_nearestMarkerIndex]);
    }
    
    // Calculate data only for visible markers
    for (size_t i = 0; i < selectorResult.projectedPositions.size(); ++i) {
        glm::dvec2 screenPos = selectorResult.projectedPositions[i];
        
        // Only add markers that are not behind camera
        if (screenPos.x > -1.9 && screenPos.y > -1.9) {
            MarkerData data;
            data.position = screenPos;

            // Scale up if this is the nearest marker to cursor
            bool isNearestMarker = m_cursorNearMarker && static_cast<int>(i) == m_nearestMarkerIndex;
            double scale = isNearestMarker ? 0.024 : 0.016;
            data.scale = glm::dvec2(scale, scale);
            
            markerData.push_back(data);
        }
    }
    
    // Manage 3D arrow instances (always show all valid arrows)
    size_t neededArrows = arrowLocalPositions.size();
    
    // Remove excess arrow instances
    while (m_arrowInstances.size() > neededArrows) {
        auto instance = m_arrowInstances.back();
        if (auto inst = instance.lock()) {
            if (auto geometry = m_arrowGeometry.lock()) {
                geometry->removeInstance(inst);
            }
        }
        m_arrowInstances.pop_back();
    }
    
    // Add missing arrow instances
    while (m_arrowInstances.size() < neededArrows) {
        if (auto geometry = m_arrowGeometry.lock()) {
            auto newInstance = geometry->addInstance(ssboIndex, -1, -1);
            m_arrowInstances.push_back(newInstance);
        }
    }
    
    // Update arrow positions and orientations
    for (size_t i = 0; i < m_arrowInstances.size() && i < arrowLocalPositions.size(); ++i) {
        if (auto inst = m_arrowInstances[i].lock()) {
            inst->m_localPosition = arrowLocalPositions[i];
            inst->m_localOrientation = arrowOrientations[i];
            inst->m_localScale = glm::dvec3(0.1); // Small arrow scale

            // Calculate dynamic alpha for 3D arrows
            double alpha = 0.5; // default
            if (i < arrowDistances.size()) {
                alpha = 0.25 + 0.5 * (maxDist - arrowDistances[i]) / (maxDist - minDist);
            }
            inst->m_color = glm::dvec4(1.0, 1.0, 1.0, alpha);
            
            // Update the instance buffer
            if (auto geometry = m_arrowGeometry.lock()) {
                geometry->updateInstanceInBuffer(inst.get());
            }
        }
    }

    // Adjust instance count to match needed markers
    size_t needed = markerData.size();
    
    // Remove excess instances
    while (m_markerInstances.size() > needed) {
        auto instance = m_markerInstances.back();
        if (auto inst = instance.lock()) {
            if (auto geometry = m_marker.lock()) {
                geometry->removeInstance(inst);
            }
        }
        m_markerInstances.pop_back();
    }
    
    // Add missing instances
    while (m_markerInstances.size() < needed) {
        if (auto geometry = m_marker.lock()) {
            auto newInstance = geometry->addInstance();
            m_markerInstances.push_back(newInstance);
        }
    }
    
    // Update positions and scales from calculated data
    for (size_t i = 0; i < m_markerInstances.size() && i < markerData.size(); ++i) {
        if (auto inst = m_markerInstances[i].lock()) {
            const MarkerData& data = markerData[i];
            inst->m_position = data.position;
            inst->m_scale = data.scale;

            // Calculate dynamic alpha for 2D markers
            double alpha = 0.5; // default
            if (i < markerDistances.size()) {
                alpha = 0.25 + 0.5 * (maxDist - markerDistances[i]) / (maxDist - minDist);
            }
            inst->m_color = glm::dvec4(1.0, 1.0, 1.0, alpha);

            // Update the instance buffer
            if (auto geometry = m_marker.lock()) {
                geometry->updateInstanceInBuffer(inst.get());
            }
        }
    }
}