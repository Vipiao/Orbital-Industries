// Creative.cpp
#include "Creative.h"
#include "../game_base/GameBase.h"
#include "../graphics/GraphicsEngine.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../game_base/Grid.h"
#include "../debug/DebugGlobals.h"
#include "../utils/PositionSelector.h"
#include <iostream>
#include "../utils/ColorUtils.h"
#include "../graphics/MeshManager2D.h"
#include "../graphics/GeometryInstance.h"
#include "StructuralBlock.h"
#include "../graphics/InstanceHandler.h"
#include <float.h>
#include <map>

Creative::Creative(GameBase* gameBase)
    : Mode(gameBase), m_hasSelectedBlock(false), m_cursorNearMarker(false), m_nearestMarkerIndex(-1) {
    
    // Load marker geometry using graphics engine's 2D mesh manager
    m_marker = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/01_marker.png", -1, true);
    std::cout << "Loaded marker geometry for configuration mode" << std::endl;

    // Load 3D arrow geometry and texture
    m_arrowGeometry = m_gameBase->m_graphicsEngine->getInstanceHandler()->createGeometry("../media/blender/04_arrow.obj");
    m_arrowTextureIndex = m_gameBase->m_graphicsEngine->getInstanceHandler()->createTexture("../media/debug_white_transparent.png");
    
    // Configure arrows for overlay rendering with transparency
    if (auto geometry = m_arrowGeometry.lock()) {
        geometry->setDepthCompression(0.1f);  // Compress depth range to render in front
        geometry->setAlphaBlending(true);     // Enable transparency
    }
    
    std::cout << "Loaded 3D arrow geometry and texture for configuration mode" << std::endl;
}

void Creative::processInputs() {
    processInputLogic();
}

void Creative::physics() {
    // Apply drag forces to all grids before physics update
    applyDragForces();
    
    if (doCreate || doRemove || doForce || doTrackSpeed || doConfigure || doModifyCell || doCopy || doPaste || doUpdateColor) {
        // Perform unified grid traversal for all actions
        std::weak_ptr<Grid> targetGridWeak;
        glm::ivec3 targetPos;
        glm::ivec3 hitPos;
        bool blockFound = false;
        double shortestSquaredDistance = DBL_MAX;
        
        // Camera position and direction
        glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 endPos = startPos + forward * 20.0; // Cast ray 20 units forward
        
        // Check all grids for ray intersections
        for (const auto& gridShared : m_gameBase->m_grids) {
            if (!gridShared) continue; // Safety check
            
            Grid* gridPtr = gridShared.get();
            
            // Convert camera ray to grid space
            glm::dvec3 startPosLocal = gridPtr->worldToGrid(startPos);
            glm::dvec3 endPosLocal = gridPtr->worldToGrid(endPos);
            
            // Perform grid traversal
            std::vector<glm::ivec3> gridCells = gridPtr->gridTraversal(startPosLocal, endPosLocal);
            
            // Check if ray hits any blocks in this grid
            for (size_t i = 1; i < gridCells.size(); i++) {
                if (gridPtr->hasCell(gridCells[i])) {
                    // Calculate center of hit cube in world space
                    glm::dvec3 hitCubeCenter = glm::dvec3(gridCells[i]) + glm::dvec3(0.5, 0.5, 0.5);
                    glm::dvec3 hitCubeCenterWorld = gridPtr->gridToWorld(hitCubeCenter);
                    
                    // Calculate squared distance from camera to cube center
                    glm::dvec3 distanceVec = hitCubeCenterWorld - startPos;
                    double squaredDistance = glm::dot(distanceVec, distanceVec);
                            
                    // Check if this is the closest hit so far
                    if (squaredDistance < shortestSquaredDistance) {
                        shortestSquaredDistance = squaredDistance;
                        targetGridWeak = gridShared;
                        targetPos = gridCells[i-1]; // Position before the hit (for placement)
                        hitPos = gridCells[i];      // Position of the hit block
                        blockFound = true;
                    }
                    break; // Only care about first hit in this grid
                }
            }
        }

        // Handle color operations with found target
        if (blockFound && (doCopy || doPaste || doUpdateColor)) {
            auto targetGrid = targetGridWeak.lock();
            if (targetGrid) {
                StructuralBlock* targetBlock = targetGrid->getCell(hitPos);
                if (targetBlock) {
                    if (doCopy) {
                        copiedColor = targetBlock->m_color;
                    }
                    if (doPaste) {
                        targetGrid->setColor(hitPos, copiedColor);
                    }
                    if (doUpdateColor) {
                        targetGrid->setColor(hitPos, color);
                    } else {
                        color = targetBlock->m_color; // Sync UI state
                    }
                }
            }
        }
        
        // Handle the different actions based on what was found
        if (doTrackSpeed) {
            if (blockFound) {
                auto targetGrid = targetGridWeak.lock();
                RigidBody* body = targetGrid ? targetGrid->getRigidBody() : nullptr;
                if (body) {
                    // Set camera velocity to match the rigid body's velocity
                    //m_cameraVelocity = body->m_velocity;
                }
            } else {
                // No target found, stop tracking
                //m_cameraVelocity = glm::dvec3(0.0, 0.0, 0.0);
                //std::cout << "No target found for speed tracking - camera velocity reset" << std::endl;
            }
        }

        if (doConfigure) {
            handleConfigureMode(blockFound, targetGridWeak, hitPos);
        }
        
        if (doForce) {
            if (blockFound) {
                auto targetGrid = targetGridWeak.lock();
                RigidBody* body = targetGrid ? targetGrid->getRigidBody() : nullptr;
                if (body) {
                    // Apply force in the view direction
                    const double forceStrength = 0.001 * body->m_mass * forceMultiplier;
                    glm::dvec3 force = forward * forceStrength;
                    
                    // Apply the force at the camera position
                    glm::dvec3 applicationPoint = m_gameBase->m_graphicsEngine->getCamPos();
                    
                    // Apply force at the point
                    m_gameBase->m_physicsEngine->applyForceAtPoint(body, force, applicationPoint);
                    
                    //std::cout << "Applied force to grid at distance: " << std::sqrt(shortestSquaredDistance) << std::endl;
                }
            } else {
                //std::cout << "No target found for force application" << std::endl;
            }
        }
        
        if (doCreate) {
            if (blockFound) {
                auto targetGrid = targetGridWeak.lock();
                if (targetGrid) {
                    // Place block at the position before the hit
                    addGridBlock(targetGrid.get(), targetPos.x, targetPos.y, targetPos.z);
                    //std::cout << "Added block at (" << targetPos.x << ", " << targetPos.y << ", " << targetPos.z << ")" << std::endl;
                }
            } else {
                // No block found, create a new grid 2 units ahead
                glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
                auto newGridWeak = m_gameBase->createGrid(newGridPos);
                Grid* newGrid = newGridWeak.lock().get();
                addGridBlock(newGrid, 0, 0, 0);  // Add initial block at grid center
                //std::cout << "Created new grid with block at world position (" 
                //        << newGridPos.x << ", " << newGridPos.y << ", " << newGridPos.z << ")" << std::endl;
            }
        }
        
        if (doRemove) {
            if (blockFound) {
                auto targetGrid = targetGridWeak.lock();
                if (targetGrid) {
                    // Remove the hit block
                    removeGridBlock(targetGrid.get(), hitPos.x, hitPos.y, hitPos.z);
                    //std::cout << "Removed block at (" << hitPos.x << ", " << hitPos.y << ", " << hitPos.z << ")" << std::endl;
                }

                // Check for grid splits by testing connectivity of neighboring blocks
                std::vector<glm::ivec3> edgeCoords = {
                    glm::ivec3(hitPos.x + 1, hitPos.y, hitPos.z),  // +X
                    glm::ivec3(hitPos.x - 1, hitPos.y, hitPos.z),  // -X
                    glm::ivec3(hitPos.x, hitPos.y + 1, hitPos.z),  // +Y
                    glm::ivec3(hitPos.x, hitPos.y - 1, hitPos.z),  // -Y
                    glm::ivec3(hitPos.x, hitPos.y, hitPos.z + 1),  // +Z
                    glm::ivec3(hitPos.x, hitPos.y, hitPos.z - 1)   // -Z
                };
                
                // Schedule the grid split check for later processing
                m_gameBase->scheduleGridSplitCheck(targetGridWeak, edgeCoords);
                //std::cout << "Scheduled grid split check for removed block at (" 
                //      << hitPos.x << ", " << hitPos.y << ", " << hitPos.z << ")" << std::endl;
                if (targetGrid->isEmpty()) {
                    m_gameBase->removeGrid(targetGridWeak);
                }
            } else {
                //std::cout << "No block found to remove" << std::endl;
            }
        }
        if (doModifyCell) {
            auto modGrid = m_modificationGrid.lock();
            if (modGrid && modGrid->canModifyCell(m_modificationCoord, m_modificationVertices)) {
                if (modGrid->modifyCell(m_modificationCoord, m_modificationVertices)) {
                    std::cout << "Successfully modified cell at (" << m_modificationCoord.x 
                              << ", " << m_modificationCoord.y << ", " << m_modificationCoord.z << ")" << std::endl;
                } else {
                    std::cout << "Failed to modify cell" << std::endl;
                }
            }
        }
    }

    // Reset flags
    doCopy = false;
    doPaste = false;
    doUpdateColor = false;

    doCreate = false;
    doRemove = false;
    doConfigure = false;
    doForce = false;
    doTrackSpeed = false;
    doModifyCell = false;
}

void Creative::addGridBlock(Grid* grid, int x, int y, int z) {
    if (grid) grid->addCell(glm::ivec3(x, y, z));
}

void Creative::removeGridBlock(Grid* grid, int x, int y, int z) {
    if (grid) grid->removeCell(glm::ivec3(x, y, z));
}

void Creative::applyDragForces() {
    // Apply drag to all objects before running physics
    for (const auto& gridShared : m_gameBase->m_grids) {
        if (!gridShared) continue;
        RigidBody* body = gridShared->getRigidBody();
        if (body && !body->m_isStatic && body->m_forces == glm::dvec3{0,0,0}) {
            // Simple drag force calculation: -dragCoefficient * velocity
            const double dragCoefficient = 0.04 * 0.4 * 2.0;
            
            // Apply drag to linear velocity
            if (glm::length(body->m_velocity) > 0.0) {
                glm::dvec3 dragForce = -dragCoefficient * body->m_velocity * body->m_mass;
                m_gameBase->m_physicsEngine->applyForce(body, dragForce);
            }
            
            // Apply drag to angular velocity
            if (glm::length(body->m_angularMomentumBody) > 0.0) {
                glm::dvec3 angularDrag = -dragCoefficient * body->getWorldInertiaTensor() * body->getAngularVelocityWorld();
                m_gameBase->m_physicsEngine->applyTorque(body, angularDrag);
            }
        }
    }
}

void Creative::handleConfigureMode(bool blockFound, std::weak_ptr<Grid> targetGridWeak, const glm::ivec3& hitPos) {
    if (!blockFound && !m_cursorNearMarker) {
        // No block found, unselect current block if any
        if (m_hasSelectedBlock) {
            m_hasSelectedBlock = false;
            m_selectedGrid.reset();
            std::cout << "Block unselected - no block found" << std::endl;
        }
        return;
    }
    
    auto targetGrid = targetGridWeak.lock();
    if (!targetGrid) {
        return;
    }
    
    // Check if this is the same block we already have selected
    bool isSameBlock = m_hasSelectedBlock && 
                      !m_selectedGrid.expired() &&
                      m_selectedGrid.lock().get() == targetGrid.get() &&
                      m_selectedBlockCoord == hitPos;
    
    if (!isSameBlock && !m_cursorNearMarker) {
        // New block selected
        m_hasSelectedBlock = true;
        m_selectedGrid = targetGridWeak;
        m_selectedBlockCoord = hitPos;
        std::cout << "Block selected at (" << hitPos.x << ", " << hitPos.y << ", " << hitPos.z << ")" << std::endl;
        return;
    }
    
    // Same block selected - check if cursor is near any marker (calculated in updateMarkerPositions)
    if (!m_cursorNearMarker) {
        // No marker nearby, unselect the block
        m_hasSelectedBlock = false;
        m_selectedGrid.reset();
        std::cout << "Block unselected" << std::endl;
    }
    // If cursor is near marker, the coordinate printing is handled in updateMarkerPositions
}

 glm::quat Creative::getArrowOrientation(const glm::ivec3& direction) {
    // Default arrow points in +X direction, calculate rotation to target direction
    // Use simple if-else chain for direction mapping
    if (direction == glm::ivec3(1, 0, 0)) {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // +X: identity
    } else if (direction == glm::ivec3(-1, 0, 0)) {
        return glm::angleAxis(glm::pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)); // -X: 180° around Z
    } else if (direction == glm::ivec3(0, 1, 0)) {
        return glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)); // +Y: 90° around Z
    } else if (direction == glm::ivec3(0, -1, 0)) {
        return glm::angleAxis(-glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)); // -Y: -90° around Z
    } else if (direction == glm::ivec3(0, 0, 1)) {
        return glm::angleAxis(-glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f)); // +Z: -90° around Y
    } else if (direction == glm::ivec3(0, 0, -1)) {
        return glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, 1.0f, 0.0f)); // -Z: 90° around Y
    } else {
        return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // Fallback: identity
     }
}

void Creative::updateMarkerPositions() {
    // Reset cursor proximity state
    m_cursorNearMarker = false;
    m_nearestMarkerIndex = -1;
    
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
                    geometry->removeInstance(inst.get());
                }
            }
        }
        m_markerInstances.clear();
        m_currentSelectedGridMeshId = -1;
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

    // Check if camera is too far from the selected block (5m threshold)
    glm::dvec3 blockWorldPos = selectedGrid->gridToWorld(glm::dvec3(m_selectedBlockCoord) + glm::dvec3(0.5, 0.5, 0.5));
    glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
    double distanceToBlock = glm::length(blockWorldPos - cameraPos);

    if (distanceToBlock > 10.0) {
        m_hasSelectedBlock = false;
        clearMarkers();
        return;
    }
    
    // Calculate corner positions using static default vertices (6 per corner = 48 total)
    std::vector<glm::dvec3> cornerPositions;
    std::vector<int> cornerIndexData;
    std::vector<glm::ivec3> directionData;

    // Check if grid mesh ID changed and recreate arrow instances if needed
    int gridMeshId = -1;
    gridMeshId = selectedGrid->getGraphicsMeshId();
    
    if (gridMeshId != m_currentSelectedGridMeshId) {
        // Clear existing arrow instances since mesh ID changed
        if (auto geometry = m_arrowGeometry.lock()) {
            for (auto& instance : m_arrowInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst);
                }
            }
        }
        m_arrowInstances.clear();
        m_currentSelectedGridMeshId = gridMeshId;
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
        glm::ivec3 defaultVertex = StructuralBlock::DEFAULT_VERTICES[cornerIndex];
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
                cornerPositions.push_back(selectedGrid->gridToWorld(corner));
                cornerIndexData.push_back(cornerIndex);
                directionData.push_back(unitDirections[i]);
            }
        }
    }

    // Calculate 3D arrow positions in grid local space
    std::vector<glm::vec3> arrowLocalPositions;
    std::vector<glm::quat> arrowOrientations;
    for (size_t i = 0; i < cornerPositions.size(); ++i) {
        glm::dvec3 localPos = glm::dvec3(m_selectedBlockCoord) + glm::dvec3(StructuralBlock::DEFAULT_VERTICES[cornerIndexData[i]]) / double(StructuralBlock::MAX_SIZE);
        arrowLocalPositions.push_back(glm::vec3(localPos));
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
        5 // Seperation iterations
    );
    
    // Check if cursor is near any marker and store calculated positions/scales
    struct MarkerData {
        glm::vec2 position;
        glm::vec2 scale;
    };
    std::vector<MarkerData> markerData;
    
    if (selectorResult.closestIndex >= 0 && selectorResult.distanceToClosest < 0.04) {
        m_cursorNearMarker = true;
        m_nearestMarkerIndex = selectorResult.closestIndex;

        //m_selectedMarkerCoordinate = glm::ivec3(cornerIndexData[m_nearestMarkerIndex], 0, 0); // Store corner index in x component
        //m_selectedMarkerDirection = directionData[m_nearestMarkerIndex];
        
        // Print the selected corner coordinate with index
        glm::dvec3 selectedCorner = cornerPositions[m_nearestMarkerIndex];
        std::cout << "Near corner " << m_nearestMarkerIndex << ": (" << selectedCorner.x << ", " 
                 << selectedCorner.y << ", " << selectedCorner.z << ")" << std::endl;
        // Check if R key is pressed to initiate modification
        KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->getKeyboardHandler();
        if (keyboard->m_r.justPressed()) {
            int cornerIndex = cornerIndexData[m_nearestMarkerIndex];
            glm::ivec3 direction = directionData[m_nearestMarkerIndex];
            
            // Since we only create markers for valid movements, we can directly apply the change
            std::array<glm::ivec3, 8> newVertices = currentVertices;
            newVertices[cornerIndex] += direction;
            
            // Store modification data for physics execution
            m_modificationGrid = selectedGrid;
            m_modificationCoord = m_selectedBlockCoord;
            m_modificationVertices = newVertices;
            doModifyCell = true;
        }
    }
    
    // Calculate data only for visible markers
    for (size_t i = 0; i < selectorResult.projectedPositions.size(); ++i) {
        glm::dvec2 screenPos = selectorResult.projectedPositions[i];
        
        // Only add markers that are not behind camera
        if (screenPos.x > -1.9 && screenPos.y > -1.9) {
            MarkerData data;
            data.position = glm::vec2(screenPos.x, screenPos.y);
            
            // Scale up if this is the nearest marker to cursor
            bool isNearestMarker = m_cursorNearMarker && static_cast<int>(i) == m_nearestMarkerIndex;
            float scale = isNearestMarker ? 0.035f : 0.02f;
            data.scale = glm::vec2(scale, scale);
            
            markerData.push_back(data);
        }
    }
    
    // Manage 3D arrow instances (always show all 48)
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
            auto newInstance = geometry->addInstance(gridMeshId, m_arrowTextureIndex, -1);
            m_arrowInstances.push_back(newInstance);
        }
    }
    
    // Update arrow positions and orientations
    for (size_t i = 0; i < m_arrowInstances.size() && i < arrowLocalPositions.size(); ++i) {
        if (auto inst = m_arrowInstances[i].lock()) {
            inst->m_localPosition = arrowLocalPositions[i];
            inst->m_localOrientation = arrowOrientations[i];
            inst->m_localScale = glm::vec3(0.1f); // Small arrow scale
            
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
                geometry->removeInstance(inst.get());
            }
        }
        m_markerInstances.pop_back();
    }
    
    // Add missing instances
    while (m_markerInstances.size() < needed) {
        if (auto geometry = m_marker.lock()) {
            auto newInstance = geometry->createInstance();
            m_markerInstances.push_back(newInstance);
        }
    }
    
    // Update positions and scales from calculated data
    for (size_t i = 0; i < m_markerInstances.size() && i < markerData.size(); ++i) {
        if (auto inst = m_markerInstances[i].lock()) {
            const MarkerData& data = markerData[i];
            inst->setPosition(data.position);
            inst->setScale(data.scale);
        }
    }
}

void Creative::processInputLogic() {
    MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->getMouseHandler();
    KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->getKeyboardHandler();

    // TEST START
    //CellMetadata* metadata = collider->get_pointer<CellMetadata>();
    for (size_t ii = 0; ii < m_gameBase->m_grids.size(); ii++) {
        auto cells = m_gameBase->m_grids[ii]->getCells();
        GridCollider* gridCollider = static_cast<GridCollider*>(m_gameBase->m_grids[ii]->getRigidBody()->m_collider);
        
        for (auto cell: cells) {
            glm::ivec3 coord = cell.first;
            
            // Get the collider for this cell
            Collider* cellCollider = gridCollider->getCell(coord);
            if (!cellCollider) continue;
            
            // Get the classification metadata
            CellMetadata* metadata = cellCollider->get_pointer<CellMetadata>();
            if (!metadata) continue;
            
            // Set color based on classification
            glm::dvec4 color;
            switch (metadata->classification) {
                case CellMetadata::CellClassification::INNER:
                    color = {1.0, 0.0, 0.0, 1.0}; // Red
                    break;
                case CellMetadata::CellClassification::FACE:
                    color = {0.0, 1.0, 0.0, 1.0}; // Green
                    break;
                case CellMetadata::CellClassification::EDGE:
                    color = {0.0, 0.0, 1.0, 1.0}; // Blue
                    break;
                case CellMetadata::CellClassification::CORNER:
                    color = {1.0, 1.0, 0.0, 1.0}; // Yellow
                    break;
                default:
                    color = {1.0, 1.0, 1.0, 1.0}; // White (fallback)
                    break;
            }
            
            m_gameBase->m_grids[ii]->setColor(coord, color);
        }
    }
    

    // TEST END
    
    handleColorInput();

    // Camera movement speed
    const double mouseSensitivity = 0.0014;
    
    // Calculate movement vectors based on camera orientation
    glm::dvec3 right = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 up = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 0.0, 1.0);

    // Update marker positions every frame for smooth 2D positioning
    updateMarkerPositions();

    // Structural analysis with G key
    if (keyboard->m_g.justPressed()) {
        //std::cout << "Visualizing structural analysis on " << m_gameBase->m_grids.size() << " grids..." << std::endl;
        
        for (const auto& gridShared : m_gameBase->m_grids) {
            if (gridShared) gridShared->visualizeStructuralIntegrity();
        }
    }
    
    // Check for input actions that require grid traversal
    // Set flags based on input (don't execute immediately)
    if (mouseHandler->rightClick() || (mouseHandler->getRightDown() && mouseHandler->getTimeRightDown() > 32)) {
        doCreate = true;
    }
    if (mouseHandler->leftClick() || (mouseHandler->getLeftDown() && mouseHandler->getTimeLeftDown() > 32)) {
        doRemove = true;
    }
    if (keyboard->m_f.isDown()) {
        doForce = true;
        forceMultiplier = (keyboard->m_f.timeDown() * 0.04 + 1.0);
    } else {
        forceMultiplier = 1.;
    }
    if (keyboard->m_z.justPressed()) {
        doTrackSpeed = true;
    }

    if (keyboard->m_r.justPressed()) {
        doConfigure = true;
    }

    // Toggle mouse lock with M key
    if (keyboard->m_m.justPressed()) {
        bool isLocked = mouseHandler->getMouseLock();
        mouseHandler->setMouseLock(!isLocked);
        std::cout << "Mouse " << (isLocked ? "unlocked" : "locked") << std::endl;
    }

    // Accelerate
    if (keyboard->m_c.isDown()) {
        m_moveSpeed *= 1.05;
    }
    if (keyboard->m_v.isDown()) {
        m_moveSpeed /= 1.05;
    }
    
    // Mouse look (camera rotation)
    if (mouseHandler->getMouseLock()) {
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        // Rotate around Z-axis for yaw (left/right)
        double yawAngle = -mouseMovement.x * mouseSensitivity;
        glm::dquat yawQuat = glm::angleAxis(yawAngle, glm::dvec3(0.0, 0.0, 1.0));
        
        // Rotate around X-axis for pitch (up/down)
        double pitchAngle = -mouseMovement.y * mouseSensitivity;
        glm::dquat pitchQuat = glm::angleAxis(pitchAngle, glm::dvec3(1.0, 0.0, 0.0));

        // Rotate around Y-axis for roll (roll right/roll left)
        const double rollSpeed = 0.01;
        double rollAngle = keyboard->m_q.isDown()?
            (keyboard->m_e.isDown()?
                0.: -rollSpeed):
            (keyboard->m_e.isDown()?
                rollSpeed: 0.);
        glm::dquat rollQuat = glm::angleAxis(rollAngle, glm::dvec3(0.0, 1.0, 0.0));
        
        // Apply rotations to camera orientation
        m_gameBase->m_graphicsEngine->getCamOri() = m_gameBase->m_graphicsEngine->getCamOri() * yawQuat * pitchQuat * rollQuat;
        m_gameBase->m_graphicsEngine->getCamOri() = glm::normalize(m_gameBase->m_graphicsEngine->getCamOri());
    }
    
    // Normalize the vectors
    right = glm::normalize(right);
    forward = glm::normalize(forward);
    up = glm::normalize(up);
    
    // Movement direction based on keyboard input
    glm::dvec3 moveDirection(0.0);
    
    if (keyboard->m_w.isDown()) {
        moveDirection += forward;
    }
    if (keyboard->m_s.isDown()) {
        moveDirection -= forward;
    }
    if (keyboard->m_a.isDown()) {
        moveDirection -= right;
    }
    if (keyboard->m_d.isDown()) {
        moveDirection += right;
    }
    if (keyboard->m_space.isDown()) {
        moveDirection += up;
    }
    if (keyboard->m_lShift.isDown()) {
        moveDirection -= up;
    }
    
    // Apply movement if any keys were pressed
    if (glm::length(moveDirection) > 0.0) {
        moveDirection = glm::normalize(moveDirection) * m_moveSpeed;
        m_gameBase->m_graphicsEngine->getCamPos() += moveDirection;
    }
}

void Creative::handleColorInput() {
    KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->getKeyboardHandler();
    MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->getMouseHandler();
    
    // Color management input handling
    int frameRate = m_gameBase->m_graphicsEngine->getFrameRate();
    double shortHoldThreshold = frameRate * 0.2; // 0.2 seconds
    
    // Handle T key for copy/paste operations
    if (keyboard->m_t.timeUp() == 0) { // Just released T
        if (keyboard->m_lCtrl.isDown() && keyboard->m_t.timeDown() < shortHoldThreshold) {
            // Shift+T brief hold = copy
            doCopy = true;
        } else if (!keyboard->m_lCtrl.isDown()) {
            // T brief press = paste
            doPaste = true;
        }
    }
    
    // Handle Ctrl+T long hold for color adjustment
    if (keyboard->m_t.isDown() && keyboard->m_lCtrl.isDown() &&
        keyboard->m_t.timeDown() > shortHoldThreshold) {
        
        // Get mouse movement and scroll for HSV adjustment
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        double scrollDelta = mouseHandler->getScrollMovement();
        
        // Convert any movement to HSV space and adjust
        if (mouseMovement.x != 0.0 || mouseMovement.y != 0.0 || scrollDelta != 0.0) {
            // Convert current color to HSV
            glm::dvec3 rgb = glm::dvec3(color.r, color.g, color.b);
            glm::dvec3 hsv = ColorUtils::rgbToHsv(rgb);
            
            // Adjust HSV based on mouse movement and scroll
            // Mouse up/down = hue, left/right = saturation, scroll = value
            double scale = 4.0;
            double hueSpeed = 0.002 * scale;
            double satSpeed = 0.003 * scale;
            double valSpeed = 0.002 * scale;
            
            hsv.x += mouseMovement.y * hueSpeed; // Up/down for hue
            hsv.y += mouseMovement.x * satSpeed; // Left/right for saturation
            hsv.z += scrollDelta * valSpeed;     // Scroll for value
            
            // Clamp values
            hsv.x = std::fmod(hsv.x + 1.0, 1.0); // Wrap hue around [0,1]
            hsv.y = glm::clamp(hsv.y, 0.0, 1.0);
            hsv.z = glm::clamp(hsv.z, 0.0, 1.0);
            
            // Convert back to RGB and update color
            rgb = ColorUtils::hsvToRgb(hsv);
            color = glm::dvec4(rgb.r, rgb.g, rgb.b, color.a);
            
            // Set flag to apply color
            doUpdateColor = true;
        }
    }
}