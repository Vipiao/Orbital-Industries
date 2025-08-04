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
#include <float.h>

Creative::Creative(GameBase* gameBase, MeshManager2D* meshManager) 
    : Mode(gameBase), m_meshManager(meshManager), m_hasSelectedBlock(false), 
      m_cursorNearMarker(false), m_nearestMarkerIndex(-1) {
    
    if (!m_meshManager) {
        throw std::runtime_error("MeshManager2D cannot be null");
    }
    
    // Load marker geometry
    m_marker = m_meshManager->loadMesh("../media/blender/03_face.obj", "../media/01_marker.png", -1, true);
    std::cout << "Loaded marker geometry for configuration mode" << std::endl;
}

void Creative::processInputs() {
    processInputLogic();
}

void Creative::physics() {
    // Apply drag forces to all grids before physics update
    applyDragForces();
    
    if (doCreate || doRemove || doForce || doTrackSpeed || doConfigure) {
        // Perform unified grid traversal for all actions
        std::weak_ptr<Grid> targetGridWeak;
        glm::ivec3 targetPos;
        glm::ivec3 hitPos;
        bool blockFound = false;
        double shortestSquaredDistance = DBL_MAX;
        
        // Camera position and direction
        glm::dvec3 startPos = m_gameBase->m_graphicsEngine->m_camPos;
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
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
                    glm::dvec3 applicationPoint = m_gameBase->m_graphicsEngine->m_camPos;
                    
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
    }

    // Reset flags
    doCreate = false;
    doRemove = false;
    doConfigure = false;
    doForce = false;
    doTrackSpeed = false;
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
            const double dragCoefficient = 0.04 * 0.4;
            
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
    if (!blockFound) {
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
    
    if (!isSameBlock) {
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

void Creative::updateMarkerPositions() {
    // Reset cursor proximity state
    m_cursorNearMarker = false;
    m_nearestMarkerIndex = -1;
    
    if (!m_hasSelectedBlock || m_selectedGrid.expired()) {
        // Clear all markers if no block selected - properly remove instances first
        if (auto geometry = m_marker.lock()) {
            for (auto& instance : m_markerInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst.get());
                }
            }
        }
        m_markerInstances.clear();
        return;
    }
    
    auto selectedGrid = m_selectedGrid.lock();
    if (!selectedGrid) {
        m_hasSelectedBlock = false;
        // Clear all markers - properly remove instances first
        if (auto geometry = m_marker.lock()) {
            for (auto& instance : m_markerInstances) {
                if (auto inst = instance.lock()) {
                    geometry->removeInstance(inst.get());
                }
            }
        }
        m_markerInstances.clear();
        return;
    }
    
    // Calculate 8 corner positions in world space
    std::vector<glm::dvec3> cornerPositions;
    for (int x = 0; x <= 1; ++x) {
        for (int y = 0; y <= 1; ++y) {
            for (int z = 0; z <= 1; ++z) {
                glm::dvec3 corner = glm::dvec3(m_selectedBlockCoord) + glm::dvec3(x, y, z);
                glm::dvec3 worldCorner = selectedGrid->gridToWorld(corner);
                cornerPositions.push_back(worldCorner);
            }
        }
    }
    
    // Get positions and scales of all markers
    auto selectorResult = PositionSelector::selectFromPositions(
        cornerPositions,
        0.02, // Small projected radius
        m_gameBase->m_graphicsEngine->m_camPos,
        m_gameBase->m_graphicsEngine->m_camOri,
        m_gameBase->m_graphicsEngine->m_fieldOfView,
        static_cast<double>(m_gameBase->m_graphicsEngine->m_screen_width) / 
        static_cast<double>(m_gameBase->m_graphicsEngine->m_screen_height),
        glm::dvec2(0.0, 0.0), // Screen center as cursor position
        5
    );
    
    // Check if cursor is near any marker and store calculated positions/scales
    struct MarkerData {
        glm::vec2 position;
        glm::vec2 scale;
    };
    std::vector<MarkerData> markerData;
    
    if (selectorResult.closestIndex >= 0 && selectorResult.distanceToClosest < 0.1) {
        m_cursorNearMarker = true;
        m_nearestMarkerIndex = selectorResult.closestIndex;
        
        // Print the selected corner coordinate with index
        glm::dvec3 selectedCorner = cornerPositions[m_nearestMarkerIndex];
        std::cout << "Near corner " << m_nearestMarkerIndex << ": (" << selectedCorner.x << ", " 
                 << selectedCorner.y << ", " << selectedCorner.z << ")" << std::endl;
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
    MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->m_mouseHandler;
    KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->m_keyboardHandler;
    
    // Camera movement speed
    const double mouseSensitivity = 0.0014;
    
    // Calculate movement vectors based on camera orientation
    glm::dvec3 right = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 up = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 0.0, 1.0);

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
        m_gameBase->m_graphicsEngine->m_camOri = m_gameBase->m_graphicsEngine->m_camOri * yawQuat * pitchQuat * rollQuat;
        m_gameBase->m_graphicsEngine->m_camOri = glm::normalize(m_gameBase->m_graphicsEngine->m_camOri);
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
        m_gameBase->m_graphicsEngine->m_camPos += moveDirection;
    }
}