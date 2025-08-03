// Creative.cpp
#include "Creative.h"
#include "../game_base/GameBase.h"
#include "../graphics/GraphicsEngine.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../game_base/Grid.h"
#include "../debug/DebugGlobals.h"
#include <iostream>
#include <float.h>

Creative::Creative(GameBase* gameBase) : Mode(gameBase) {
}

void Creative::processInputs() {
    processInputLogic();
}

void Creative::physics() {
    // Apply drag forces to all grids before physics update
    applyDragForces();
    
    if (doCreate || doRemove || doForce || doTrackSpeed) {
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

void Creative::processInputLogic() {
    MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->m_mouseHandler;
    KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->m_keyboardHandler;
    
    // Camera movement speed
    const double mouseSensitivity = 0.0014;
    
    // Calculate movement vectors based on camera orientation
    glm::dvec3 right = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 up = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 0.0, 1.0);

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