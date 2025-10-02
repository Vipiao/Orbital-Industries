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
#include "../graphics/MeshManager2D/MeshManager2D.h"
#include "../graphics/MeshManager2D/GeometryInstance.h"
#include "StructuralBlock.h"
#include "../graphics/InstanceHandler/InstanceHandler.h"
#include "RadialMenu.h"
#include "ColorTool.h"
#include "ModifyTool.h"
#include "BuildTool.h"
#include "../utils/ColorUtils.h"
#include <float.h>
#include <map>

Creative::Creative(GameBase* gameBase)
    : Mode(gameBase) {
    
    // Create radial menu
    m_radialMenu = std::make_unique<RadialMenu>(m_gameBase->m_graphicsEngine.get());
    m_radialMenu->getGeometry().lock()->setDepthCompression(0.01);
    
    // Create root node for radial menu
    int64_t rootId = m_radialMenu->createNode(); // parentId defaults to -1 (root)

    // Add fake node so color tool isn't in center
    m_radialMenu->createNode(rootId);

    // Create color tool
    m_colorTool = std::make_unique<ColorTool>(m_gameBase, m_radialMenu.get(), rootId);

    // Create modify tool
    m_modifyTool = std::make_unique<ModifyTool>(m_gameBase, m_radialMenu.get(), rootId);

    // Create build tool
    m_buildTool = std::make_unique<BuildTool>(m_gameBase, m_radialMenu.get(), rootId);

    m_radialMenu->setVisible(false);

    // Load crosshair geometry (instance will be managed in processInputLogic)
    m_crosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/00_crosshair.png", -1, true);
}

Creative::~Creative() {
    // Remove crosshair instance if it exists
    if (auto instance = m_crosshairInstance.lock()) {
        if (auto geometry = m_crosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
        }
    }
    // Destructor defined here where RadialMenu is complete type
}

void Creative::processInputs() {
    processInputLogic();
}

void Creative::physics() {
    if (doForce || doTrackSpeed) {
        // Perform ray casting for force and speed tracking only
        std::weak_ptr<Grid> targetGridWeak;
        glm::ivec3 hitPos;
        bool blockFound = false;
        double closestT = -1.0;
        
        // Camera position and direction
        glm::dvec3 startPos = m_gameBase->m_graphicsEngine->getCamPos();
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 endPos = startPos + forward * 20.0; // Cast ray 20 units forward
        
        // Find closest ray intersection across all grids
        for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
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
                    glm::dvec3 applicationPoint = m_gameBase->m_graphicsEngine->getCamPos();
                    
                    // Apply force at the point
                    m_gameBase->m_physicsEngine->applyForceAtPoint(body, force, applicationPoint);
                    
                    //std::cout << "Applied force to grid at t: " << closestT << std::endl;
                }
            } else {
                //std::cout << "No target found for force application" << std::endl;
            }
        }
    }

    // Reset flags
    doForce = false;
    doTrackSpeed = false;

    // Call color tool physics callback
    m_colorTool->onPhysicsUpdateComplete();

    // Call modify tool physics callback
    m_modifyTool->onPhysicsUpdateComplete();

    // Call build tool physics callback
    m_buildTool->onPhysicsUpdateComplete();
    
    // Apply drag forces to all grids before physics update
    applyDragForces();
}

void Creative::applyDragForces() {
    // Apply drag to all objects before running physics
    for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
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

void Creative::processInputLogic() {
    // Manage crosshair visibility based on color tool state
    bool anyToolActive = m_colorTool->isActive() || m_modifyTool->isActive() || m_buildTool->isActive();
    if (anyToolActive) {
        // Hide regular crosshair when color tool is active
        if (auto instance = m_crosshairInstance.lock()) {
            if (auto geometry = m_crosshairGeometry.lock()) {
                geometry->removeInstance(instance.get());
                m_crosshairInstance.reset();
            }
        }
    } else {
        // Show regular crosshair when color tool is not active
        if (!m_crosshairInstance.lock() && m_crosshairGeometry.lock()) {
            m_crosshairInstance = m_crosshairGeometry.lock()->createInstance();
            if (auto instance = m_crosshairInstance.lock()) {
                instance->setPosition(glm::vec2(0.0f, 0.0f));
                instance->setScale(glm::vec2(0.05f, 0.05f));
                instance->setColor(glm::dvec4(1.0, 0.0, 0.0, 0.75)); // 50% transparency
            }
        }
    }

    MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->getMouseHandler();
    KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->getKeyboardHandler();

    // TEST START
    //CellMetadata* metadata = collider->get_pointer<CellMetadata>();
    if (keyboard->m_h.justPressed()) {
        for (size_t ii = 0; ii < m_gameBase->getGridSubsystem()->getGrids().size(); ii++) {
            auto cells = m_gameBase->getGridSubsystem()->getGrids()[ii]->getCells();
            GridCollider* gridCollider = static_cast<GridCollider*>(m_gameBase->getGridSubsystem()->getGrids()[ii]->getRigidBody()->m_collider);
            
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
                
                m_gameBase->getGridSubsystem()->getGrids()[ii]->setColor(coord, color);
            }
        }
    }

    // TEST END
    
    // Camera movement speed
    const double mouseSensitivity = 0.0014;

    // Get framerate for framerate-independent movement
    int frameRate = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime = 1.0 / static_cast<double>(frameRate);
    
    // Calculate movement vectors based on camera orientation
    glm::dvec3 right = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(1.0, 0.0, 0.0);
    glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
    glm::dvec3 up = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 0.0, 1.0);

    // Structural analysis with G key
    if (keyboard->m_g.justPressed()) {
        //std::cout << "Visualizing structural analysis on " << m_gameBase->m_grids.size() << " grids..." << std::endl;
        
        for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
            if (gridShared) gridShared->visualizeStructuralIntegrity();
        }
    }
    
    // Check for input actions that require grid traversal
    // Set flags based on input (don't execute immediately)
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
        //m_moveSpeed *= 1.05;
        m_moveSpeed *= glm::exp(8. * deltaTime);
    }
    if (keyboard->m_v.isDown()) {
        m_moveSpeed /= glm::exp(8. * deltaTime);
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
        const double rollSpeed = 1.65 * deltaTime;
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
        moveDirection = glm::normalize(moveDirection) * m_moveSpeed * deltaTime;
        m_gameBase->m_graphicsEngine->getCamPos() += moveDirection;
    }

    // Toggle radial menu visibility with B key
    if (keyboard->m_r.justPressed()) {
        bool visible = m_radialMenu->isVisible();
        m_radialMenu->setVisible(!visible);
        glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        m_radialMenuRelativePosition = forward * m_radialMenuDistance;
        m_radialMenu->setPosition(cameraPos + m_radialMenuRelativePosition);

    }

    // Handle radial menu interaction when visible.
    bool radialMenuConsumedMouse = false;
    if (m_radialMenu->isVisible()) {
        //
        glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
        m_radialMenu->setPosition(cameraPos + m_radialMenuRelativePosition);

        // Make menu use camera orientation with 90 degree offset around X axis
        glm::dquat xOffset = glm::angleAxis(glm::radians(90.0), glm::dvec3(1.0, 0.0, 0.0));
        glm::dquat menuOrientation = m_gameBase->m_graphicsEngine->getCamOri() * xOffset;
        m_radialMenu->setOrientation(menuOrientation);

        // Generate camera ray
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 rayStart = cameraPos;
        glm::dvec3 rayEnd = cameraPos + forward * 5.0;
        
        // Convert to local space
        glm::dvec3 localRayStart = m_radialMenu->worldToLocal(rayStart);
        glm::dvec3 localRayEnd = m_radialMenu->worldToLocal(rayEnd);
        
        // Check for selection
        bool doSelect = mouseHandler->leftClick();
        
        // Run radial menu interaction and check if it consumed the input
        radialMenuConsumedMouse = m_radialMenu->run(localRayStart, localRayEnd, doSelect);
        
        // Handle right click for navigate up
        if (mouseHandler->rightClick() && radialMenuConsumedMouse) {
            if (!m_radialMenu->navigateToParent()) {
                // Close menu if already at root
                m_radialMenu->setVisible(false);
            }
        }
    }

    // Only pass mouse clicks to tools if menu didn't consume them
    if (!radialMenuConsumedMouse) {
        // Color tool uses right click for copy, left click for paste
        bool doTryCopy = mouseHandler->rightClick();
        bool doTryPaste = mouseHandler->getLeftDown();
        m_colorTool->preRenderCallback(doTryCopy, doTryPaste);

        // ModifyTool uses left click for modify, right click for cancel
        bool doModify = mouseHandler->leftClick();
        bool doCancel = mouseHandler->rightClick();
        m_modifyTool->preRenderCallback(doModify, doCancel);


        // BuildTool uses the classic right click to create, left click to remove
        bool doCreate = mouseHandler->rightClick() || (mouseHandler->getRightDown() && mouseHandler->getTimeRightDown() > 32);
        bool doRemove = mouseHandler->leftClick() || (mouseHandler->getLeftDown() && mouseHandler->getTimeLeftDown() > 32);
        m_buildTool->preRenderCallback(doCreate, doRemove);
    } else{
        m_colorTool->preRenderCallback(false, false);
        m_modifyTool->preRenderCallback(false, false);
        m_buildTool->preRenderCallback(false, false);
    }
}
