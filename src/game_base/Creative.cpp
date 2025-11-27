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
#include "../graphics/instanceHandler/InstanceHandler.h"
#include "RadialMenu.h"
#include "tools/CharacterSelectionTool.h"
#include "tools/FreeCameraController.h"
#include "DigibotPlayerController.h"
#include "tools/ColorTool.h"
#include "tools/ModifyTool.h"
#include "tools/BuildTool.h"
#include "../utils/ColorUtils.h"
#include <float.h>
#include "../physics/SensorCollider.h"
#include "../utils/GridGeometry.h"
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
    m_colorTool = std::make_unique<ColorTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    // Create free camera controller
    m_freeCameraController = std::make_unique<FreeCameraController>(m_gameBase->m_graphicsEngine.get());

    // Create digibot player controller
    m_digibotPlayerController = std::make_unique<DigibotPlayerController>(m_gameBase->m_graphicsEngine.get());

    // Create character control tool
    m_characterSelectionTool = std::make_unique<CharacterSelectionTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    // Create modify tool
    m_modifyTool = std::make_unique<ModifyTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    // Create build tool
    m_buildTool = std::make_unique<BuildTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    m_radialMenu->setVisible(false);

    // Load crosshair geometry (instance will be managed in processInputLogic)
    m_crosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/00_crosshair.png", -1, true);

    // Create interaction sensor for spatial filtering
    glm::dvec3 sensorHalfScale(m_interactionRange / 2.0, m_interactionRange / 2.0, m_interactionRange / 2.0);
    glm::dvec3 initialPos = m_gameBase->m_graphicsEngine->getCamPos() + 
        m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, m_interactionRange / 2.0, 0.0);
    m_interactionSensor = m_gameBase->m_physicsEngine->getCollisionDetector().addSensorCollider(initialPos, sensorHalfScale);
}

Creative::~Creative() {
    // Remove crosshair instance if it exists
    if (auto instance = m_crosshairInstance.lock()) {
        if (auto geometry = m_crosshairGeometry.lock()) {
            geometry->removeInstance(instance.get());
        }
    }

    // Remove sensor collider
    if (!m_interactionSensor.expired()) {
        m_gameBase->m_physicsEngine->getCollisionDetector().removeCollider(m_interactionSensor);
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
        glm::dvec3 endPos = startPos + forward * m_interactionRange;

        // Get interpolation time for accurate raycasting
        auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
        
        // Find closest ray intersection across all grids
        for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
            if (!gridShared) continue; // Safety check
            
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

    // Apply drag forces to all grids before physics update
    applyDragForces();

    // Update interaction sensor position (after physics completes)
    if (auto sensor = m_interactionSensor.lock()) {
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 sensorPosition = m_gameBase->m_graphicsEngine->getCamPos() + forward * (m_interactionRange / 2.0);
        
        sensor->m_position = sensorPosition;
        sensor->m_orientation = m_gameBase->m_graphicsEngine->getCamOri();
    }

    // Get filtered grids from sensor
    std::vector<std::weak_ptr<Grid>> interactionGrids;
    if (auto sensorShared = m_interactionSensor.lock()) {
        SensorCollider* sensorPtr = static_cast<SensorCollider*>(sensorShared.get());
        interactionGrids = m_gameBase->getGridSubsystem()->getGridsFromOverlaps(sensorPtr);
    }

    //// TEST: Press J to randomly color grids in sensor range
    //KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->getKeyboardHandler();
    //if (keyboard->m_j.justPressed()) {
    //    for (const auto& gridWeak : interactionGrids) {
    //        if (auto grid = gridWeak.lock()) {
    //            auto cells = grid->getCells();
    //            // Generate random color
    //            glm::dvec4 randomColor(
    //                (double)rand() / RAND_MAX,
    //                (double)rand() / RAND_MAX,
    //                (double)rand() / RAND_MAX,
    //                1.0
    //            );
    //            for (auto cell : cells) {
    //                glm::ivec3 coord = cell.first;
    //                grid->setColor(coord, randomColor);
    //            }
    //        }
    //    }
    //}

    // Call color tool physics callback
    m_colorTool->onPhysicsUpdateComplete(interactionGrids);

    // Call character selection tool physics callback
    m_characterSelectionTool->onPhysicsUpdateComplete();

    // Call modify tool physics callback
    m_modifyTool->onPhysicsUpdateComplete(interactionGrids);

    // Call build tool physics callback
    m_buildTool->onPhysicsUpdateComplete(interactionGrids);

    // Call player controller physics callback if character control is active
    if (m_characterSelectionTool->isActive()) {
        const auto& characters = m_gameBase->m_characterSubsystem->getCharacters();
        for (const auto& character : characters) {
            std::shared_ptr<Digibot> digibot = std::dynamic_pointer_cast<Digibot>(character);
            if (digibot) {
                m_digibotPlayerController->onPhysicsUpdateComplete(digibot->getController(), interactionGrids, m_interactionRange);
                break; // Only process first character for now
            }
        }
    }
    
    // Apply drag forces to all grids before physics update
    applyDragForces();
}

void Creative::applyDragForces() {
    // Apply drag to all objects before running physics
    // Get all rigid bodies from the physics engine
    const auto& rigidBodies = m_gameBase->m_physicsEngine->getRigidBodies();
    
    // Apply drag to each non-static rigid body
    for (const auto& bodyPtr : rigidBodies) {
        // Skip null or static bodies, or bodies that already have forces applied
        if (!bodyPtr || bodyPtr->m_isStatic) {
            continue;
        }
        
        // Simple drag force calculation: -dragCoefficient * velocity
        const double dragCoefficient = 0.04 * 0.2*0.;
        
        // Apply drag to linear velocity
        if (glm::length(bodyPtr->m_velocity) > 0.0) {
            glm::dvec3 dragForce = -dragCoefficient * bodyPtr->m_velocity * bodyPtr->m_mass;
            m_gameBase->m_physicsEngine->applyForce(bodyPtr.get(), dragForce);
        }
        
        // Apply drag to angular velocity
        if (glm::length(bodyPtr->m_angularMomentumBody) > 0.0) {
            glm::dvec3 angularDrag = -dragCoefficient * bodyPtr->getWorldInertiaTensor() * bodyPtr->getAngularVelocityWorld();
            m_gameBase->m_physicsEngine->applyTorque(bodyPtr.get(), angularDrag);
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
            // Get grid collider from the grid itself (not from rigid body)
            auto colliderWeak = m_gameBase->getGridSubsystem()->getGrids()[ii]->getCollider();
            auto collider = colliderWeak.lock();
            GridCollider* gridCollider = static_cast<GridCollider*>(collider.get());
            
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
    
    // Get frame timing for deltaTime calculation
    int frameRate = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime = 1.0 / static_cast<double>(frameRate);
    
    // Structural analysis with G key
    if (keyboard->m_g.justPressed()) {
        for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
            if (gridShared) gridShared->visualizeStructuralIntegrity();
        }
    }
 
    // Toggle mouse lock with M key
    if (keyboard->m_m.justPressed()) {
        bool isLocked = mouseHandler->getMouseLock();
        mouseHandler->setMouseLock(!isLocked);
        std::cout << "Mouse " << (isLocked ? "unlocked" : "locked") << std::endl;
    }

    // Force application with F key
    doForce = keyboard->m_f.isDown();
    forceMultiplier = doForce ? (keyboard->m_f.timeDown() * 0.04 + 1.0) : 1.0;
    
    // Speed tracking with Z key  
    doTrackSpeed = keyboard->m_z.justPressed();

    // Determine which camera controller to use based on character selection
    if (m_characterSelectionTool->isActive()) {
        // Get render parameters for interpolation
        auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();

        // Character control mode - use player controller
        // Get first available character
        std::weak_ptr<Digibot> pilotableCharacter;
        const auto& characters = m_gameBase->m_characterSubsystem->getCharacters();
        for (const auto& character : characters) {
            // Try to cast to Digibot
            std::shared_ptr<Digibot> digibot = std::dynamic_pointer_cast<Digibot>(character);
            if (digibot) {
                pilotableCharacter = digibot;
                break;
            }
        }

        // Get controller for update
        DigibotController* controller = nullptr;
        auto digibot = pilotableCharacter.lock();
        if (digibot) {
            controller = digibot->getController();
        }

        m_digibotPlayerController->setPilotableCharacter(pilotableCharacter);
        m_digibotPlayerController->enable();
        m_digibotPlayerController->update(
            controller,
            m_gameBase->m_graphicsEngine->getCamPos(),
            m_gameBase->m_graphicsEngine->getCamOri(),
            timeRemainder
        );
    } else {
        // Free camera mode
        m_digibotPlayerController->disable();
        
        m_freeCameraController->update(
            deltaTime, 
            m_gameBase->m_graphicsEngine->getCamPos(),
            m_gameBase->m_graphicsEngine->getCamOri()
        );
    }

    // Toggle radial menu visibility with R key
    if (keyboard->m_r.justPressed()) {
        bool visible = m_radialMenu->isVisible();
        m_radialMenu->setVisible(!visible);
        
        // Position radial menu in front of camera
        glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        m_radialMenuRelativePosition = forward * m_radialMenuDistance;
        m_radialMenu->setPosition(cameraPos + m_radialMenuRelativePosition);
    }

    // Handle radial menu interaction when visible
    bool radialMenuConsumedMouse = false;
    if (m_radialMenu->isVisible()) {
        // Apply grid rotation to menu relative position if in character control mode
        if (m_characterSelectionTool->isActive() && m_digibotPlayerController->isEnabled()) {
            glm::dquat gridRotation = m_digibotPlayerController->getSurfaceRotation();
            m_radialMenuRelativePosition = gridRotation * m_radialMenuRelativePosition;
        }

        // Update radial menu position to follow camera
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

        // Character selection tool doesn't need input for now (toggle handled by radial menu)
        m_characterSelectionTool->preRenderCallback(false);
    } else{
        m_colorTool->preRenderCallback(false, false);
        m_modifyTool->preRenderCallback(false, false);
        m_buildTool->preRenderCallback(false, false);
        m_characterSelectionTool->preRenderCallback(false);
    }
}
