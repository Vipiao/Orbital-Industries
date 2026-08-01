// Creative.cpp
#include "Creative.h"
#include "../game_base/GameBase.h"
#include "GridSubsystem.h"
#include "../characters/CharacterSubsystem.h"
#include "../characters/digibot/Digibot.h"
#include "../physics/GridCollider.h"
#include "graphics/GraphicsEngine.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../game_base/Grid.h"
#include "debug/DebugGlobals.h"
#include "utils/PositionSelector.h"
#include <iostream>
#include "utils/ColorUtils.h"
#include "graphics/MeshManager2D/MeshManager2D.h"
#include "graphics/MeshManager2D/Instance2D.h"
#include "StructuralBlock.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include "RadialMenu.h"
#include "tools/CharacterSelectionTool.h"
#include "tools/FreeCameraController.h"
#include "DigibotPlayerController.h"
#include "tools/ColorTool.h"
#include "tools/ModifyTool.h"
#include "tools/BuildTool.h"
#include "utils/ColorUtils.h"
#include <float.h>
#include "../physics/SensorCollider.h"
#include "utils/GridGeometry.h"
#include <map>

Creative::Creative(GameBase* gameBase)
    : Mode(gameBase) {
    
    // Create radial menu
    m_radialMenu = std::make_unique<RadialMenu>(m_gameBase->m_graphicsEngine.get());

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
    // Test fixture: start in character control so both peers grab a character
    // immediately without touching the radial menu.
    m_characterSelectionTool->activate();

    // Create modify tool
    m_modifyTool = std::make_unique<ModifyTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    // Create build tool
    m_buildTool = std::make_unique<BuildTool>(m_gameBase, m_radialMenu.get(), rootId, m_interactionRange);

    m_radialMenu->setVisible(false);

    // Load crosshair geometry (instance will be managed in processInputLogic)
    m_crosshairGeometry = m_gameBase->m_graphicsEngine->getMeshManager2D()->loadMesh("../media/blender/03_face.obj", "../media/2d_graphics/00_crosshair.png", -1, true);

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
            geometry->removeInstance(instance);
        }
    }

    // Remove sensor collider
    if (!m_interactionSensor.expired()) {
        m_gameBase->m_physicsEngine->getCollisionDetector().removeCollider(m_interactionSensor);
    }
    // Destructor defined here where RadialMenu is complete type
}

void Creative::frameProcessInputs() {
    processInputLogic();
}

bool Creative::isControllingCharacter() const {
    // Controlling requires both the intent (selection tool engaged) and a granted
    // character; between the two the player waits in free camera.
    return m_characterSelectionTool->isActive() && !m_boundCharacter.expired();
}

bool Creative::wantsCharacterControl() const {
    return m_characterSelectionTool->isActive();
}

std::weak_ptr<Digibot> Creative::desiredCharacter() const {
    // The character the player means is the one nearest the camera.
    glm::dvec3 cameraPos{m_gameBase->m_graphicsEngine->getCamPos()};
    std::shared_ptr<Digibot> nearest{};
    double nearestDistance{0.0};
    for (const auto& character : m_gameBase->m_characterSubsystem->getCharacters()) {
        std::shared_ptr<Digibot> digibot{std::dynamic_pointer_cast<Digibot>(character)};
        std::shared_ptr<RigidBody> body{digibot ? digibot->getRigidBody().lock() : nullptr};
        if (!body) {
            continue;
        }
        double distance{glm::length(body->m_position - cameraPos)};
        if (!nearest || distance < nearestDistance) {
            nearest = digibot;
            nearestDistance = distance;
        }
    }
    return nearest;
}

bool Creative::hasBoundCharacter() const {
    return !m_boundCharacter.expired();
}

void Creative::stepControl() {
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
                if (targetGrid) {
                    auto bodyWeak = targetGrid->getRigidBody();
                    auto body = bodyWeak.lock();
                    if (body) {
                        // Set camera velocity to match the rigid body's velocity
                        //m_cameraVelocity = body->m_velocity;
                    }
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
                if (targetGrid) {
                    auto bodyWeak = targetGrid->getRigidBody();
                    auto body = bodyWeak.lock();
                    if (body) {
                        // Apply force in the view direction (acceleration * mass)
                        const double forceStrength =
                            PhysicsUnits::metersPerSecondSquared(4.096) * body->m_mass * forceMultiplier;
                        // Get body interpolated transform
                        glm::dvec3 interpolatedPos;
                        glm::dquat interpolatedOri;
                        body->getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);

                        // Convert camera position to body-local space
                        glm::dvec3 camPos = m_gameBase->m_graphicsEngine->getCamPos();
                        glm::dvec3 localApplicationPoint = GridGeometry::worldToGrid(
                            camPos, interpolatedPos, interpolatedOri, {});

                        // Rotate force vector into body-local space
                        glm::dvec3 localForce = glm::conjugate(interpolatedOri) * (forward * forceStrength);

                        m_gameBase->m_physicsEngine->applyLocalForceAtPoint(bodyWeak, localForce, localApplicationPoint);
                        
                        //std::cout << "Applied force to grid at t: " << closestT << std::endl;
                    }
                }
            } else {
                //std::cout << "No target found for force application" << std::endl;
            }
        }
    }

    // Reset flags
    doForce = false;
    doTrackSpeed = false;

    // Update interaction sensor position for the coming step. The sensor is an
    // input to that step's overlap test, so it is anchored in physics state:
    // the controlled body's position advanced one tick along its velocity is
    // where the body sits when the test runs, keeping the sensor co-moving
    // with the grids it filters at any world speed. The camera position is
    // render-space (timeRemainder-extrapolated) and anchors the sensor only
    // when no character is bound.
    if (auto sensor = m_interactionSensor.lock()) {
        glm::dvec3 forward{m_gameBase->m_graphicsEngine->getCamOri() *
                           glm::dvec3{0.0, 1.0, 0.0}};
        glm::dvec3 anchor{m_gameBase->m_graphicsEngine->getCamPos()};
        std::shared_ptr<Digibot> boundDigibot{m_boundCharacter.lock()};
        std::shared_ptr<RigidBody> body{
            boundDigibot ? boundDigibot->getRigidBody().lock() : nullptr};
        if (body) {
            anchor = body->m_position + body->m_velocity;
        }
        sensor->m_position = anchor + forward * (m_interactionRange / 2.0);
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

    m_colorTool->stepControl(interactionGrids);

    m_characterSelectionTool->stepControl();

    m_modifyTool->stepControl(interactionGrids);

    m_buildTool->stepControl(interactionGrids);

    // Run player controller if character control is active
    if (isControllingCharacter()) {
        std::shared_ptr<Digibot> digibot{m_boundCharacter.lock()};
        if (digibot) {
            m_digibotPlayerController->stepControl(digibot->getController(),
                                                   interactionGrids, m_interactionRange);
        }
    }

    // Apply drag forces to all grids before the coming physics step
    applyDragForces();
}

void Creative::applyDragForces() {
    // Apply drag to all objects before running physics
    // Get all rigid bodies from the physics engine
    for (const auto& weak : m_gameBase->m_physicsEngine->getRigidBodies()) {
        auto bodyPtr = weak.lock();
        if (!bodyPtr || bodyPtr->m_isStatic) {
            continue;
        }

        // Velocity/spin damping rate (force = -coeff * velocity * mass), so a 1/time
        // gain. Currently disabled by the trailing * 0.0.
        const double dragCoefficient = PhysicsUnits::perSecond(0.128) * 0.;

        if (glm::length(bodyPtr->m_velocity) > 0.0) {
            glm::dvec3 dragForce = -dragCoefficient * bodyPtr->m_velocity * bodyPtr->m_mass;
            m_gameBase->m_physicsEngine->applyForce(weak, dragForce);
        }

        if (glm::length(bodyPtr->m_angularMomentumBody) > 0.0) {
            glm::dvec3 angularDrag = -dragCoefficient * bodyPtr->getWorldInertiaTensor()
                                     * bodyPtr->getAngularVelocityWorld();
            m_gameBase->m_physicsEngine->applyTorque(weak, angularDrag);
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
                geometry->removeInstance(instance);
                m_crosshairInstance.reset();
            }
        }
    } else {
        // Show regular crosshair when color tool is not active
        if (!m_crosshairInstance.lock() && m_crosshairGeometry.lock()) {
            auto geometry = m_crosshairGeometry.lock();
            m_crosshairInstance = geometry->addInstance();
            if (auto instance = m_crosshairInstance.lock()) {
                instance->m_position = glm::dvec2(0.0, 0.0);
                instance->m_scale = glm::dvec2(0.05, 0.05);
                instance->m_color = glm::dvec4(1.0, 0.0, 0.0, 0.75); // 50% transparency
                geometry->updateInstanceInBuffer(instance.get());
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
    double frameRate = m_gameBase->m_graphicsEngine->getFrameRate();
    double deltaTime = 1.0 / frameRate;
    
    // Structural analysis with G key
    if (keyboard->m_g.justPressed()) {
        for (const auto& gridShared : m_gameBase->getGridSubsystem()->getGrids()) {
            if (gridShared) gridShared->visualizeStructuralIntegrity();
        }
    }
 
    // Toggle fullscreen with F11 key
    if (keyboard->m_f11.justPressed()) {
        m_gameBase->m_graphicsEngine->toggleFullscreen();
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

    // Character control drives the player controller only once a character is
    // bound; an engaged tool with no grant yet stays in free camera.
    if (isControllingCharacter()) {
        // Get render parameters for interpolation
        auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();

        std::shared_ptr<Digibot> digibot{m_boundCharacter.lock()};
        DigibotController* controller{digibot ? digibot->getController() : nullptr};

        m_digibotPlayerController->setPilotableCharacter(m_boundCharacter);
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

    // Calculate delta time remainder for surface rotation
    auto [_, timeRemainder] = m_gameBase->m_graphicsEngine->getRenderParameters();
    double deltaTimeRemainder = timeRemainder - m_lastTimeRemainder;
    if (deltaTimeRemainder < 0.0) deltaTimeRemainder += 1.0; // Handle wraparound
    m_lastTimeRemainder = timeRemainder;

    // Handle radial menu interaction when visible
    bool radialMenuConsumedMouse = false;
    if (m_radialMenu->isVisible()) {
        // Apply grid rotation to menu relative position if in character control mode
        if (m_characterSelectionTool->isActive() && m_digibotPlayerController->isEnabled()) {
            glm::dvec3 angularVelocity = m_digibotPlayerController->getSurfaceAngularVelocity();
            double angularVelocityMagnitude = glm::length(angularVelocity);
            if (angularVelocityMagnitude > 1e-6) {
                double rotationAngle = angularVelocityMagnitude * deltaTimeRemainder;
                glm::dvec3 rotationAxis = angularVelocity / angularVelocityMagnitude;
                glm::dquat surfaceRotation = glm::angleAxis(rotationAngle, rotationAxis);
                m_radialMenuRelativePosition = surfaceRotation * m_radialMenuRelativePosition;
            }
        }

        // Update radial menu position to follow camera
        glm::dvec3 cameraPos = m_gameBase->m_graphicsEngine->getCamPos();
        m_radialMenu->setPosition(cameraPos + m_radialMenuRelativePosition);

        // Billboard the menu: local +Z (its plane normal) points from the menu back at the camera,
        // and local +Y follows the camera up axis so the menu stays upright on screen.
        glm::dvec3 normal{glm::normalize(-m_radialMenuRelativePosition)};
        glm::dvec3 cameraUp{m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3{0.0, 0.0, 1.0}};

        // Fall back to the camera forward axis when the up axis is near parallel to the normal
        if (glm::abs(glm::dot(cameraUp, normal)) > 0.999) {
            cameraUp = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3{0.0, 1.0, 0.0};
        }

        glm::dvec3 right{glm::normalize(glm::cross(cameraUp, normal))};
        glm::dvec3 up{glm::cross(normal, right)};
        m_radialMenu->setOrientation(glm::quat_cast(glm::dmat3{right, up, normal}));

        // Generate camera ray, long enough to reach the menu plane at any viewing angle
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->getCamOri() * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 rayStart = cameraPos;
        glm::dvec3 rayEnd = cameraPos + forward * m_radialMenuDistance * 4.0;
        
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
        m_colorTool->framePreRender(doTryCopy, doTryPaste);

        // ModifyTool uses left click for modify, right click for cancel
        bool doModify = mouseHandler->leftClick();
        bool doCancel = mouseHandler->rightClick();
        m_modifyTool->framePreRender(doModify, doCancel);


        // BuildTool uses the classic right click to create, left click to remove
        bool doCreate = mouseHandler->rightClick() || (mouseHandler->getRightDown() && mouseHandler->getTimeRightDown() > 32);
        bool doRemove = mouseHandler->leftClick() || (mouseHandler->getLeftDown() && mouseHandler->getTimeLeftDown() > 32);
        m_buildTool->framePreRender(doCreate, doRemove);

        // Character selection tool doesn't need input for now (toggle handled by radial menu)
        m_characterSelectionTool->framePreRender(false);
    } else{
        m_colorTool->framePreRender(false, false);
        m_modifyTool->framePreRender(false, false);
        m_buildTool->framePreRender(false, false);
        m_characterSelectionTool->framePreRender(false);
    }
}
