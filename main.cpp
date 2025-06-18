// main.cpp
#include "src/GameBase.h"
#include "src/TimeHandler.h"
#include "src/DebugVisualization.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class MyGame : public GameBase {
private:
    std::unique_ptr<DebugVisualization> m_debugViz;

public:
    MyGame(TimeHandler* timeHandler, 
           GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE) 
      : GameBase(800, 600, "3D Grid Demo", timeHandler, controlMode) {
        
        // Create debug visualization system
        m_debugViz = std::make_unique<DebugVisualization>(m_graphicsEngine->m_meshHandler.get());

        // Set up initial camera position and orientation
        m_graphicsEngine->m_camPos = glm::dvec3(0, 0, 0);
        m_graphicsEngine->m_camOri = glm::angleAxis(glm::radians(0.0), glm::dvec3(1, 0, 0));
        m_graphicsEngine->m_fieldOfView = glm::radians(90.0);
        
        // Enable mouse lock for camera control
        m_graphicsEngine->m_mouseHandler->setMouseLock(true);

        // Create debug sphere at origin for testing
        m_debugViz->createSphere(glm::dvec3(0.0, 0.0, 0.0), 1.0);
        
        // Create a center grid that will be our player object
        Grid* initialGrid = createGrid(glm::dvec3(0, 0, 0));
        addGridBlock(initialGrid, 1, 0, 0);  // Block to the right
        addGridBlock(initialGrid, 0, 0, 0);  // Center block

        
        Grid* gg = createGrid(glm::dvec3(0, 0, 0));
        addGridBlock(gg, 0, 0, 0);  // Center block
        PhysicsEngine::RigidBody* bb = gg->getRigidBody();
        bb->position = {0.5, -2, 0.5};
        
        PhysicsEngine::RigidBody* body = initialGrid->getRigidBody();
        //body->angularVelocity = glm::dvec3{ 0, 0, glm::radians(180.) };
        //body->orientation = glm::angleAxis(glm::radians(180.0), glm::dvec3{1.0, 0.,0.});
        
        // Print instructions
        std::cout << "3D Grid Block Demo" << std::endl;
        std::cout << "Controls:" << std::endl;
        std::cout << "  WASD: Move camera" << std::endl;
        std::cout << "  Mouse: Look around" << std::endl;
        std::cout << "  Space/Shift: Move up/down" << std::endl;
        std::cout << "  M: Toggle mouse lock" << std::endl;
        std::cout << "  F: Apply force to grid" << std::endl;
        std::cout << "  E: Add block at (1,1,1)" << std::endl;
        std::cout << "  Q: Remove block at (1,1,1)" << std::endl;
    }

    void addGridBlock(Grid* grid, int x, int y, int z) {
        if (grid) {
            grid->addCell(glm::ivec3(x, y, z));
        }
    }
    
    // New - Method to remove a block from the grid
    void removeGridBlock(Grid* grid, int x, int y, int z) {
        if (grid) {
            grid->removeCell(glm::ivec3(x, y, z));
        }
    }
    
protected:
    double m_moveSpeed = 0.05;

    virtual void processInput() override {
        MouseHandler* mouseHandler = m_graphicsEngine->m_mouseHandler;
        KeyboardHandler* keyboard = m_graphicsEngine->m_keyboardHandler;
        
        // Camera movement speed
        const double mouseSensitivity = 0.002;
        
        // Calculate movement vectors based on camera orientation
        glm::dvec3 right = m_graphicsEngine->m_camOri * glm::dvec3(1.0, 0.0, 0.0);
        glm::dvec3 forward = m_graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 up = m_graphicsEngine->m_camOri * glm::dvec3(0.0, 0.0, 1.0);
        
        // Add/remove blocks with T/R keys
        bool doCreate = mouseHandler->rightClick(), doRemove = mouseHandler->leftClick();
        if (!m_grids.empty() && (doCreate || doRemove)) {
            // Start position (camera position)
            glm::dvec3 startPos = m_graphicsEngine->m_camPos;
            // End position (10 meters in the direction the camera is looking)
            glm::dvec3 endPos = startPos + forward * 10.0;

            Grid* targetGrid = nullptr;
            glm::ivec3 targetPos;
            bool blockFound = false;
            size_t blockFoundIndex = 0;
            std::vector<glm::ivec3> cellsInPath;
            
            // Check all grids to find the first block hit by the ray
            for (const auto& gridPtr : m_grids) {
                Grid* grid = gridPtr.get();
                glm::dvec3 startPosLocal = grid->worldToGrid(startPos);
                glm::dvec3 endPosLocal = grid->worldToGrid(endPos);
                
                std::vector<glm::ivec3> gridCells = grid->gridTraversal(startPosLocal, endPosLocal);
                
                // Check if any cells already have blocks
                for (size_t i = 1; i < gridCells.size(); i++) { // Skip first cell (where camera is)
                    if (grid->hasCell(gridCells[i])) {
                        // We found a block! Use the cell right before it
                        targetGrid = grid;
                        targetPos = gridCells[i-1];
                        blockFound = true;
                        blockFoundIndex = i;
                        cellsInPath = gridCells;
                        break;
                    }
                }
                if (blockFound) break;
            }
            
            // If no block found and right-clicking, create a new grid
            if (!blockFound && doCreate) {
                glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
                Grid* newGrid = createGrid(newGridPos);
                addGridBlock(newGrid, 0, 0, 0);  // Add initial block at grid center
                return;  // Early return since we created a new grid
            }
            
            // If no block found and left-clicking, do nothing
            if (!blockFound) {
                return;
             }
            
            // Add or remove blocks based on key presses
            if (doCreate && targetGrid) {
                addGridBlock(targetGrid, targetPos.x, targetPos.y, targetPos.z);
            }
            
            if (doRemove && targetGrid) {
                // If we found a block, remove it instead of the target position
                if (blockFound) {
                    removeGridBlock(targetGrid, cellsInPath[blockFoundIndex].x, cellsInPath[blockFoundIndex].y, cellsInPath[blockFoundIndex].z);
                } else {
                    removeGridBlock(targetGrid, targetPos.x, targetPos.y, targetPos.z);
                }
            }
        }

        // Handle grid force application with F key
        if (keyboard->m_f.isDown()) {
            // Find the grid with the shortest ray hit distance
            Grid* targetGrid = nullptr;
            int shortestHitDistance = INT_MAX;
            
            // Camera position and direction
            glm::dvec3 startPos = m_graphicsEngine->m_camPos;
            glm::dvec3 endPos = startPos + forward * 10.0; // Cast ray 10 units forward
            
            // Check all grids for ray intersections
            for (const auto& gridPtr : m_grids) {
                Grid* grid = gridPtr.get();
                
                // Convert camera ray to grid space
                glm::dvec3 startPosLocal = grid->worldToGrid(startPos);
                glm::dvec3 endPosLocal = grid->worldToGrid(endPos);
                
                // Perform grid traversal
                std::vector<glm::ivec3> gridCells = grid->gridTraversal(startPosLocal, endPosLocal);
                
                // Check if ray hits any blocks in this grid
                for (size_t i = 1; i < gridCells.size(); i++) { // Skip first cell (where camera is)
                    if (grid->hasCell(gridCells[i])) {
                        // Found a hit! Check if this is the closest hit so far
                        if (static_cast<int>(i) < shortestHitDistance) {
                            shortestHitDistance = static_cast<int>(i);
                            targetGrid = grid;
                        }
                        break; // Only care about first hit in this grid
                    }
                }
            }
            
            // Apply force to the grid with the closest hit
            if (targetGrid) {
                PhysicsEngine::RigidBody* body = targetGrid->getRigidBody();
                if (body) {
                    // Apply force in the view direction
                    const double forceStrength = 1.0;
                    glm::dvec3 force = forward * forceStrength;
                    
                    // Apply the force at the camera position
                    glm::dvec3 applicationPoint = m_graphicsEngine->m_camPos;
                    
                    // Apply force at the point
                    m_physicsEngine->applyForceAtPoint(body, force, applicationPoint);
                    
                    std::cout << "Applied force to grid at hit distance: " << shortestHitDistance << std::endl;
                }
            }
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
            m_graphicsEngine->m_camOri = m_graphicsEngine->m_camOri * yawQuat * pitchQuat* rollQuat;
            m_graphicsEngine->m_camOri = glm::normalize(m_graphicsEngine->m_camOri);
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
            m_graphicsEngine->m_camPos += moveDirection;
        }
    }
    
    virtual void updatePhysics() override {
        // Apply drag to all objects before running physics
        for (const auto& grid : m_grids) {
            PhysicsEngine::RigidBody* body = grid->getRigidBody();
            if (body && !body->isStatic) {
                // Simple drag force calculation: -dragCoefficient * velocity
                const double dragCoefficient = 0.04;
                
                // Apply drag to linear velocity
                if (glm::length(body->velocity) > 0.0) {
                    glm::dvec3 dragForce = -dragCoefficient * body->velocity * body->mass;
                    m_physicsEngine->applyForce(body, dragForce);
                }
                
                // Apply drag to angular velocity
                if (glm::length(body->angularVelocity) > 0.0) {
                    glm::dvec3 angularDrag = -dragCoefficient * body->angularVelocity * body->momentOfInertia;
                    m_physicsEngine->applyTorque(body, angularDrag);
                }
            }
        }
        
        // Call the base class implementation to run the physics simulation
        GameBase::updatePhysics();
    }
    
private:
};

int main() {
    try {
        // Create the TimeHandler with appropriate mode
        TimeHandler* timeHandler = new TimeHandler(TimeHandler::Mode::PLAY);

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::PLAY;

        MyGame game(timeHandler, controlMode); // Updated
        game.run();
        
        // Clean up TimeHandler
        delete timeHandler; // New
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}