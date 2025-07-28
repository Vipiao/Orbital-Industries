// main.cpp
#include "src/game_base/GameBase.h"
#include "src/utils/TimeHandler.h"
#include "src/debug/DebugVisualization.h"
#include "src/debug/DebugRenderer.h"
#include "src/debug/DebugGlobals.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Define the global debug renderer (must be in exactly one .cpp file)
DebugRenderer* DebugGlobals::g_debugRenderer = nullptr;

class Game : public GraphicsEngine::CallBack {
private:
    std::unique_ptr<GameBase> m_gameBase;
    std::unique_ptr<DebugVisualization> m_debugViz;
    DebugRendererGuard m_debugGuard;
    double m_moveSpeed = 0.05;

public:
    Game(TimeHandler* timeHandler, 
         GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE) {
        
        // Create the game base instance
        m_gameBase = std::make_unique<GameBase>(800, 600, "3D Grid Demo", timeHandler, controlMode);
        
        // Set ourselves as the callback object instead of GameBase
        m_gameBase->m_graphicsEngine->addCallbackObject(this);
        
        // Setup debug visualization
        setupDebugVisualization();

        // Set global debug renderer with RAII guard  
        m_debugGuard = DebugGlobals::setDebugRenderer(m_debugViz.get());

        // Set up initial camera position and orientation
        m_gameBase->m_graphicsEngine->m_camPos = glm::dvec3(0, 0, 0);
        m_gameBase->m_graphicsEngine->m_camOri = glm::angleAxis(glm::radians(0.0), glm::dvec3(1, 0, 0));
        m_gameBase->m_graphicsEngine->m_fieldOfView = glm::radians(90.0);
        
        // Enable mouse lock for camera control
        m_gameBase->m_graphicsEngine->m_mouseHandler->setMouseLock(true);
        
        // Create a center grid that will be our player object
        Grid* initialGrid = m_gameBase->createGrid(glm::dvec3(0, 0, 0));
        RigidBody* bb = initialGrid->getRigidBody();
        bb->m_position = {0,0,0};
        //bb->m_velocity = {0.0,0.0,-0.01};
        //addGridBlock(initialGrid, 0, 0, 0);
        //addGridBlock(initialGrid, 1, 0, 0);
        //addGridBlock(initialGrid, 2, 0, 0);
        //addGridBlock(initialGrid, 3, 0, 0);
        
        for (int ll = 0; ll < 2; ll++) {
            for (int ii = -3; ii < 4; ii++)
            {
                for (int jj = -3; jj < 4; jj++)
                {
                    
                    for (int kk = -3; kk < 4; kk++)
                    {
                        addGridBlock(initialGrid, ii + ll*10, jj, kk);
                    }
                }
            }
            for (int ii = -2; ii < 3; ii++)
            {
                for (int jj = -2; jj < 3; jj++)
                {
                    
                    for (int kk = -2; kk < 3; kk++)
                    {
                        removeGridBlock(initialGrid, ii + ll*10, jj, kk);
                    }
                }
            }
        }
        for (int ii = 4; ii < 7; ii++)
        {
            for (int jj = -1; jj < 2; jj++)
            {
                
                for (int kk = -2; kk < 2; kk++)
                {
                    addGridBlock(initialGrid, ii, jj, kk);
                }
            }
        }
        for (int ii = 4-1; ii < 7+1; ii++)
        {
            for (int jj = -1+1; jj < 2-1; jj++)
            {
                
                for (int kk = -2+1; kk < 2-1; kk++)
                {
                    removeGridBlock(initialGrid, ii, jj, kk);
                }
            }
        }
        // Ground.
        int size{ 0 };
        for (int ii = -size; ii < size; ii++)
        {
            for (int jj = -size; jj < size; jj++)
            {
                for (int kk = -3; kk < -2; kk++)
                {
                    addGridBlock(initialGrid, ii, jj, kk);
                    std::cout << ii << std::endl;
                }
            }
        }
        
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

    void run() {
        m_gameBase->run();
    }

    // Helper method for setting up debug visualization
    void setupDebugVisualization() {
        m_debugViz = std::make_unique<DebugVisualization>(m_gameBase->m_graphicsEngine->m_meshHandler.get());
        m_gameBase->setDebugRenderer(m_debugViz.get());
    }

    void addGridBlock(Grid* grid, int x, int y, int z) {
        if (grid) grid->addCell(glm::ivec3(x, y, z));
    }
    
    void removeGridBlock(Grid* grid, int x, int y, int z) {
        if (grid) grid->removeCell(glm::ivec3(x, y, z));
    }

    // Override GraphicsEngine::CallBack methods
    virtual void preRenderCallback(uint64_t frameNum) override {
        // Process input BEFORE calling gamebase preRenderCallback
        processInputLogic();
        
        // Apply drag forces to all grids before physics update
        applyDragForces();
        
        // Call the game base preRenderCallback to handle physics and other updates
        m_gameBase->preRenderCallback(frameNum);
    }

    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override {
        //
    }

    virtual void framebufferSizeCallback(int width, int height) override {
        //
    }

    virtual void windowPosCallback(int xpos, int ypos) override {
        //
    }

private:
    void processInputLogic() {
        MouseHandler* mouseHandler = m_gameBase->m_graphicsEngine->m_mouseHandler;
        KeyboardHandler* keyboard = m_gameBase->m_graphicsEngine->m_keyboardHandler;
        
        // Camera movement speed
        const double mouseSensitivity = 0.002;
        
        // Calculate movement vectors based on camera orientation
        glm::dvec3 right = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(1.0, 0.0, 0.0);
        glm::dvec3 forward = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 up = m_gameBase->m_graphicsEngine->m_camOri * glm::dvec3(0.0, 0.0, 1.0);

        // Structural analysis with G key
        if (keyboard->m_g.justPressed()) {
            std::cout << "Visualizing structural analysis on " << m_gameBase->m_grids.size() << " grids..." << std::endl;
            
            for (const auto& grid : m_gameBase->m_grids) {
                grid->visualizeStructuralIntegrity();
            }
        }
        
        // Check for input actions that require grid traversal
        bool doCreate = mouseHandler->rightClick()
            || (mouseHandler->getRightDown() && mouseHandler->getTimeRightDown() > 32);
        bool doRemove = mouseHandler->leftClick()
            || (mouseHandler->getLeftDown() && mouseHandler->getTimeLeftDown() > 32);
        bool doForce = keyboard->m_f.isDown();
        bool doTrackSpeed = keyboard->m_z.justPressed();
        double forceMultiplier = (keyboard->m_f.timeDown() * 0.01 + 1.0);

        if (doCreate || doRemove || doForce || doTrackSpeed) {
            // Perform unified grid traversal for all actions
            Grid* targetGrid = nullptr;
            glm::ivec3 targetPos;
            glm::ivec3 hitPos;
            bool blockFound = false;
            double shortestSquaredDistance = DBL_MAX;
            
            // Camera position and direction
            glm::dvec3 startPos = m_gameBase->m_graphicsEngine->m_camPos;
            glm::dvec3 endPos = startPos + forward * 20.0; // Cast ray 20 units forward
            
            // Check all grids for ray intersections
            for (const auto& gridPtr : m_gameBase->m_grids) {
                Grid* grid = gridPtr.get();
                
                // Convert camera ray to grid space
                glm::dvec3 startPosLocal = grid->worldToGrid(startPos);
                glm::dvec3 endPosLocal = grid->worldToGrid(endPos);
                
                // Perform grid traversal
                std::vector<glm::ivec3> gridCells = grid->gridTraversal(startPosLocal, endPosLocal);
                
                // Check if ray hits any blocks in this grid
                for (size_t i = 1; i < gridCells.size(); i++) { // Skip first cell (where camera is)
                    if (grid->hasCell(gridCells[i])) {
                        // Calculate center of hit cube in world space
                        glm::dvec3 hitCubeCenter = glm::dvec3(gridCells[i]) + glm::dvec3(0.5, 0.5, 0.5);
                        glm::dvec3 hitCubeCenterWorld = grid->gridToWorld(hitCubeCenter);
                        
                        // Calculate squared distance from camera to cube center
                        glm::dvec3 distanceVec = hitCubeCenterWorld - startPos;
                        double squaredDistance = glm::dot(distanceVec, distanceVec);
                                
                        // Check if this is the closest hit so far
                        if (squaredDistance < shortestSquaredDistance) {
                            shortestSquaredDistance = squaredDistance;
                            targetGrid = grid;
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
                if (blockFound && targetGrid) {
                    RigidBody* body = targetGrid->getRigidBody();
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
                if (blockFound && targetGrid) {
                    RigidBody* body = targetGrid->getRigidBody();
                    if (body) {
                        // Apply force in the view direction
                        const double forceStrength = 0.001 * body->m_mass * forceMultiplier;
                        glm::dvec3 force = forward * forceStrength;
                        
                        // Apply the force at the camera position
                        glm::dvec3 applicationPoint = m_gameBase->m_graphicsEngine->m_camPos;
                        
                        // Apply force at the point
                        m_gameBase->m_physicsEngine->applyForceAtPoint(body, force, applicationPoint);
                        
                        std::cout << "Applied force to grid at distance: " << std::sqrt(shortestSquaredDistance) << std::endl;
                    }
                } else {
                    std::cout << "No target found for force application" << std::endl;
                }
            }
            
            if (doCreate) {
                if (blockFound && targetGrid) {
                    // Place block at the position before the hit
                    addGridBlock(targetGrid, targetPos.x, targetPos.y, targetPos.z);
                    std::cout << "Added block at (" << targetPos.x << ", " << targetPos.y << ", " << targetPos.z << ")" << std::endl;
                } else {
                    // No block found, create a new grid 2 units ahead
                    glm::dvec3 newGridPos = startPos + forward * 2.0 - glm::dvec3{0.5};
                    Grid* newGrid = m_gameBase->createGrid(newGridPos);
                    addGridBlock(newGrid, 0, 0, 0);  // Add initial block at grid center
                    std::cout << "Created new grid with block at world position (" 
                            << newGridPos.x << ", " << newGridPos.y << ", " << newGridPos.z << ")" << std::endl;
                }
            }
            
            if (doRemove) {
                if (blockFound && targetGrid) {
                    // Remove the hit block
                    removeGridBlock(targetGrid, hitPos.x, hitPos.y, hitPos.z);
                    std::cout << "Removed block at (" << hitPos.x << ", " << hitPos.y << ", " << hitPos.z << ")" << std::endl;

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
                    m_gameBase->scheduleGridSplitCheck(targetGrid, edgeCoords);
                    std::cout << "Scheduled grid split check for removed block at (" 
                          << hitPos.x << ", " << hitPos.y << ", " << hitPos.z << ")" << std::endl;
                    if (targetGrid->isEmpty()) {
                        m_gameBase->removeGrid(targetGrid);
                    }
                } else {
                    std::cout << "No block found to remove" << std::endl;
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

    void applyDragForces() {
        // Apply drag to all objects before running physics
        for (const auto& grid : m_gameBase->m_grids) {
            RigidBody* body = grid->getRigidBody();
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
};

int main() {
    try {
        // Create the TimeHandler with appropriate mode
        TimeHandler* timeHandler = new TimeHandler(TimeHandler::Mode::NONE);

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE;

        Game game(timeHandler, controlMode);
        game.run();
        
        // Clean up TimeHandler
        delete timeHandler;
    } catch (const std::bad_alloc& e) {
        std::cerr << "Out of memory: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}