// main.cpp
#include "src/GameBase.h"
#include "src/TimeHandler.h"
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class MyGame : public GameBase {
public:
    MyGame(TimeHandler* timeHandler, 
           GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE) 
      : GameBase(800, 600, "3D Grid Demo", timeHandler, controlMode) {
        // Set up initial camera position and orientation
        graphicsEngine->m_camPos = glm::dvec3(0, 0, 0);
        graphicsEngine->m_camOri = glm::angleAxis(glm::radians(0.0), glm::dvec3(1, 0, 0));
        graphicsEngine->m_fieldOfView = glm::radians(90.0);
        
        // Enable mouse lock for camera control
        graphicsEngine->m_mouseHandler->setMouseLock(true);
        
        // Create a center grid that will be our player object
        m_playerGrid = createGrid(glm::dvec3(0, 0, 0));

        addGridBlock(1, 0, 0);  // Block to the right
        addGridBlock(0, 0, 0);  // Center block
        //addGridBlock(0, 1, 0);  // Block to the front
        //addGridBlock(0, 0, 1);  // Block above
        
        PhysicsEngine::RigidBody* body = physicsEngine->getRigidBody(m_playerGrid->getRigidBodyId());
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

    void addGridBlock(int x, int y, int z) {
        if (m_playerGrid) {
            m_playerGrid->addCell(glm::ivec3(x, y, z));
            std::cout << "Added block at (" << x << "," << y << "," << z << ")" << std::endl;
        }
    }
    
    // New - Method to remove a block from the grid
    void removeGridBlock(int x, int y, int z) {
        if (m_playerGrid) {
            m_playerGrid->removeCell(glm::ivec3(x, y, z));
            std::cout << "Removed block at (" << x << "," << y << "," << z << ")" << std::endl;
        }
    }
    
protected:
    double m_moveSpeed = 0.05;

    virtual void processInput() override {
        MouseHandler* mouseHandler = graphicsEngine->m_mouseHandler;
        KeyboardHandler* keyboard = graphicsEngine->m_keyboardHandler;
        
        // Camera movement speed
        const double mouseSensitivity = 0.002;
        
        // Calculate movement vectors based on camera orientation
        glm::dvec3 right = graphicsEngine->m_camOri * glm::dvec3(1.0, 0.0, 0.0);
        glm::dvec3 forward = graphicsEngine->m_camOri * glm::dvec3(0.0, 1.0, 0.0);
        glm::dvec3 up = graphicsEngine->m_camOri * glm::dvec3(0.0, 0.0, 1.0);
        
        // Add/remove blocks with E/Q keys
        if (m_playerGrid) {
            glm::dvec3 pos{graphicsEngine->m_camPos + forward * 2.};
            PhysicsEngine::RigidBody* body = physicsEngine->getRigidBody(m_playerGrid->getRigidBodyId());
            glm::i64vec3 posI{glm::floor(glm::conjugate(body->orientation) * (pos - body->position) + m_playerGrid->m_centerOfMass)};
            if (keyboard->m_t.justPressed()) {
                addGridBlock(posI.x, posI.y, posI.z);
            }
            
            if (keyboard->m_r.justPressed()) {
                removeGridBlock(posI.x, posI.y, posI.z);
            }
        }

        // Handle grid force application with F key
        if (keyboard->m_f.isDown()) {
            // Get player rigid body
            PhysicsEngine::RigidBody* body = physicsEngine->getRigidBody(m_playerGrid->getRigidBodyId());
            if (body) {
                // Apply an upward force when F is pressed
                const double forceStrength = 0.001;
                glm::dvec3 force = forward * forceStrength;
                
                // Apply the force at a point slightly offset from center
                // This will create both linear movement and rotation
                glm::dvec3 applicationPoint = graphicsEngine->m_camPos;
                
                // Apply force at the point
                physicsEngine->applyForceAtPoint(body->id, force, applicationPoint);
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
            glm::dvec3 rotAxis = glm::dvec3(0.0, 0.0, 1.0);
            double yawAngle = -mouseMovement.x * mouseSensitivity;
            glm::dquat yawQuat = glm::angleAxis(yawAngle, rotAxis);
            
            // Rotate around X-axis for pitch (up/down)
            rotAxis = glm::dvec3(1.0, 0.0, 0.0);
            double pitchAngle = -mouseMovement.y * mouseSensitivity;
            glm::dquat pitchQuat = glm::angleAxis(pitchAngle, rotAxis);
            
            // Apply rotations to camera orientation
            graphicsEngine->m_camOri = graphicsEngine->m_camOri * yawQuat * pitchQuat;
            graphicsEngine->m_camOri = glm::normalize(graphicsEngine->m_camOri);
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
            graphicsEngine->m_camPos += moveDirection;
        }
    }
    
    virtual void updatePhysics() override {
        // Apply drag to all objects before running physics
        for (const auto& grid : grids) {
            PhysicsEngine::RigidBody* body = physicsEngine->getRigidBody(grid->getRigidBodyId());
            if (body && !body->isStatic) {
                // Simple drag force calculation: -dragCoefficient * velocity
                const double dragCoefficient = 0.04;
                
                // Apply drag to linear velocity
                if (glm::length(body->velocity) > 0.0) {
                    glm::dvec3 dragForce = -dragCoefficient * body->velocity * body->mass;
                    physicsEngine->applyForce(body->id, dragForce);
                }
                
                // Apply drag to angular velocity
                if (glm::length(body->angularVelocity) > 0.0) {
                    glm::dvec3 angularDrag = -dragCoefficient * body->angularVelocity * body->momentOfInertia;
                    physicsEngine->applyTorque(body->id, angularDrag);
                }
            }
        }
        
        // Call the base class implementation to run the physics simulation
        GameBase::updatePhysics();
    }
    
private:
    Grid* m_playerGrid; // Reference to the player's grid object
};

int main() {
    try {
        // Create the TimeHandler with appropriate mode
        TimeHandler* timeHandler = new TimeHandler(TimeHandler::Mode::NONE);

        // Use existing GraphicsEngineBase::Mode for controls
        GraphicsEngineBase::Mode controlMode = GraphicsEngineBase::Mode::NONE;

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