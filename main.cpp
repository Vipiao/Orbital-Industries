#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <filesystem>

#include "src/GraphicsEngineBase.h"
#include "src/MeshHandler.h"

// Camera/player movement settings
double moveSpeed = 0.1f;
double mouseSensitivity = 0.002f;

// Custom renderer class that uses MeshHandler
class TriangleRenderer : public GraphicsEngineBase::CallBack {
private:
    MeshHandler* meshHandler;
    GraphicsEngineBase* engine;
    int triangleMeshId;
    glm::dvec3 lightPos;
    double rotationSpeed = 0.0f;
    
public:
    TriangleRenderer(GraphicsEngineBase* engineRef) : engine(engineRef) {
        // Create mesh handler with capacity for 100 triangles and 10 meshes
        meshHandler = new MeshHandler(100, 10);
        
        // Create a triangle mesh
        setupTriangleMesh();
        
        // Initialize light position
        lightPos = glm::dvec3(2.0, 2.0, 2.0);
    }
    
    ~TriangleRenderer() {
        delete meshHandler;
    }
    
    void setupTriangleMesh() {
        // Create a new mesh
        triangleMeshId = meshHandler->addMesh();
        
        // Define a colorful triangle with varying occlusion factors
        std::vector<glm::dvec3> vertices = {
            glm::dvec3(-0.5, -0.5, 0.0),  // Bottom left
            glm::dvec3(0.5, -0.5, 0.0),   // Bottom right
            glm::dvec3(0.0, 0.5, 0.0)     // Top center
        };
        
        // All normals pointing out of the screen
        std::vector<glm::dvec3> normals = {
            glm::dvec3(0.0, 0.0, 1.0),
            glm::dvec3(0.0, 0.0, 1.0),
            glm::dvec3(0.0, 0.0, 1.0)
        };
        
        // Tangents for normal mapping (pointing right)
        std::vector<glm::dvec3> tangents = {
            glm::dvec3(1.0, 0.0, 0.0),
            glm::dvec3(1.0, 0.0, 0.0),
            glm::dvec3(1.0, 0.0, 0.0)
        };
        
        // UV coordinates for texture mapping
        std::vector<glm::dvec2> uvs = {
            glm::dvec2(0.0, 0.0),  // Bottom left
            glm::dvec2(1.0, 0.0),  // Bottom right
            glm::dvec2(0.5, 1.0)   // Top center
        };
        
        // Create varying occlusion factors for demonstration
        std::vector<double> occlusionFactors = {
            1.0,  // Bottom left: fully lit
            0.6,  // Bottom right: partially occluded
            0.3   // Top center: more occluded
        };
        
        // Add triangle to mesh
        meshHandler->appendTrianglesToMesh(triangleMeshId, &vertices, &normals, &tangents, &uvs, &occlusionFactors);
        
        // Set up mesh position and orientation
        glm::dvec3 position(0.0, 0.0, 0.0);
        glm::dvec3 velocity(0.0, 0.0, 0.0);
        glm::dquat orientation(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
        glm::dvec3 angVelAxis(0.0, 0.0, 1.0);        // Rotate around Z-axis
        double angVel = rotationSpeed;                // Rotation speed
        glm::dvec3 centerOfRotation(0.0, 0.0, 0.0);
        
        // Update mesh data - this initializes the mesh's position and rotation
        meshHandler->updateMeshData(
            triangleMeshId, 
            &position, &velocity, 
            orientation, 
            angVelAxis, angVel, 
            centerOfRotation,
            -1, -1,      // No textures for this demo
            static_cast<uint64_t>(glfwGetTime() * 1000.0)  // Current time
        );
    }
    
    virtual void preRenderCallback(uint64_t frameNum) override {
        // Process player movement
        handlePlayerMovement();
        
        // Animate light position for visual effect
        double time = frameNum * 0.0001;
        lightPos = glm::dvec3(
            2.0 * sin(time),      // Circle around X
            2.0 * cos(time),      // Circle around Y
            1.0 + sin(time * 0.5) // Subtle up/down movement
        );
    }
    
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override {
        // Convert double precision matrices to float precision
        glm::mat4 view = glm::mat4(viewMatrix);
        glm::mat4 projection = glm::mat4(projectionMatrix);
        
        // Render the triangle using MeshHandler's single-pass render method
        meshHandler->render(
            view, projection, 
            engine->m_frameNum,     // frame number
            static_cast<uint64_t>(glfwGetTime() * 1000.0),  // time in milliseconds
            fmod(glfwGetTime() * 1000.0, 1.0),  // time remainder (fractional part)
            lightPos,               // animated light position
            engine->m_camPos        // camera position
        );
    }
    
    virtual void framebufferSizeCallback(int width, int height) override {
        // Handle window resize - nothing needed here
    }
    
    virtual void windowPosCallback(int xpos, int ypos) override {
        // Handle window position change - nothing needed here
    }
    
private:
    // Handle player movement - adapted from your original code
    void handlePlayerMovement() {
        // Get the mouse handler and keyboard handler from the engine
        MouseHandler* mouseHandler = engine->m_mouseHandler;
        KeyboardHandler* keyboardHandler = engine->m_keyboardHandler;
        
        // Mouse movement for camera orientation
        glm::dvec2 mouseMovement = mouseHandler->getMouseMovement();
        
        if (mouseHandler->getMouseLock()) {
            // Adjust camera orientation based on mouse movement
            glm::dvec3 rotAxis = glm::dvec3(0.0, 0.0, 1.0);  // Z-axis for yaw
            double yawAngle = -mouseMovement.x * mouseSensitivity;
            glm::dquat yawQuat = glm::angleAxis(yawAngle, rotAxis);
            
            rotAxis = glm::dvec3(1.0, 0.0, 0.0);  // X-axis for pitch
            double pitchAngle = -mouseMovement.y * mouseSensitivity;
            glm::dquat pitchQuat = glm::angleAxis(pitchAngle, rotAxis);
            
            // Apply the rotation to the camera orientation
            engine->m_camOri = engine->m_camOri * yawQuat * pitchQuat;
            engine->m_camOri = glm::normalize(engine->m_camOri);
        }
        
        // Toggle mouse lock with M key
        if (keyboardHandler->m_m.justPressed()) {
            mouseHandler->setMouseLock(!mouseHandler->getMouseLock());
        }

        // Get movement directions from camera orientation
        glm::dvec3 right = engine->m_camOri * glm::dvec3(1.0f, 0.0f, 0.0f);
        glm::dvec3 forward = engine->m_camOri * glm::dvec3(0.0f, 1.0f, 0.0f);
        glm::dvec3 up = engine->m_camOri * glm::dvec3(0.0f, 0.0f, 1.0f);
        
        // Normalize the vectors
        forward = glm::normalize(forward);
        right = glm::normalize(right);
        up = glm::normalize(up);
        
        // WASD movement
        glm::dvec3 moveDirection(0.0);
        
        if (keyboardHandler->m_w.isDown()) {
            moveDirection += forward;
        }
        if (keyboardHandler->m_s.isDown()) {
            moveDirection -= forward;
        }
        if (keyboardHandler->m_a.isDown()) {
            moveDirection -= right;
        }
        if (keyboardHandler->m_d.isDown()) {
            moveDirection += right;
        }
        
        // Up/down movement with space and left shift
        if (keyboardHandler->m_space.isDown()) {
            moveDirection += up;
        }
        if (keyboardHandler->m_lShift.isDown()) {
            moveDirection -= up;
        }
        
        // Normalize the movement direction if not zero
        if (glm::length(moveDirection) > 0.0) {
            moveDirection = glm::normalize(moveDirection) * moveSpeed;
        }
        
        // Update camera position
        engine->m_camPos += moveDirection;
    }
};

int main() {
    // Create the graphics engine
    GraphicsEngineBase engine(GraphicsEngineBase::Mode::NONE);
    
    // Initialize camera position and orientation
    engine.m_camPos = glm::dvec3(0.0, 0.0, 3.0);  // Position camera to view the triangle
    engine.m_camOri = glm::dquat(1.0, 0.0, 0.0, 0.0);  // Initial orientation (no rotation)
    engine.m_camOri = glm::rotate(engine.m_camOri, glm::radians(-90.), glm::dvec3{1,0,0});
    
    // Set field of view
    engine.m_fieldOfView = glm::radians(60.0);
    
    // Enable mouse lock for camera control
    engine.m_mouseHandler->setMouseLock(true);
    
    // Print instructions
    std::cout << "Triangle Rendering Demo" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  WASD: Move camera" << std::endl;
    std::cout << "  Mouse: Look around" << std::endl;
    std::cout << "  Space/Shift: Move up/down" << std::endl;
    std::cout << "  M: Toggle mouse lock" << std::endl;
    
    // Create the triangle renderer and add it as a callback
    TriangleRenderer triangleRenderer(&engine);
    engine.addCallbackObject(&triangleRenderer);
    
    // Start the render loop provided by GraphicsEngineBase
    engine.startRenderLoop();
    
    return 0;
}