#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <filesystem>
#include <vector>

#include "src/GraphicsEngineBase.h"
#include "src/MeshHandler.h"
#include "src/STBImageLoader.h"
#include "src/AssimpLoader.h"

// Camera/player movement settings
double moveSpeed = 0.02;
double mouseSensitivity = 0.002;

// Custom renderer class that uses MeshHandler
class ModelRenderer : public GraphicsEngineBase::CallBack {
private:
    MeshHandler* meshHandler;
    GraphicsEngineBase* engine;
    int modelMeshId;
    glm::dvec3 lightPos;
    double rotationSpeed = 0.001*0.;
    
public:
    ModelRenderer(GraphicsEngineBase* engineRef) : engine(engineRef) {
        // Create mesh handler with capacity for 10000 triangles and 10 meshes
        meshHandler = new MeshHandler(10000, 10);
        
        // Create a model mesh using Assimp
        loadModel();
        
        // Initialize light position
        lightPos = glm::dvec3(2.0, 2.0, 2.0);
    }
    
    ~ModelRenderer() {
        delete meshHandler;
    }
    
    void loadModel() {
        // Create a new mesh
        modelMeshId = meshHandler->addMesh();
        
        // Load model using AssimpLoader
        std::string modelPath = "../media/blender/00_cube.obj"; // Path to your model file
        
        std::vector<AssetMeshData> meshes;
        
        // Use AssimpLoader to load the model with the renamed structure
        AssimpLoader::load(modelPath, &meshes);
        
        // Process each mesh
        for (const AssetMeshData& mesh : meshes) {
            // Convert data to GLM vectors for MeshHandler
            std::vector<glm::dvec3> positions;
            std::vector<glm::dvec3> normals;
            std::vector<glm::dvec3> tangents;
            std::vector<glm::dvec2> uvs;
            
            // Convert positions (x,y,z triplets)
            for (const auto& posData : mesh.positionsData) {
                positions.push_back(glm::dvec3(posData[0], posData[1], posData[2]));
            }
            
            // Convert normals (x,y,z triplets)
            for (const auto& normData : mesh.normalsData) {
                normals.push_back(glm::dvec3(normData[0], normData[1], normData[2]));
            }
            
            // Convert tangents (x,y,z triplets)
            for (const auto& tanData : mesh.tangentsData) {
                tangents.push_back(glm::dvec3(tanData[0], tanData[1], tanData[2]));
            }
            
            // Convert UVs (u,v pairs)
            for (const auto& uvData : mesh.uvsData) {
                uvs.push_back(glm::dvec2(uvData[0], uvData[1]));
            }
            
            // Use indices from Assimp to reorder vertices if needed
            if (!mesh.indices.empty()) {
                std::vector<glm::dvec3> indexedPositions;
                std::vector<glm::dvec3> indexedNormals;
                std::vector<glm::dvec3> indexedTangents;
                std::vector<glm::dvec2> indexedUvs;
                
                // Process triplets of indices (triangles)
                for (size_t i = 0; i < mesh.indices.size(); i += 3) {
                    // For each triangle (3 vertices)
                    for (int j = 0; j < 3; j++) {
                        int idx = mesh.indices[i + j];
                        
                        // Ensure the index is valid
                        if (idx < positions.size()) {
                            indexedPositions.push_back(positions[idx]);
                            indexedNormals.push_back(normals[idx]);
                            indexedTangents.push_back(tangents[idx]);
                            indexedUvs.push_back(uvs[idx]);
                        }
                    }
                }
                
                // Add indexed triangles to the mesh
                if (!indexedPositions.empty()) {
                    meshHandler->appendTrianglesToMesh(
                        modelMeshId, 
                        &indexedPositions, 
                        &indexedNormals, 
                        &indexedTangents, 
                        &indexedUvs
                    );
                }
            } else {
                // No indices - just add the vertices directly
                meshHandler->appendTrianglesToMesh(
                    modelMeshId, 
                    &positions, 
                    &normals, 
                    &tangents, 
                    &uvs
                );
            }
        }
        
        // Set up mesh position and orientation
        glm::dvec3 position(0.0, 0.0, 0.0);
        glm::dvec3 velocity(0.0, 0.0, 0.0);
        glm::dquat orientation(1.0, 0.0, 0.0, 0.0);  // Identity quaternion
        glm::dvec3 angVelAxis(1.0, 1.0, 1.0);        // Rotate around combined axis
        angVelAxis = glm::normalize(angVelAxis);
        double angVel = rotationSpeed;                // Rotation speed
        glm::dvec3 centerOfRotation(0., 0., 0.);

        // After creating your mesh and adding triangles, load the textures
        MeshHandler::Texture colorTexture = meshHandler->createTexture("../media/color_512x512_occluded.png");
        MeshHandler::Texture normalTexture = meshHandler->createTexture("../media/normal_combined_512x512.png");

        // Store the texture units for later use
        int colorTextureUnit = colorTexture.m_textureUnit;  
        int normalTextureUnit = normalTexture.m_textureUnit;
        
        // Update mesh data - this initializes the mesh's position and rotation
        meshHandler->updateMeshData(
            modelMeshId, 
            &position, &velocity, 
            orientation, 
            angVelAxis, angVel, 
            centerOfRotation,
            colorTextureUnit, normalTextureUnit,
            static_cast<uint64_t>(glfwGetTime() * 1000.0)  // Current time
        );
    }
    
    virtual void preRenderCallback(uint64_t frameNum) override {
        // Process player movement
        handlePlayerMovement();
        
        // Animate light position for visual effect
        //double time = frameNum * 0.0001;
        //lightPos = glm::dvec3(
        //    2.0 * sin(time),      // Circle around X
        //    2.0 * cos(time),      // Circle around Y
        //    1.0 + sin(time * 0.5) // Subtle up/down movement
        //);
    }
    
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override {
        // Convert double precision matrices to float precision
        glm::mat4 view = glm::mat4(viewMatrix);
        glm::mat4 projection = glm::mat4(projectionMatrix);
        
        // Render the model using MeshHandler's single-pass render method
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
    engine.m_camPos = glm::dvec3(0.0, -2.0, 0.0);  // Position camera to view the model
    engine.m_camOri = glm::dquat(1.0, 0.0, 0.0, 0.0);  // Initial orientation (no rotation)
    engine.m_camOri = glm::rotate(engine.m_camOri, glm::radians(0.0), glm::dvec3{1,0,0});
    
    // Set field of view
    engine.m_fieldOfView = glm::radians(100.0);
    
    // Enable mouse lock for camera control
    engine.m_mouseHandler->setMouseLock(true);
    
    // Print instructions
    std::cout << "3D Model Rendering Demo with Tangents" << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  WASD: Move camera" << std::endl;
    std::cout << "  Mouse: Look around" << std::endl;
    std::cout << "  Space/Shift: Move up/down" << std::endl;
    std::cout << "  M: Toggle mouse lock" << std::endl;
    
    // Create the model renderer and add it as a callback
    ModelRenderer modelRenderer(&engine);
    engine.addCallbackObject(&modelRenderer);
    
    // Start the render loop provided by GraphicsEngineBase
    engine.startRenderLoop();
    
    return 0;
}