#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <iostream>
#include <filesystem>

#include "src/GraphicsEngineBase.h"
#include "src/ShaderProgram.h"
#include "src/STBImageLoader.h"

// Global variables for the triangle
unsigned int triangleVAO, triangleVBO;
unsigned int textureID;

// Updated shader program for the triangle with texture support
const char* vertexShaderSource = "#version 330 core\n"
"layout (location = 0) in vec3 aPos;\n"
"layout (location = 1) in vec3 aColor;\n"
"layout (location = 2) in vec2 aTexCoord;\n"
"uniform mat4 view;\n"
"uniform mat4 projection;\n"
"out vec3 vertexColor;\n"
"out vec2 texCoord;\n"
"void main()\n"
"{\n"
"   gl_Position = projection * view * vec4(aPos, 1.0);\n"
"   vertexColor = aColor;\n"
"   texCoord = aTexCoord;\n"
"}\0";

const char* fragmentShaderSource = "#version 330 core\n"
"in vec3 vertexColor;\n"
"in vec2 texCoord;\n"
"out vec4 FragColor;\n"
"uniform sampler2D textureSampler;\n"
"uniform bool useTexture;\n"
"void main()\n"
"{\n"
"   if(useTexture) {\n"
"       vec4 texColor = texture(textureSampler, texCoord);\n"
"       FragColor = texColor * vec4(vertexColor, 1.0);\n"
"   } else {\n"
"       FragColor = vec4(vertexColor, 1.0);\n"
"   }\n"
"}\0";

// Camera/player variables
double moveSpeed = 0.1f;
double mouseSensitivity = 0.002f;

// Custom class for handling rendering callbacks
class TriangleRenderer : public GraphicsEngineBase::CallBack {
private:
    ShaderProgram shaderProgram;
    GraphicsEngineBase* engine;
    bool useTexture;
    
public:
    TriangleRenderer(GraphicsEngineBase* engineRef) : engine(engineRef), useTexture(false) {
        // Set up shader program
        shaderProgram.loadVertexShader(vertexShaderSource);
        shaderProgram.loadFragmentShader(fragmentShaderSource);
        shaderProgram.linkShaders();
        
        // Load texture using STBImageLoader
        loadTexture("../media/test_image.png");
        
        // Create triangle data with texture coordinates
        float vertices[] = {
             0.0f,  0.5f,  0.0f,    1.0f, 0.0f, 0.0f,    0.5f, 1.0f,  // top vertex (red)
            -0.5f, -0.5f,  0.0f,    0.0f, 1.0f, 0.0f,    0.0f, 0.0f,  // bottom left vertex (green)
             0.5f, -0.5f,  0.0f,    0.0f, 0.0f, 1.0f,    1.0f, 0.0f   // bottom right vertex (blue)
        };
        
        // Create buffers for the triangle
        glGenVertexArrays(1, &triangleVAO);
        glGenBuffers(1, &triangleVBO);
        
        glBindVertexArray(triangleVAO);
        glBindBuffer(GL_ARRAY_BUFFER, triangleVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
        
        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        
        // Color attribute
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        
        // Texture coordinate attribute
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);
        
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
    
    void loadTexture(const std::string& texturePath) {
        try {
            // Create the media directory if it doesn't exist
            std::filesystem::path mediaDir = "media";
            if (!std::filesystem::exists(mediaDir)) {
                std::filesystem::create_directory(mediaDir);
                std::cout << "Created media directory" << std::endl;
            }
            if (!std::filesystem::exists(texturePath)) {
                std::cout << "ERROR: File " << texturePath << " does not exist." << std::endl;
            }
            
            // Generate texture
            glGenTextures(1, &textureID);
            glBindTexture(GL_TEXTURE_2D, textureID);
            
            // Set texture wrapping/filtering options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            
            // Load image using STBImageLoader
            int width, height, nrChannels;
            unsigned char* data = STBImageLoader::load(true, texturePath, &width, &height, &nrChannels);
            
            if (data) {
                // Load texture data
                if (nrChannels == 3) {
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
                } else if (nrChannels == 4) {
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
                }
                
                glGenerateMipmap(GL_TEXTURE_2D);
                STBImageLoader::free(data);
                useTexture = true;
                std::cout << "Texture loaded successfully: " << texturePath << std::endl;
            } else {
                std::cerr << "Failed to load texture: " << texturePath << std::endl;
                useTexture = false;
            }
        } catch (const std::exception& e) {
            std::cerr << "Exception loading texture: " << e.what() << std::endl;
            useTexture = false;
        }
    }
    
    ~TriangleRenderer() {
        glDeleteVertexArrays(1, &triangleVAO);
        glDeleteBuffers(1, &triangleVBO);
        if (glIsTexture(textureID)) {
            glDeleteTextures(1, &textureID);
        }
    }
    
    virtual void preRenderCallback(uint64_t frameNum) override {
        // Process player movement before rendering
        handlePlayerMovement();
        
        // Toggle texture with T key
        if (engine->m_keyboardHandler->m_t.justPressed()) {
            useTexture = !useTexture;
            std::cout << "Texture " << (useTexture ? "enabled" : "disabled") << std::endl;
        }
    }
    
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override {
        // Convert double precision matrices to float precision
        glm::mat4 view = glm::mat4(viewMatrix);
        glm::mat4 projection = glm::mat4(projectionMatrix);
        
        // Render the triangle
        shaderProgram.use();
        
        // Set the view and projection matrices
        GLint viewLoc = glGetUniformLocation(shaderProgram.getID(), "view");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
        
        GLint projLoc = glGetUniformLocation(shaderProgram.getID(), "projection");
        glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
        
        // Set the texture
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glUniform1i(glGetUniformLocation(shaderProgram.getID(), "textureSampler"), 0);
        glUniform1i(glGetUniformLocation(shaderProgram.getID(), "useTexture"), useTexture);
        
        glBindVertexArray(triangleVAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
        glBindVertexArray(0);
    }
    
    virtual void framebufferSizeCallback(int width, int height) override {
        // Handle window resize
    }
    
    virtual void windowPosCallback(int xpos, int ypos) override {
        // Handle window position change
    }

private:
    // Handle player movement
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

        //std::cout << "x:" <<  engine->m_camOri.x << std::endl << "y:" <<  engine->m_camOri.y << std::endl << "z:" <<  engine->m_camOri.z << std::endl << "w:" <<  engine->m_camOri.w << std::endl << std::endl;
        
        // Toggle mouse lock with M key
        if (keyboardHandler->m_m.justPressed()) {
            mouseHandler->setMouseLock(!mouseHandler->getMouseLock());
        }

        glm::dvec3 right = engine->m_camOri * glm::dvec3(1.0f, 0.0f, 0.0f);
        glm::dvec3 forward = engine->m_camOri * glm::dvec3(0.0f, 1.0f, 0.0f);
        glm::dvec3 up = engine->m_camOri * glm::dvec3(0.0f, 0.0f, 1.0f);
        
        //std::cout << "x:" <<  forward.x << std::endl << "y:" <<  forward.y << std::endl << "z:" <<  forward.z << std::endl << std::endl << std::endl;

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
    engine.m_camPos = glm::dvec3(0.0, 0.0, 2.0);  // Position the camera to look down on the triangle
    engine.m_camOri = glm::dquat(1.0, 0.0, 0.0, 0.0);  // Initial orientation (no rotation)
    
    // Set field of view
    engine.m_fieldOfView = glm::radians(70.0);
    
    // Enable mouse lock for camera control
    engine.m_mouseHandler->setMouseLock(true);
    
    // Create the triangle renderer and add it as a callback
    TriangleRenderer triangleRenderer(&engine);
    engine.addCallbackObject(&triangleRenderer);
    
    // Start the render loop provided by GraphicsEngineBase
    engine.startRenderLoop();
    
    return 0;
}