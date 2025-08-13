// GraphicsEngine.h
#pragma once

#include "GraphicsEngineBase.h"
#include "GraphicsCallbacks.h"
#include "CallbackManager.h"
#include "CallbackManager.h"
#include "MeshHandler.h"
#include "AssimpLoader.h"
#include "SSBOManager.h"
#include <memory>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine : public IGraphicsCallbacks, public CallbackManager {
public:
    GraphicsEngine(
        int screenWidth = 800,
        int screenHeight = 600,
        const std::string& windowTitle = "Graphics Engine",
        size_t maxTriangles = 10000,
        size_t maxMeshes = 100,
        GraphicsEngineBase::Mode mode = GraphicsEngineBase::Mode::NONE
    );
    
    ~GraphicsEngine();

    // IGraphicsCallbacks implementation
    virtual void preRenderCallback(uint64_t frameNum) override;
    virtual void renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) override;
    virtual void framebufferSizeCallback(int width, int height) override;
    virtual void windowPosCallback(int xpos, int ypos) override;
    
    // Access to GraphicsEngineBase properties
    GLFWwindow* getWindow() { return getGraphicsEngineBase()->m_window; }
    unsigned int getScreenWidth() { return getGraphicsEngineBase()->m_screen_width; }
    unsigned int getScreenHeight() { return getGraphicsEngineBase()->m_screen_height; }
    glm::dvec3& getCamPos() { return getGraphicsEngineBase()->m_camPos; }
    glm::dvec3& getCamVel() { return getGraphicsEngineBase()->m_camVel; }
    int getFrameRate() { return getGraphicsEngineBase()->m_frameRate; }
    glm::dquat& getCamOri() { return getGraphicsEngineBase()->m_camOri; }
    uint64_t getFrameNum() { return getGraphicsEngineBase()->m_frameNum; }
    double& getFieldOfView() { return getGraphicsEngineBase()->m_fieldOfView; }
    MouseHandler* getMouseHandler() { return getGraphicsEngineBase()->m_mouseHandler; }
    KeyboardHandler* getKeyboardHandler() { return getGraphicsEngineBase()->m_keyboardHandler; }
    
    // Graphics engine functionality
    void startRenderLoop();
    void setTriangleRenderMode(bool useTriangles);
    bool getTriangleRenderMode();
    
    int createMesh();
    
    void updateMeshTransform(
        int meshId,
        const glm::dvec3& position,
        const glm::dvec3& velocity,
        const glm::dquat& orientation,
        const glm::dvec3& angVelAxis,
        double angVel,
        const glm::dvec3& centerOfRotation,
        const glm::dvec3& scale = glm::dvec3(1.0, 1.0, 1.0),
        int32_t colorTextureUnit = -1,
        int32_t normalTextureUnit = -1,
        uint64_t physicsTimeStep = 0
    );
    
    void removeMesh(int meshId);
    
    MeshHandler::Texture createTexture(const std::string& texturePath);
    
    int loadModel(
        const std::string& modelPath,
        const std::string& colorTexturePath = "",
        const std::string& normalTexturePath = "",
        bool ignoreTextureCoordinates = false,
        int* outColorTextureUnit = nullptr,
        int* outNormalTextureUnit = nullptr
    );
    
    std::vector<uint32_t> loadModelIntoMesh(
        int meshId,
        const std::string& modelPath,
        bool ignoreTextureCoordinates = false
    );
    
    std::unique_ptr<SSBOManager> m_ssboManager;
    std::unique_ptr<MeshHandler> m_meshHandler;
    uint64_t currentTime{0};

private:
    std::shared_ptr<GraphicsEngineBase> m_graphicsEngineBase;
    
    GraphicsEngineBase* getGraphicsEngineBase() const;
};