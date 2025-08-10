// GraphicsEngine.h
#pragma once

#include "GraphicsEngineBase.h"
#include "MeshHandler.h"
#include "AssimpLoader.h"
#include "SSBOManager.h"
#include <memory>
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine : public GraphicsEngineBase {
public:
    GraphicsEngine(
        int screenWidth = 800,
        int screenHeight = 600,
        const std::string& windowTitle = "Graphics Engine",
        size_t maxTriangles = 10000,
        size_t maxMeshes = 100,
        Mode mode = Mode::NONE
    );
    
    ~GraphicsEngine();
    
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
};