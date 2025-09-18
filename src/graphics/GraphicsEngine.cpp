// GraphicsEngine.cpp
#include "GraphicsEngine.h"
#include "GraphicsEngineBase.h"
#include <iostream>
#include <filesystem>

GraphicsEngine::GraphicsEngine(
    int screenWidth,
    int screenHeight,
    const std::string& windowTitle,
    size_t maxTriangles,
    size_t maxMeshes,
    GraphicsEngineBase::Mode mode)
{
    // Create GraphicsEngineBase
    m_graphicsEngineBase = std::make_shared<GraphicsEngineBase>(mode);
    
    // Register self as callback with GraphicsEngineBase
    m_graphicsEngineBase->addCallback(this);
    
    // Configure window
    m_graphicsEngineBase->m_screen_width = screenWidth;
    m_graphicsEngineBase->m_screen_height = screenHeight;
    
    if (!windowTitle.empty()) {
        glfwSetWindowTitle(m_graphicsEngineBase->m_window, windowTitle.c_str());
    }

    // Create SSBO manager and pass to mesh handler
    m_ssboManager = std::make_unique<SSBOManager>(maxMeshes);
    m_meshHandler = std::make_unique<MeshHandler>(maxTriangles, m_ssboManager.get());

    // Create instance handler
    m_instanceHandler = std::make_unique<InstanceHandler>(m_ssboManager.get());

    // Create 2D mesh manager
    m_meshManager2D = std::make_unique<MeshManager2D>(1000);
}

GraphicsEngine::~GraphicsEngine() {
    // Unregister from GraphicsEngineBase
    if (auto base = m_graphicsEngineBase.get()) {
        base->removeCallback(this);
    }
}

GraphicsEngineBase* GraphicsEngine::getGraphicsEngineBase() const {
    return m_graphicsEngineBase.get();
}

void GraphicsEngine::preRenderCallback(uint64_t frameNum) {
    // Call registered callbacks first
    callPreRenderCallbacks(frameNum);
    
    // No additional logic needed here for GraphicsEngine itself
}

void GraphicsEngine::renderCallback(glm::dmat4 viewMatrix, glm::dmat4 projectionMatrix) {
    // Call registered callbacks first
    callRenderCallbacks(viewMatrix, projectionMatrix);
    
    // GraphicsEngine's own render logic
    // Convert double precision matrices to float precision
    glm::mat4 view = glm::mat4(viewMatrix);
    glm::mat4 projection = glm::mat4(projectionMatrix);
    
    // Render using MeshHandler's single-pass render method
    m_meshHandler->render(
        view, projection, 
        getFrameNum(),                    // frame number
        m_currentPhysicsTimeStep,         // physics time step
        m_physicsTimeRemainder,           // time remainder (fractional part)
        {4.0, 4.0, 4.0},                  // hardcoded light position
        getCamPos()                       // camera position
    );

    // Render instanced geometry
    m_instanceHandler->render(
        view, projection,
        getFrameNum(),                    // frame number
        m_currentPhysicsTimeStep,         // physics time step
        m_physicsTimeRemainder,           // time remainder (fractional part)
        {4.0, 4.0, 4.0},                  // hardcoded light position
        getCamPos()                       // camera position
    );

    // Render 2D overlay
    float aspectRatio = getScreenWidth() / (float)getScreenHeight();
    glm::mat4 projection2D = glm::ortho(-1.0f, 1.0f, -1.0f/aspectRatio, 1.0f/aspectRatio, 0.0f, 1.0f);
    m_meshManager2D->render(projection2D);
}

void GraphicsEngine::setRenderParameters(uint64_t physicsTimeStep, double timeRemainder) {
    m_currentPhysicsTimeStep = physicsTimeStep;
    m_physicsTimeRemainder = timeRemainder;
}

void GraphicsEngine::framebufferSizeCallback(int width, int height) {
    // Call registered callbacks first
    callFramebufferSizeCallbacks(width, height);
    
    // No additional logic needed here for GraphicsEngine itself
}

void GraphicsEngine::windowPosCallback(int xpos, int ypos) {
    // Call registered callbacks first  
    callWindowPosCallbacks(xpos, ypos);
    
    // No additional logic needed here for GraphicsEngine itself
}

void GraphicsEngine::startRenderLoop() {
    m_graphicsEngineBase->startRenderLoop();
}

void GraphicsEngine::setTriangleRenderMode(bool useTriangles) {
    m_graphicsEngineBase->setTriangleRenderMode(useTriangles);
}

bool GraphicsEngine::getTriangleRenderMode() {
    return m_graphicsEngineBase->getTriangleRenderMode();
}

int GraphicsEngine::createMesh() {
    return m_meshHandler->addMesh();
}

void GraphicsEngine::updateMeshTransform(
    int meshId,
    const glm::dvec3& position,
    const glm::dvec3& velocity,
    const glm::dquat& orientation,
    const glm::dvec3& angVelAxis,
    double angVel,
    const glm::dvec3& centerOfRotation,
    const glm::dvec3& scale,
    int32_t colorTextureUnit,
    int32_t normalTextureUnit,
    int32_t materialTextureUnit,
    uint64_t physicsTimeStep,
    double emissiveScalar)
{
    m_ssboManager->updateMeshTransform(
        meshId,
        position,
        velocity,
        orientation,
        angVelAxis,
        angVel,
        centerOfRotation,
        scale,
        colorTextureUnit,
        normalTextureUnit,
        materialTextureUnit,
        physicsTimeStep,
        emissiveScalar
    );
}

void GraphicsEngine::removeMesh(int meshId) {
    m_meshHandler->removeMesh(meshId);
}

MeshHandler::Texture GraphicsEngine::createTexture(const std::string& texturePath) {
    return m_meshHandler->createTexture(texturePath);
}

int GraphicsEngine::loadModel(
    const std::string& modelPath,
    const std::string& colorTexturePath,
    const std::string& normalTexturePath,
    const std::string& materialTexturePath,
    bool ignoreTextureCoordinates,
    int* outColorTextureUnit,
    int* outNormalTextureUnit,
    int* outMaterialTextureUnit)
{
    int meshId = createMesh();
    if (meshId < 0) {
        std::cerr << "Failed to create mesh for model: " << modelPath << std::endl;
        return -1;
    }
    
    if (loadModelIntoMesh(meshId, modelPath, ignoreTextureCoordinates).empty()) {
        removeMesh(meshId);
        return -1;
    }
    
    int32_t colorTextureUnit = -1;
    int32_t normalTextureUnit = -1;
    int32_t materialTextureUnit = -1;
    
    if (!colorTexturePath.empty()) {
        try {
            MeshHandler::Texture colorTexture = createTexture(colorTexturePath);
            colorTextureUnit = colorTexture.m_textureUnit;
            
            // Store the color texture unit in the output parameter if provided
            if (outColorTextureUnit != nullptr) {
                *outColorTextureUnit = colorTextureUnit;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to load color texture: " << e.what() << std::endl;
        }
    }
    
    if (!normalTexturePath.empty()) {
        try {
            MeshHandler::Texture normalTexture = createTexture(normalTexturePath);
            normalTextureUnit = normalTexture.m_textureUnit;
            
            // Store the normal texture unit in the output parameter if provided
            if (outNormalTextureUnit != nullptr) {
                *outNormalTextureUnit = normalTextureUnit;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to load normal texture: " << e.what() << std::endl;
        }
    }

    if (!materialTexturePath.empty()) {
        try {
            MeshHandler::Texture materialTexture = createTexture(materialTexturePath);
            materialTextureUnit = materialTexture.m_textureUnit;
            
            // Store the material texture unit in the output parameter if provided
            if (outMaterialTextureUnit != nullptr) {
                *outMaterialTextureUnit = materialTextureUnit;
            }
        } catch (const std::exception& e) {
            std::cerr << "Failed to load material texture: " << e.what() << std::endl;
        }
    }
    
    glm::dvec3 position(0.0, 0.0, 0.0);
    glm::dvec3 velocity(0.0, 0.0, 0.0);
    glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
    glm::dvec3 angVelAxis(0.0, 1.0, 0.0);
    double angVel = 0.0;
    glm::dvec3 centerOfRotation(0.0, 0.0, 0.0);
    
    updateMeshTransform(
        meshId,
        position,
        velocity,
        orientation,
        angVelAxis,
        angVel,
        centerOfRotation,
        glm::dvec3(1.0, 1.0, 1.0), // Default scale
        colorTextureUnit,
        normalTextureUnit,
        materialTextureUnit,
        0,                          // Default physics time step
        1.0                         // Default emissive scalar
    );
    
    return meshId;
}

std::vector<uint32_t> GraphicsEngine::loadModelIntoMesh(
    int meshId,
    const std::string& modelPath,
    bool ignoreTextureCoordinates)
{
    if (!std::filesystem::exists(modelPath)) {
        std::cerr << "Model file not found: " << modelPath << std::endl;
        return {};
    }
    
    std::vector<uint32_t> allTriangleIds;
    
    try {
        std::vector<AssetMeshData> meshes;
        AssimpLoader::load(modelPath, &meshes, ignoreTextureCoordinates);
        
        for (const AssetMeshData& mesh : meshes) {
            std::vector<glm::dvec3> positions;
            std::vector<glm::dvec3> normals;
            std::vector<glm::dvec3> tangents;
            std::vector<glm::dvec2> uvs;
            
            if (!mesh.indices.empty()) {
                for (size_t i = 0; i < mesh.indices.size(); i++) {
                    int idx = mesh.indices[i];
                    
                    if (idx < mesh.positionsData.size()) {
                        const auto& pos = mesh.positionsData[idx];
                        const auto& norm = mesh.normalsData[idx];
                        const auto& tang = mesh.tangentsData[idx];
                        const auto& texUV = mesh.uvsData[idx];
                        
                        positions.push_back(glm::dvec3(pos[0], pos[1], pos[2]));
                        normals.push_back(glm::dvec3(norm[0], norm[1], norm[2]));
                        tangents.push_back(glm::dvec3(tang[0], tang[1], tang[2]));
                        uvs.push_back(glm::dvec2(texUV[0], texUV[1]));
                    }
                }
            } else {
                for (size_t i = 0; i < mesh.positionsData.size(); i++) {
                    const auto& pos = mesh.positionsData[i];
                    const auto& norm = mesh.normalsData[i];
                    const auto& tang = mesh.tangentsData[i];
                    const auto& texUV = mesh.uvsData[i];
                    
                    positions.push_back(glm::dvec3(pos[0], pos[1], pos[2]));
                    normals.push_back(glm::dvec3(norm[0], norm[1], norm[2]));
                    tangents.push_back(glm::dvec3(tang[0], tang[1], tang[2]));
                    uvs.push_back(glm::dvec2(texUV[0], texUV[1]));
                }
            }
            
            if (positions.empty()) {
                continue;
            }
            std::vector<uint32_t> triangleIds = m_meshHandler->appendTrianglesToMesh(
                meshId,
                &positions,
                &normals,
                &tangents,
                &uvs
            );
            
            allTriangleIds.insert(allTriangleIds.end(), triangleIds.begin(), triangleIds.end());
        }
    } catch (const std::exception& e) {
        std::cerr << "Error loading model: " << e.what() << std::endl;
        return {};
    }
    
    return allTriangleIds;
}