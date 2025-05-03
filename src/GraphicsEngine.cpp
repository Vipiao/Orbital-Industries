// GraphicsEngine.cpp
#include "GraphicsEngine.h"
#include <iostream>
#include <filesystem>

GraphicsEngine::GraphicsEngine(
    int screenWidth,
    int screenHeight,
    const std::string& windowTitle,
    size_t maxTriangles,
    size_t maxMeshes,
    Mode mode)
    : GraphicsEngineBase(mode)
{
    m_screen_width = screenWidth;
    m_screen_height = screenHeight;
    
    if (!windowTitle.empty()) {
        glfwSetWindowTitle(m_window, windowTitle.c_str());
    }
    
    meshHandler = std::make_unique<MeshHandler>(maxTriangles, maxMeshes);
}

GraphicsEngine::~GraphicsEngine() {
    // Unique_ptr will handle cleanup
}

int GraphicsEngine::createMesh() {
    return meshHandler->addMesh();
}

void GraphicsEngine::updateMeshTransform(
    int meshId,
    const glm::dvec3& position,
    const glm::dvec3& velocity,
    const glm::dquat& orientation,
    const glm::dvec3& angVelAxis,
    double angVel,
    const glm::dvec3& centerOfRotation,
    int32_t colorTextureUnit,
    int32_t normalTextureUnit)
{
    currentTime = static_cast<uint64_t>(glfwGetTime() * 1000.0);
    
    meshHandler->updateMeshData(
        meshId,
        &position,
        &velocity,
        orientation,
        angVelAxis,
        angVel,
        centerOfRotation,
        colorTextureUnit,
        normalTextureUnit,
        currentTime
    );
}

void GraphicsEngine::removeMesh(int meshId) {
    meshHandler->removeMesh(meshId);
}

MeshHandler::Texture GraphicsEngine::createTexture(const std::string& texturePath) {
    return meshHandler->createTexture(texturePath);
}

int GraphicsEngine::loadModel(
    const std::string& modelPath,
    const std::string& colorTexturePath,
    const std::string& normalTexturePath,
    bool ignoreTextureCoordinates,
    int* outColorTextureUnit,
    int* outNormalTextureUnit)
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
        colorTextureUnit,
        normalTextureUnit
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
            std::vector<uint32_t> triangleIds = meshHandler->appendTrianglesToMesh(
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