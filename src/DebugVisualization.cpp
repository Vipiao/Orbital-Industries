// DebugVisualization.cpp
#include "DebugVisualization.h"
#include <iostream>
#include <algorithm>

DebugVisualization::DebugVisualization(MeshHandler* meshHandler)
    : m_meshHandler(meshHandler)
    , m_sphereMeshLoaded(false)
    , m_redTextureUnit(-1)
    , m_redTextureLoaded(false)
{
    if (!m_meshHandler) {
        throw std::runtime_error("MeshHandler pointer cannot be null");
    }
    
    loadSphereMeshData();
    loadRedTexture();
}

DebugVisualization::~DebugVisualization() {
    // Clean up all active debug meshes
    for (int meshId : m_activeMeshIds) {
        m_meshHandler->removeMesh(meshId);
    }
}

void DebugVisualization::loadSphereMeshData() {
    if (m_sphereMeshLoaded) return;
    
    try {
        // Load the sphere mesh using AssimpLoader
        AssimpLoader::load("../media/blender/02_sphere.obj", &s_sphereMeshData);
        m_sphereMeshLoaded = true;
        std::cout << "Debug sphere mesh loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load debug sphere mesh: " << e.what() << std::endl;
        // Create a simple fallback sphere data if needed
        m_sphereMeshLoaded = false;
    }
}

void DebugVisualization::loadRedTexture() {
    if (m_redTextureLoaded) return;
    
    try {
        // Load red texture - you'll need to create a simple red texture file
        MeshHandler::Texture redTexture = m_meshHandler->createTexture("../media/debug_red.png");
        m_redTextureUnit = redTexture.m_textureUnit;
        m_redTextureLoaded = true;
        std::cout << "Debug red texture loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load debug red texture: " << e.what() << std::endl;
        m_redTextureUnit = -1; // Use no texture
        m_redTextureLoaded = false;
    }
}

int DebugVisualization::createSphere(const glm::dvec3& position, double radius) {
    if (!m_sphereMeshLoaded || s_sphereMeshData.empty()) {
        std::cerr << "Cannot create debug sphere: mesh data not loaded" << std::endl;
        return -1;
    }
    
    // Create a new mesh
    int meshId = m_meshHandler->addMesh();
    if (meshId < 0) {
        std::cerr << "Failed to create mesh for debug sphere" << std::endl;
        return -1;
    }
    
    // Load sphere mesh data into the mesh
    const AssetMeshData& sphereMesh = s_sphereMeshData[0];
    
    std::vector<glm::dvec3> positions;
    std::vector<glm::dvec3> normals;
    std::vector<glm::dvec3> tangents;
    std::vector<glm::dvec2> uvs;
    
    // Scale and transform sphere vertices according to radius
    for (size_t i = 0; i < sphereMesh.indices.size(); i++) {
        int idx = sphereMesh.indices[i];
        
        // Position - scale by radius
        const auto& pos = sphereMesh.positionsData[idx];
        glm::dvec3 scaledPos = glm::dvec3(pos[0], pos[1], pos[2]) * radius;
        positions.push_back(scaledPos);
        
        // Normal - don't scale
        const auto& norm = sphereMesh.normalsData[idx];
        normals.push_back(glm::dvec3(norm[0], norm[1], norm[2]));
        
        // Tangent
        const auto& tang = sphereMesh.tangentsData[idx];
        tangents.push_back(glm::dvec3(tang[0], tang[1], tang[2]));
        
        // UV coordinates
        const auto& texUV = sphereMesh.uvsData[idx];
        uvs.push_back(glm::dvec2(texUV[0], texUV[1]));
    }
    
    // Add triangles to mesh
    std::vector<uint32_t> triangleIds = m_meshHandler->appendTrianglesToMesh(
        meshId, &positions, &normals, &tangents, &uvs);
    
    if (triangleIds.empty()) {
        m_meshHandler->removeMesh(meshId);
        std::cerr << "Failed to add triangles to debug sphere mesh" << std::endl;
        return -1;
    }
    
    // Set mesh transform
    glm::dvec3 velocity(0.0);
    glm::dquat orientation(1.0, 0.0, 0.0, 0.0);
    glm::dvec3 angVelAxis(0.0, 1.0, 0.0);
    double angVel = 0.0;
    glm::dvec3 centerOfRotation(0.0);
    
    m_meshHandler->updateMeshData(
        meshId,
        &position,
        &velocity,
        orientation,
        angVelAxis,
        angVel,
        centerOfRotation,
        m_redTextureUnit,  // Red texture
        -1,                // No normal texture
        0                  // Time
    );
    
    // Add to active meshes list
    m_activeMeshIds.push_back(meshId);
    
    std::cout << "Created debug sphere at (" << position.x << ", " << position.y << ", " << position.z 
              << ") with radius " << radius << std::endl;
    
    return meshId;
}

void DebugVisualization::deleteMesh(int meshId) {
    // Remove from mesh handler
    m_meshHandler->removeMesh(meshId);
    
    // Remove from active meshes list
    auto it = std::find(m_activeMeshIds.begin(), m_activeMeshIds.end(), meshId);
    if (it != m_activeMeshIds.end()) {
        m_activeMeshIds.erase(it);
    }
}

void DebugVisualization::update() {
    // Currently no per-frame updates needed for debug shapes
    // This function is here for future extensions
}