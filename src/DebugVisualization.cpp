// DebugVisualization.cpp
#include "DebugVisualization.h"
#include <iostream>
#include <sstream>
#include <iomanip>
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
    for (const auto& pair : m_idToName) {
        int meshId = pair.first;
        m_meshHandler->removeMesh(meshId);
    }
    m_nameToId.clear();
    m_idToName.clear();
    m_meshProperties.clear();
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
        glm::dvec3 scaledPos = glm::dvec3(pos[0], pos[1], pos[2]);
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
    glm::dvec3 scale(radius, radius, radius);  // Scale the sphere by radius
    
    m_meshHandler->updateMeshData(
        meshId,
        &position,
        &velocity,
        orientation,
        angVelAxis,
        angVel,
        centerOfRotation,
        scale,
        m_redTextureUnit,  // Red texture
        -1,                // No normal texture
        0                  // Time
    );
    
    //std::cout << "Created debug sphere at (" << position.x << ", " << position.y << ", " << position.z 
    //          << ") with radius " << radius << std::endl;
    
    return meshId;
}

int DebugVisualization::createSphere(const std::string& name, const glm::dvec3& position, double radius) {
    // Remove existing mesh with this name if it exists
    auto it = m_nameToId.find(name);
    if (it != m_nameToId.end()) {
        removeMesh(it->second);
    }
    
    // Create the sphere
    int meshId = createSphere(position, radius);
    
    if (meshId >= 0) {
        // Store name mapping
        m_nameToId[name] = meshId;
        m_idToName[meshId] = name;
        
        // Store initial properties
        m_meshProperties[meshId] = {position, glm::dquat(1.0, 0.0, 0.0, 0.0), glm::dvec3(radius)};
    }
    
    return meshId;
}

void DebugVisualization::removeMesh(const std::string& name) {
    auto it = m_nameToId.find(name);
    if (it != m_nameToId.end()) {
        removeMesh(it->second);
    }
}

void DebugVisualization::removeMesh(int id) {
    // Remove from mesh handler
    m_meshHandler->removeMesh(id);
    
    // Remove from name mappings
    auto idToNameIt = m_idToName.find(id);
    if (idToNameIt != m_idToName.end()) {
        std::string name = idToNameIt->second;
        m_nameToId.erase(name);
        m_idToName.erase(id);
    }
    
    // Remove properties
    m_meshProperties.erase(id);
}

void DebugVisualization::setPosition(const std::string& name, const glm::dvec3& position) {
    auto it = m_nameToId.find(name);
    if (it != m_nameToId.end()) {
        setPosition(it->second, position);
    }
}

void DebugVisualization::setOrientation(const std::string& name, const glm::dquat& orientation) {
    auto it = m_nameToId.find(name);
    if (it != m_nameToId.end()) {
        setOrientation(it->second, orientation);
    }
}

void DebugVisualization::setScale(const std::string& name, const glm::dvec3& scale) {
    auto it = m_nameToId.find(name);
    if (it != m_nameToId.end()) {
        setScale(it->second, scale);
    }
}

void DebugVisualization::setPosition(int id, const glm::dvec3& position) {
    auto it = m_meshProperties.find(id);
    if (it != m_meshProperties.end()) {
        it->second.position = position;
        updateMeshTransform(id);
    }
}

void DebugVisualization::setOrientation(int id, const glm::dquat& orientation) {
    auto it = m_meshProperties.find(id);
    if (it != m_meshProperties.end()) {
        it->second.orientation = orientation;
        updateMeshTransform(id);
    }
}

void DebugVisualization::setScale(int id, const glm::dvec3& scale) {
    auto it = m_meshProperties.find(id);
    if (it != m_meshProperties.end()) {
        it->second.scale = scale;
        updateMeshTransform(id);
    }
}

int DebugVisualization::getIdFromName(const std::string& name) const {
    auto it = m_nameToId.find(name);
    return (it != m_nameToId.end()) ? it->second : -1;
}

std::string DebugVisualization::getNameFromId(int id) const {
    auto it = m_idToName.find(id);
    return (it != m_idToName.end()) ? it->second : "";
}

void DebugVisualization::updateMeshTransform(int id) {
    auto it = m_meshProperties.find(id);
    if (it != m_meshProperties.end()) {
        const auto& props = it->second;
        glm::dvec3 velocity(0.0);
        glm::dvec3 angVelAxis(0.0, 1.0, 0.0);
        double angVel = 0.0;
        glm::dvec3 centerOfRotation(0.0);
        
        m_meshHandler->updateMeshData(
            id,
            &props.position,
            &velocity,
            props.orientation,
            angVelAxis,
            angVel,
            centerOfRotation,
            props.scale,
            m_redTextureUnit,
            -1,
            0
        );
    }
}

void DebugVisualization::update() {
    // Currently no per-frame updates needed for debug shapes
    // This function is here for future extensions
}

std::string DebugVisualization::generateGeogebraCommands(const std::vector<glm::dvec2>& points, int precision) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    
    for (size_t i = 0; i < points.size(); ++i) {
        // Generate point name (A, B, C, ..., Z, A1, B1, C1, ...)
        std::string pointName;
        if (i < 26) {
            pointName = static_cast<char>('A' + i);
        } else {
            size_t seriesIndex = (i - 26) / 26 + 1;
            size_t letterIndex = (i - 26) % 26;
            pointName = static_cast<char>('A' + letterIndex) + std::to_string(seriesIndex);
        }
        
        oss << pointName << "=(" << points[i].x << "," << points[i].y << ")";
        if (i < points.size() - 1) {
            oss << "\n";
        }
    }
    return oss.str();
}

std::string DebugVisualization::generateGeogebraCommands(const std::vector<glm::dvec3>& points, int precision) const {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);
    
    for (size_t i = 0; i < points.size(); ++i) {
        // Generate point name (A, B, C, ..., Z, A1, B1, C1, ...)
        std::string pointName;
        if (i < 26) {
            pointName = static_cast<char>('A' + i);
        } else {
            size_t seriesIndex = (i - 26) / 26 + 1;
            size_t letterIndex = (i - 26) % 26;
            pointName = static_cast<char>('A' + letterIndex) + std::to_string(seriesIndex);
        }
        
        oss << pointName << "=(" << points[i].x << "," << points[i].y << "," << points[i].z << ")";
        if (i < points.size() - 1) {
            oss << "\n";
        }
    }
    return oss.str();
}