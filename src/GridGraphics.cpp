// GridGraphics.cpp
#include "GridGraphics.h"
#include <iostream>
#include <set>
#include <algorithm>

// Static member initialization
int GridGraphics::s_colorTextureUnit = -1;
int GridGraphics::s_normalTextureUnit = -1;
bool GridGraphics::s_texturesLoaded = false;

std::vector<AssetMeshData> GridGraphics::s_faceMeshData;
bool GridGraphics::s_faceMeshDataLoaded = false;

// Face transform lookup table initialization
const GridGraphics::FaceTransform GridGraphics::s_faceTransforms[6] = {
    // Right (+X)
    {glm::dvec3(0, 1, 0), glm::radians(90.0)},
    // Left (-X)
    {glm::dvec3(0, 1, 0), glm::radians(-90.0)},
    // Front (+Y) 
    {glm::dvec3(1, 0, 0), glm::radians(-90.0)},
    // Back (-Y)
    {glm::dvec3(1, 0, 0), glm::radians(90.0)},
    // Up (+Z) - No rotation needed
    {glm::dvec3(1, 0, 0), 0.0},
    // Down (-Z)
    {glm::dvec3(1, 0, 0), glm::radians(180.0)}
};

GridGraphics::GridGraphics(GraphicsEngine* graphics) 
    : m_graphics(graphics), m_meshId(-1) {
    
    if (!graphics) {
        throw std::runtime_error("GraphicsEngine pointer cannot be null");
    }
    
    // Create mesh in graphics engine
    m_meshId = m_graphics->createMesh();
    if (m_meshId < 0) {
        throw std::runtime_error("Failed to create mesh for GridGraphics");
    }
    
    // Load face mesh data
    loadFaceMeshData();
    
    // Load textures
    loadTextures();
    
    // Initialize next update time to 0 to force initial update
    m_nextUpdateTimeStep = 0;
    m_lastCheckedTimeStep = 0;
}

GridGraphics::~GridGraphics() {
    // Clean up mesh
    if (m_meshId >= 0) {
        m_graphics->removeMesh(m_meshId);
    }
}

void GridGraphics::loadTextures() {
    if (s_texturesLoaded) return;
    
    try {
        MeshHandler::Texture colorTexture = m_graphics->createTexture("../media/color_512x512_occluded.png");
        s_colorTextureUnit = colorTexture.m_textureUnit;
        
        MeshHandler::Texture normalTexture = m_graphics->createTexture("../media/normal_combined_512x512.png");
        s_normalTextureUnit = normalTexture.m_textureUnit;
        
        s_texturesLoaded = true;
        std::cout << "GridGraphics: Textures loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "GridGraphics: Failed to load textures: " << e.what() << std::endl;
    }
}

void GridGraphics::loadFaceMeshData() {
    if (s_faceMeshDataLoaded) return;
    
    try {
        // Load the face mesh using AssimpLoader - a face in positive Z direction
        AssimpLoader::load("../media/blender/01_face.obj", &s_faceMeshData);
        s_faceMeshDataLoaded = true;
        std::cout << "GridGraphics: Face mesh data loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "GridGraphics: Failed to load face mesh data: " << e.what() << std::endl;
        s_faceMeshDataLoaded = false;
    }
}

void GridGraphics::addCell(const glm::ivec3& coord, CellType type) {
    // Add graphics cell to map
    m_graphicsCells.emplace(coord, GraphicsCell{});
    
    // Queue this cell for graphics update
    queueCellForUpdate(coord);
    
    // Queue neighbors for potential graphics updates
    queueNeighborsForUpdate(coord);
}

void GridGraphics::removeCell(const glm::ivec3& coord) {
    auto cellIt = m_graphicsCells.find(coord);
    if (cellIt == m_graphicsCells.end()) {
        return; // Cell doesn't exist
    }
    
    // Remove all face triangles from the mesh
    for (int face = 0; face < 6; face++) {
        if (!cellIt->second.faceTriangleIds[face].empty()) {
            m_graphics->m_meshHandler->removeTrianglesFromMesh(
                m_meshId, &cellIt->second.faceTriangleIds[face]);
        }
    }
    
    // Queue neighbors for potential graphics updates before removing the cell
    queueNeighborsForUpdate(coord);
    
    // Remove graphics cell from map
    m_graphicsCells.erase(coord);
}

void GridGraphics::queueCellForUpdate(const glm::ivec3& coord) {
    m_graphicsUpdateQueue.push(coord);
}

void GridGraphics::queueNeighborsForUpdate(const glm::ivec3& coord) {
    // Define the 6 neighbor directions (±X, ±Y, ±Z)
    static const glm::ivec3 neighbors[6] = {
        {1, 0, 0}, {-1, 0, 0}, 
        {0, 1, 0}, {0, -1, 0}, 
        {0, 0, 1}, {0, 0, -1}
    };
    
    // Add each neighbor to the update queue
    for (int i = 0; i < 6; i++) {
        glm::ivec3 neighborCoord = coord + neighbors[i];
        
        // If the neighbor exists, queue it for update
        if (hasGraphicsCell(neighborCoord)) {
            queueCellForUpdate(neighborCoord);
        }
    }
}

bool GridGraphics::hasGraphicsCell(const glm::ivec3& coord) const {
    return m_graphicsCells.find(coord) != m_graphicsCells.end();
}

GraphicsCell* GridGraphics::getGraphicsCell(const glm::ivec3& coord) {
    auto it = m_graphicsCells.find(coord);
    return (it != m_graphicsCells.end()) ? &it->second : nullptr;
}

void GridGraphics::processGraphicsQueue() {
    // Comparator for std::set to work with glm::ivec3
    struct IVec3Comparator {
        bool operator()(const glm::ivec3& a, const glm::ivec3& b) const {
            if (a.x != b.x) return a.x < b.x;
            if (a.y != b.y) return a.y < b.y;
            return a.z < b.z;
        }
    };
    
    // Process the graphics update queue
    std::set<glm::ivec3, IVec3Comparator> processed; // To avoid processing the same cell multiple times
    
    while (!m_graphicsUpdateQueue.empty()) {
        glm::ivec3 coord = m_graphicsUpdateQueue.front();
        m_graphicsUpdateQueue.pop();
        
        // Skip if already processed
        if (processed.find(coord) != processed.end()) {
            continue;
        }
        processed.insert(coord);
        
        // Only update if the cell exists
        if (hasGraphicsCell(coord)) {
            updateCellGraphics(coord);
        }
    }
}

glm::dmat4 GridGraphics::getFaceTransform(int faceIndex, const glm::ivec3& coord) {
    glm::dmat4 transform = glm::dmat4(1.0);
    
    // Position at grid coordinate
    transform = glm::translate(transform, glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5));
    
    // Apply rotation based on face lookup table
    const FaceTransform& faceTransform = s_faceTransforms[faceIndex];
    if (faceTransform.angle != 0.0) {
        transform = glm::rotate(transform, faceTransform.angle, faceTransform.axis);
    }
    
    transform = glm::scale(transform, glm::dvec3{0.5, 0.5, 0.5});
    
    return transform;
}

void GridGraphics::updateCellGraphics(const glm::ivec3& coord) {
    auto it = m_graphicsCells.find(coord);
    if (it == m_graphicsCells.end()) return;
    
    GraphicsCell& cell = it->second;
    
    // First, remove any existing face meshes for this cell
    for (int face = 0; face < 6; face++) {
        if (!cell.faceTriangleIds[face].empty()) {
            m_graphics->m_meshHandler->removeTrianglesFromMesh(m_meshId, &cell.faceTriangleIds[face]);
            cell.faceTriangleIds[face].clear();
        }
    }
    
    // Define the 6 neighbor directions (±X, ±Y, ±Z)
    static const glm::ivec3 neighbors[6] = {
        {1, 0, 0}, {-1, 0, 0},  // Right, Left (X)
        {0, 1, 0}, {0, -1, 0},  // Front, Back (Y)
        {0, 0, 1}, {0, 0, -1}   // Up, Down (Z)
    };
    
    // Then add faces that should be visible
    for (int face = 0; face < 6; face++) {
        // Check if this face is visible (neighbor is empty)
        glm::ivec3 neighborCoord = coord + neighbors[face];
        bool faceVisible = !hasGraphicsCell(neighborCoord);
        
        if (faceVisible) {
            // Get transformation for this face
            glm::dmat4 transform = getFaceTransform(face, coord);
            
            // Apply transformation to face mesh data
            std::vector<glm::dvec3> positions;
            std::vector<glm::dvec3> normals;
            std::vector<glm::dvec3> tangents;
            std::vector<glm::dvec2> uvs;
            
            // Get base face data (pointing in +Z direction)
            if (!s_faceMeshDataLoaded || s_faceMeshData.empty()) {
                continue; // Skip if mesh data not loaded
            }
            
            const AssetMeshData& faceMesh = s_faceMeshData[0];
            
            // Transform vertices according to face direction
            for (size_t i = 0; i < faceMesh.indices.size(); i++) {
                int idx = faceMesh.indices[i];
                
                // Position
                const auto& pos = faceMesh.positionsData[idx];
                glm::dvec4 transformedPos = transform * glm::dvec4(pos[0], pos[1], pos[2], 1.0);
                positions.push_back(glm::dvec3(transformedPos));
                
                // Normal 
                const auto& norm = faceMesh.normalsData[idx];
                glm::dvec4 transformedNorm = transform * glm::dvec4(norm[0], norm[1], norm[2], 0.0);
                normals.push_back(glm::normalize(glm::dvec3(transformedNorm)));
                
                // Tangent
                const auto& tang = faceMesh.tangentsData[idx];
                glm::dvec4 transformedTang = transform * glm::dvec4(tang[0], tang[1], tang[2], 0.0);
                tangents.push_back(glm::normalize(glm::dvec3(transformedTang)));
                
                // UV coordinates (don't need transformation)
                const auto& texUV = faceMesh.uvsData[idx];
                uvs.push_back(glm::dvec2(texUV[0], texUV[1]));
            }
            
            // Add this face to the mesh
            cell.faceTriangleIds[face] = m_graphics->m_meshHandler->appendTrianglesToMesh(
                m_meshId, &positions, &normals, &tangents, &uvs);
        }
    }
}

bool GridGraphics::shouldUpdateGPU(
    const glm::dvec3& cameraPos,
    const glm::dvec3& gridPosition,
    const glm::dquat& gridOrientation,
    uint64_t currentTimeStep,
    double approximateRadius) const {
    
    // Check if time step has incremented since last check
    if (currentTimeStep > m_lastCheckedTimeStep) {
        // Advance our cached state by the number of time steps that have passed
        uint64_t stepsDelta = currentTimeStep - m_lastCheckedTimeStep;
        
        // Update cached position with velocity
        m_lastSentPosition += m_lastSentVelocity * static_cast<double>(stepsDelta);
        
        // Update cached orientation with angular velocity quaternion
        for (uint64_t i = 0; i < stepsDelta; ++i) {
            m_lastSentOrientation = m_lastSentAngularVelocityQuat * m_lastSentOrientation;
        }
        m_lastSentOrientation = glm::normalize(m_lastSentOrientation);
        
        m_lastCheckedTimeStep = currentTimeStep;
    }
    
    // Check if it's time for a scheduled update
    if (currentTimeStep >= m_nextUpdateTimeStep) {
        return true;
    }

    // Calculate distance-based scaling factor
    double distanceToCamera = glm::length(cameraPos - gridPosition);
    double meshRadius = approximateRadius;
    double effectiveDistance = std::max(distanceToCamera - meshRadius, 0.1); // Prevent division by very small numbers
    double scalingFactor = std::max(effectiveDistance, 1.0); // Cap scaling factor at 1.0
    
    // Check position difference (now using interpolated cached position)
    double positionDiff = glm::length(gridPosition - m_lastSentPosition);
    double adaptivePositionThreshold = POSITION_THRESHOLD * scalingFactor;
    if (positionDiff > adaptivePositionThreshold) {
        return true;
    }
    
    // Check orientation difference using radius-based threshold (now using interpolated cached orientation)
    double orientationDot = glm::abs(glm::dot(gridOrientation, m_lastSentOrientation));
    // Clamp to handle numerical precision issues
    orientationDot = glm::clamp(orientationDot, 0.0, 1.0);
    double halfAngleDiff = glm::acos(orientationDot);
    double angleDiff = 2.0 * halfAngleDiff;
    
    // Calculate radius-based orientation threshold
    double radius = approximateRadius;
    double orientationThreshold = ORIENTATION_THRESHOLD_BASE / radius * scalingFactor;
    
    if (angleDiff > orientationThreshold) {
        return true;
    }
    
    // No significant change detected
    return false;
}

void GridGraphics::updateGraphics(
    const glm::dvec3& cameraPos,
    const glm::dvec3& gridPosition,
    const glm::dquat& gridOrientation,
    const glm::dvec3& gridVelocity,
    const glm::dvec3& gridAngularVelocity,
    const glm::dvec3& gridCenter,
    uint64_t currentTimeStep,
    double approximateRadius) {
    
    if (m_meshId < 0) {
        return;
    }

    // Only update GPU if there's a significant change
    if (!shouldUpdateGPU(cameraPos, gridPosition, gridOrientation, currentTimeStep, approximateRadius)) {
        return;
    } else {
        std::cout << "Updated graphics for " << m_graphicsCells.size() << " cells" << std::endl;
    }
    
    glm::dvec3 angVelAxis = gridAngularVelocity;
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        // If angular velocity is effectively zero, use a safe default axis
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }
    
    m_graphics->updateMeshTransform(
        m_meshId,
        gridPosition - gridCenter,
        gridVelocity,
        gridOrientation,
        angVelAxis,
        angVelMagnitude,
        gridCenter,
        glm::dvec3(1.0, 1.0, 1.0),      // Default scale
        s_colorTextureUnit,
        s_normalTextureUnit,
        currentTimeStep
    );

    // Update tracking variables with current grid state
    m_lastSentPosition = gridPosition;
    m_lastSentOrientation = gridOrientation;
    m_lastSentVelocity = gridVelocity;
    
    // Convert angular velocity to quaternion for one time step
    if (angVelMagnitude > 0.00001) {
        m_lastSentAngularVelocityQuat = glm::angleAxis(angVelMagnitude, angVelAxis);
    } else {
        m_lastSentAngularVelocityQuat = glm::dquat(1.0, 0.0, 0.0, 0.0); // Identity quaternion (no rotation)
    }
    
    // Update the last checked time step to current
    m_lastCheckedTimeStep = currentTimeStep;
    
    // Schedule next mandatory update
    m_nextUpdateTimeStep = currentTimeStep + TIME_THRESHOLD;
}