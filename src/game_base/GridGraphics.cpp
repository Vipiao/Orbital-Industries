// GridGraphics.cpp
#include "GridGraphics.h"
#include "BlockResourceCache.h"
#include "thruster/PlumeGraphics.h"
#include "graphics/SSBOManager.h"
#include <iostream>
#include <set>
#include <algorithm>
#include "../game_base/JobPriorities.h"

// Static member initialization
int GridGraphics::s_colorTextureUnit = -1;
int GridGraphics::s_normalTextureUnit = -1;
int GridGraphics::s_maskTextureUnit = -1;
bool GridGraphics::s_texturesLoaded = false;

GridGraphics::GridGraphics(GraphicsEngine* graphics, JobManager* jobManager,
                           BlockResourceCache* blockResources)
    : m_graphics(graphics), m_blockResources(blockResources), m_jobManager(jobManager) {

    if (!graphics) {
        throw std::runtime_error("GraphicsEngine pointer cannot be null");
    }

    if (!jobManager) {
        throw std::runtime_error("JobManager pointer cannot be null");
    }

    if (!blockResources) {
        throw std::runtime_error("BlockResourceCache pointer cannot be null");
    }

    // Allocate SSBO index here — GridGraphics owns this slot and shares it with both
    // the mesh system (for block triangles) and the instance system (for thruster models).
    m_ssboIndex = m_graphics->m_ssboManager->allocateIndex();
    m_graphics->createMesh(m_ssboIndex);

    // Load textures
    loadTextures();
    
    // Initialize next update time to 0 to force initial update
    m_nextUpdateTimeStep = 0;
    m_lastCheckedTimeStep = 0;
}

GridGraphics::~GridGraphics() {
    // Cancel all pending jobs
    for (auto& jobHandle : m_pendingJobs) {
        if (!jobHandle.expired()) {
            m_jobManager->cancel(jobHandle);
        }
    }

    // Remove mesh triangles, then release the SSBO slot we own
    if (m_ssboIndex >= 0) {
        m_graphics->removeMesh(m_ssboIndex);
        m_graphics->m_ssboManager->deallocateIndex(m_ssboIndex);
    }
}

void GridGraphics::loadTextures() {
    if (s_texturesLoaded) return;
    
    try {
        MeshHandler::Texture colorTexture = m_graphics->createTexture("../media/color_512x512_occluded.png");
        s_colorTextureUnit = colorTexture.m_textureUnit;

        MeshHandler::Texture normalTexture = m_graphics->createTexture("../media/normal_combined_512x512.png");
        s_normalTextureUnit = normalTexture.m_textureUnit;

        MeshHandler::Texture maskTexture = m_graphics->createTexture("../media/albedo_mask.png");
        s_maskTextureUnit = maskTexture.m_textureUnit;

        s_texturesLoaded = true;
        std::cout << "GridGraphics: Textures loaded successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "GridGraphics: Failed to load textures: " << e.what() << std::endl;
    }
}

void GridGraphics::addCell(const glm::ivec3& coord, CellType type, const PolyhedronProcessor::MeshData& meshData, const glm::dvec4& color) {
    // Add graphics cell to map
    m_graphicsCells.emplace(coord, GraphicsCell{});
    
    // Schedule update job for this cell
    auto jobHandle = m_jobManager->schedule([this, coord, meshData, color](std::chrono::time_point<std::chrono::high_resolution_clock> /*endTime*/) -> bool {
        updateCellGraphics(coord, meshData, color);
        return false; // Job complete
    }, JobPriorities::GRAPHICS_UPDATE);
    trackJob(jobHandle);
}

void GridGraphics::removeCell(const glm::ivec3& coord) {
    auto cellIt = m_graphicsCells.find(coord);
    if (cellIt == m_graphicsCells.end()) {
        return; // Cell doesn't exist
    }
    
    // Capture the triangle IDs before removing the cell
    std::vector<uint32_t> triangleIds = cellIt->second.triangleIds;

    // Schedule remove job with captured triangle IDs
    auto jobHandle = m_jobManager->schedule([this, triangleIds](std::chrono::time_point<std::chrono::high_resolution_clock> /*endTime*/) -> bool {
        removeCellGraphics(triangleIds);
        return false; // Job complete
    }, JobPriorities::GRAPHICS_REMOVE);
    trackJob(jobHandle);

    // Remove graphics cell from map immediately
    m_graphicsCells.erase(coord);
}

void GridGraphics::updateCell(const glm::ivec3& coord, const PolyhedronProcessor::MeshData& meshData, const glm::dvec4& color) {
    // Check if cell exists
    if (m_graphicsCells.find(coord) == m_graphicsCells.end()) {
        return; // Cell doesn't exist
    }
    
    // Schedule update job for this cell
    auto jobHandle = m_jobManager->schedule([this, coord, meshData, color](std::chrono::time_point<std::chrono::high_resolution_clock> /*endTime*/) -> bool {
        updateCellGraphics(coord, meshData, color);
        return false; // Job complete
    }, JobPriorities::GRAPHICS_UPDATE);
    trackJob(jobHandle);
}

bool GridGraphics::hasGraphicsCell(const glm::ivec3& coord) const {
    return m_graphicsCells.find(coord) != m_graphicsCells.end();
}

GraphicsCell* GridGraphics::getGraphicsCell(const glm::ivec3& coord) {
    auto it = m_graphicsCells.find(coord);
    return (it != m_graphicsCells.end()) ? &it->second : nullptr;
}

void GridGraphics::removeCellGraphics(const std::vector<uint32_t>& triangleIds) {
    // Remove all triangles from the mesh
    if (!triangleIds.empty()) {
        m_graphics->removeTrianglesFromMesh(m_ssboIndex, &triangleIds);
    }
}

void GridGraphics::updateCellGraphics(const glm::ivec3& coord, const PolyhedronProcessor::MeshData& meshData, const glm::dvec4& color) {
    auto it = m_graphicsCells.find(coord);
    if (it == m_graphicsCells.end()) return; // Cell doesn't exist, job is no-op
    
    GraphicsCell& cell = it->second;
    
    // Remove existing mesh for this cell first
    if (!cell.triangleIds.empty()) {
        m_graphics->removeTrianglesFromMesh(m_ssboIndex, &cell.triangleIds);
        cell.triangleIds.clear();
    }

        
    // Make a copy of mesh data for transformation
    PolyhedronProcessor::MeshData transformedMeshData = meshData;
    
    // Apply translation to position at coord + 0.5 in grid-local coordinates
    glm::dvec3 offset = glm::dvec3(coord);
    
    // Transform positions by adding offset (normals and tangents unchanged for translation)
    for (size_t i = 0; i < transformedMeshData.positions.size(); ++i) {
        transformedMeshData.positions[i] += offset;
    }
    
    if (transformedMeshData.isEmpty()) {
        return; // No mesh data to render
    }
    
    // Add complete block mesh to graphics
    if (!transformedMeshData.positions.empty()) {
        // Create color vector with same size as positions
        std::vector<glm::dvec4> colors(transformedMeshData.positions.size(), color);

        // Materials travel with the vertices rather than with the grid's
        // transform, so one grid mesh is free to mix them per cell.
        const std::vector<uint32_t> textureUnits(
            transformedMeshData.positions.size(),
            MeshHandler::packTextureUnits(s_colorTextureUnit, s_normalTextureUnit, -1,
                                          s_maskTextureUnit));

        cell.triangleIds = m_graphics->appendTrianglesToMesh(
            m_ssboIndex, &transformedMeshData.positions, &transformedMeshData.normals,
            &transformedMeshData.tangents, &transformedMeshData.uvs,
            nullptr, &colors, &textureUnits);
    }
}

void GridGraphics::addBlockInstance(CellType type,
                                    const glm::ivec3& anchorCoord,
                                    const glm::dquat& orientation,
                                    const glm::dvec3& modelCentre)
{
    // Resources are preloaded and shared via the cache — no lazy load here.
    BlockResources* resources = m_blockResources->get(type);
    if (!resources) {
        throw std::runtime_error("GridGraphics::addBlockInstance: no preloaded "
                                 "resources for block type");
    }
    m_blockGraphicsMap.emplace(anchorCoord,
        std::make_unique<BlockGraphics>(
            resources, m_ssboIndex, anchorCoord, orientation, modelCentre));

    // Thrusters also get an always-on ion-plume ray-volume instance, sharing the
    // grid's SSBO slot so it follows the ship.
    if (type == CellType::THRUSTER) {
        m_plumeGraphicsMap.emplace(anchorCoord,
            std::make_unique<PlumeGraphics>(
                m_graphics, m_blockResources->getPlumeGeometry(),
                m_ssboIndex, anchorCoord, orientation, modelCentre));
    }
}

void GridGraphics::removeBlockInstance(const glm::ivec3& anchorCoord) {
    m_blockGraphicsMap.erase(anchorCoord);
    m_plumeGraphicsMap.erase(anchorCoord);
}

void GridGraphics::setPlumeThrust(const glm::ivec3& anchorCoord, double level) {
    auto it = m_plumeGraphicsMap.find(anchorCoord);
    if (it != m_plumeGraphicsMap.end()) {
        it->second->setThrustLevel(level);
    }
}

void GridGraphics::trackJob(std::weak_ptr<Job> jobHandle) {
    // Clean up expired handles periodically to prevent unbounded growth
    if (m_pendingJobs.size() % 50 == 0) {
        m_pendingJobs.erase(std::remove_if(m_pendingJobs.begin(), m_pendingJobs.end(),
            [](const std::weak_ptr<Job>& handle) { return handle.expired(); }), m_pendingJobs.end());
    }
    
    m_pendingJobs.push_back(jobHandle);
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
    
    if (m_ssboIndex < 0) {
        return;
    }

    // Only update GPU if there's a significant change
    if (!shouldUpdateGPU(cameraPos, gridPosition, gridOrientation, currentTimeStep, approximateRadius)) {
        return;
    } else {
        //std::cout << "Updated graphics for " << m_graphicsCells.size() << " cells" << std::endl;
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
        m_ssboIndex,
        gridPosition - gridCenter,
        gridVelocity,
        gridOrientation,
        angVelAxis,
        angVelMagnitude,
        gridCenter,
        glm::dvec3(1.0, 1.0, 1.0),      // Default scale
        currentTimeStep,
        0.0                             // Grid blocks non-emissive by default
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