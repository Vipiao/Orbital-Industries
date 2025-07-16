// GridGraphics.h
#pragma once

#include "../graphics/GraphicsEngine.h"
#include "CellType.h"
#include "../utils/HashFunctions.h"
#include "../graphics/AssimpLoader.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <unordered_map>
#include <vector>
#include <memory>
#include "../utils/JobManager.h"

/**
 * @brief Graphics data for grid cells (not a base class)
 */
class GraphicsCell {
public:
    std::vector<uint32_t> faceTriangleIds[6]; // Triangle IDs for each face direction
    
    GraphicsCell() = default;
    ~GraphicsCell() = default;
};

/**
 * @brief Handles all graphics/rendering logic for grid systems
 */
class GridGraphics {
public:
    /**
     * @brief Constructor
     * @param graphics Pointer to the graphics engine
     */
    explicit GridGraphics(GraphicsEngine* graphics, JobManager* jobManager);
    
    /**
     * @brief Destructor - cleans up mesh and resources
     */
    ~GridGraphics();
    
    // Cell management
    void addCell(const glm::ivec3& coord, CellType type);
    void removeCell(const glm::ivec3& coord);
    
    // Graphics updates
    void updateGraphics(
        const glm::dvec3& cameraPos,
        const glm::dvec3& gridPosition,
        const glm::dquat& gridOrientation,
        const glm::dvec3& gridVelocity,
        const glm::dvec3& gridAngularVelocity,
        const glm::dvec3& gridCenter,
        uint64_t currentTimeStep,
        double approximateRadius);
    
    // Cell access for other systems
    bool hasGraphicsCell(const glm::ivec3& coord) const;
    GraphicsCell* getGraphicsCell(const glm::ivec3& coord);
    
private:
    // Face transform data structure
    struct FaceTransform {
        glm::dvec3 axis;
        double angle;
    };
    
    // Graphics cell storage
    std::unordered_map<glm::ivec3, GraphicsCell, IVec3Hash> m_graphicsCells;
    
    // Graphics engine reference
    GraphicsEngine* m_graphics;
    int m_meshId;
    
    // Texture management
    static int s_colorTextureUnit;
    static int s_normalTextureUnit;
    static bool s_texturesLoaded;
    
    // Face mesh data
    static std::vector<AssetMeshData> s_faceMeshData;
    static bool s_faceMeshDataLoaded;
    static const FaceTransform s_faceTransforms[6];

    // Job system
    JobManager* m_jobManager;

    // Track pending jobs for cleanup
    std::vector<std::weak_ptr<Job>> m_pendingJobs;
    
    // GPU state tracking for optimization
    mutable glm::dvec3 m_lastSentPosition{0.0};
    mutable glm::dquat m_lastSentOrientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_lastSentVelocity{0.0};
    glm::dquat m_lastSentAngularVelocityQuat{1.0, 0.0, 0.0, 0.0};
    uint64_t m_nextUpdateTimeStep{0};
    mutable uint64_t m_lastCheckedTimeStep{0};
    
    // Update thresholds
    static constexpr double POSITION_THRESHOLD = 0.0008;
    static constexpr double ORIENTATION_THRESHOLD_BASE = 0.0004;
    static constexpr uint64_t TIME_THRESHOLD = 256;
    
    // Private methods
    void loadTextures();
    static void loadFaceMeshData();
    static glm::dmat4 getFaceTransform(int faceIndex, const glm::ivec3& coord);
    
    // Job-based graphics operations
    void removeCellGraphics(const std::vector<std::vector<uint32_t>>& faceTriangleIds);
    void updateCellGraphics(const glm::ivec3& coord);
    void scheduleNeighborUpdateJobs(const glm::ivec3& coord);

    // Helper to track job handles
    void trackJob(std::weak_ptr<Job> jobHandle);
    
    bool shouldUpdateGPU(
        const glm::dvec3& cameraPos,
        const glm::dvec3& gridPosition,
        const glm::dquat& gridOrientation,
        uint64_t currentTimeStep,
        double approximateRadius) const;
};