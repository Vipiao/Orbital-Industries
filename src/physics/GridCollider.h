// GridCollider.h
#pragma once

#include "Collider.h"
#include "CubeCollider.h"
#include "../utils/HashFunctions.h"
#include <glm/glm.hpp>
#include <unordered_map>
#include <memory>
#include <queue>
#include <unordered_set>
#include <chrono>
 
class Job;
class JobManager;
class TimeHandler;

// Metadata for grid cells
class CellMetadata {
public:
    // Cell classification for optimization
    enum class CellClassification {
        INNER,   // All 6 neighbors occupied
        FACE,    // 4/6 neighbors, 2 empty in opposite directions
        EDGE,    // At least 2 neighbors with at least one opposite pair
        CORNER   // Everything else
    };
    
    CellClassification classification = CellClassification::CORNER;
};

/**
 * @brief Pre-computed neighborhood of cells for fast spatial queries
 * Stores pointers to nearby cells within a fixed radius to avoid repeated hash lookups
 */
class CellNeighborhood {
public:
    std::vector<Collider*> m_neighbors;             // All cells: center cell first (if exists), then neighbors
    bool m_hasCenter = false;                       // True if first element is the center cell
    
    CellNeighborhood() {
        m_neighbors.reserve(27); // 3x3x3 = 27 max cells (center + neighbors)
    }
};

class GridCollider : public Collider {
public:
    GridCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 ColliderReference* reference = nullptr,
                 JobManager* jobManager = nullptr,
                 TimeHandler* timeHandler = nullptr);
    
    virtual ~GridCollider();
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateSimpleAABB(uint64_t currentTimestep) override;
    virtual void updateAdvancedAABB(uint64_t currentTimestep) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Grid-specific methods
    /**
     * @brief Add a collider to the grid at the specified coordinate
     * @param coord Grid coordinate where the collider will be placed
     * @param collider Collider to add (ownership is transferred - caller's unique_ptr becomes null after this call)
     */
    void addCell(const glm::ivec3& coord, std::unique_ptr<Collider> collider);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    
    // Get sub-collider for a specific cell
    Collider* getCell(const glm::ivec3& coord);

    // Spatial search within radius  
    std::vector<Collider*> findCellsInRadius(const glm::dvec3& worldPos, double searchRadius) const;

    // Allow access to cells for collision detection utils
    const std::unordered_map<glm::ivec3, std::unique_ptr<Collider>, IVec3Hash>& getCells() const { return m_cells; }

    // Optimization: Pre-computed neighborhoods for fast spatial queries
    static constexpr int NEIGHBORHOOD_RADIUS = 1; // 3x3x3 neighborhood
    std::unordered_map<glm::ivec3, CellNeighborhood, IVec3Hash> m_neighborhoods;

    // Access to local AABB bounds for optimization
    const glm::ivec3& getLocalAABBMin() const { return m_localAABBMin; }
    const glm::ivec3& getLocalAABBMax() const { return m_localAABBMax; }

    // Allow CollisionDetectionUtils to access neighborhoods for optimization
    friend class CollisionDetectionUtils;
    
    glm::dvec3 gridToWorld(const glm::dvec3& gridCoord) const;
    glm::dvec3 worldToGrid(const glm::dvec3& worldCoord) const;

    // Type identification  
    static constexpr int TYPE_ID = hashColliderName("GridCollider");

private:
    // Job system references for classification
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;
    
    // Classification system
    std::queue<glm::ivec3> m_classificationQueue;
    std::unordered_set<glm::ivec3, IVec3Hash> m_queuedCoordinates;
    std::weak_ptr<Job> m_classificationJob;

    // Classification-based cell maps for optimized collision detection
    std::unordered_map<glm::ivec3, Collider*, IVec3Hash> m_cornerCells;
    std::unordered_map<glm::ivec3, Collider*, IVec3Hash> m_edgeCells;

    // Axis counts for efficient AABB calculation
    std::unordered_map<int, int> m_xAxisCounts;
    std::unordered_map<int, int> m_yAxisCounts;
    std::unordered_map<int, int> m_zAxisCounts;
    
    // Local integer AABB
    glm::ivec3 m_localAABBMin{0};
    glm::ivec3 m_localAABBMax{0};

    // Cached local AABB corners (only recalculated when local AABB changes)
    glm::dvec3 m_localCorners[8];
    uint64_t m_cornersValidUntilTime = 0;

    // Map of grid coordinates to cube colliders
    std::unordered_map<glm::ivec3, std::unique_ptr<Collider>, IVec3Hash> m_cells;
    
    // Helper methods
    void updateSubColliderTransformsAndAABB(uint64_t currentTimestep);

    // Filter normal management
    void updateFilterNormalsForCell(const glm::ivec3& coord);
    void updateFilterNormalsAfterRemoval(const glm::ivec3& removedCoord);

    // Helper to update cached corners when local AABB changes
    void updateLocalCorners();

    // Classification methods
    void queueCoordinateForClassification(const glm::ivec3& coord);
    void scheduleClassificationJob();
    bool processClassificationQueue(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    CellMetadata::CellClassification classifyCell(const glm::ivec3& coord);
};