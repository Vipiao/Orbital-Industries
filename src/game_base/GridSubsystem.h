// GridSubsystem.h
#pragma once

#include "Grid.h"
#include "GridSplitPiece.h"
#include "GridSplitter.h"
#include "utils/IHashable.h"
#include <array>
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <chrono>
#include <unordered_map>

// Forward declarations
class PhysicsEngine;
class GraphicsEngine;
class JobManager;
class TimeHandler;
class SensorCollider;
class Collider;
class RigidBody;
class BlockResourceCache;

/**
 * @brief Subsystem managing all grid lifecycle, updates, and splitting logic
 * 
 * Owns all Grid instances and coordinates their operations with physics/graphics.
 * Provides clean interface for grid operations to other systems.
 */
class GridSubsystem : public IHashable {
public:
    /**
     * @brief Constructor
     * @param physics Physics engine for grid rigid bodies
     * @param graphics Graphics engine for grid rendering
     * @param jobManager Job manager for background tasks
     * @param timeHandler Time handler for timing operations
     */
    GridSubsystem(
        PhysicsEngine* physics,
        GraphicsEngine* graphics,
        JobManager* jobManager,
        TimeHandler* timeHandler
    );
    
    ~GridSubsystem();
    
    // No-id overload allocates locally; explicit-id constructs with a caller-supplied
    // id and advances the counter past it.
    std::weak_ptr<Grid> createGrid(
        const glm::dvec3& position,
        const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0)
    );
    std::weak_ptr<Grid> createGrid(
        uint64_t uniqueId,
        const glm::dvec3& position,
        const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0)
    );

    uint64_t allocateGridId() { return m_nextGridId++; }

    void removeGrid(std::weak_ptr<Grid> grid);

    // Remove a cell and return the coords actually removed. Does not look for splits
    // or despawn an emptied grid; call scheduleSplitCheck / despawnGrid for those.
    std::vector<glm::ivec3> removeCell(uint64_t gridId, const glm::ivec3& coord);
    // Remove a whole grid by id.
    void despawnGrid(uint64_t gridId);
    // Nudge one corner of a cell by a direction if the grid permits; returns whether
    // it did. The nudge composes on the grid's current shape, so repeated nudges add
    // up. Does not look for splits; call scheduleSplitCheck for that.
    bool modifyCell(uint64_t gridId, const glm::ivec3& coord, int cornerIndex,
                    const glm::ivec3& direction);
    // Schedule a split check seeded from the given coords and their neighbours.
    void scheduleSplitCheck(uint64_t gridId, const std::vector<glm::ivec3>& seedCoords);

    // Grid updates (called by GameBase each frame)
    void stepUpdateGraphicsAll(const glm::dvec3& cameraPos);
    bool handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    
    // Grid operations (used by tools)
    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);

    // Resolve the source grid and realise the split on it (see GridSplitter).
    void applySplit(uint64_t sourceGridId, const std::vector<GridSplitPiece>& pieces);

    // Splits realised locally since the last drain (i.e. those this peer originated).
    std::vector<GridSplitResult> drainCompletedSplits() {
        return m_gridSplitter->drainCompletedSplits();
    }
    
    // Access
    const std::vector<std::shared_ptr<Grid>>& getGrids() const { return m_grids; }

    /**
     * @brief Resolve a grid by its uniqueId (O(1) hash lookup)
     * @return The grid, or empty if no grid has that id
     */
    std::weak_ptr<Grid> getGridById(uint64_t id) const;
    
    /**
     * @brief Convert sensor collider overlaps to Grid pointers
     * @param sensor SensorCollider to query for overlapping grids
     * @return Vector of Grid weak_ptrs that overlap with the sensor
     */
    std::vector<std::weak_ptr<Grid>> getGridsFromOverlaps(const SensorCollider* sensor) const;

    /**
     * @brief Resolve a collider to its owning grid (O(1) hash lookup)
     * @param collider Collider to resolve
     * @return The grid the collider belongs to, or empty if it is not a grid collider
     */
    std::weak_ptr<Grid> getGridFromCollider(Collider* collider) const;

    /**
     * @brief Resolve a rigid body to its owning grid, via its primary collider
     *        (O(1) hash lookup)
     * @param body Body to resolve
     * @return The grid the body belongs to, or empty if it isn't a grid's body
     */
    std::weak_ptr<Grid> getGridFromBody(const std::weak_ptr<RigidBody>& body) const;

    // IHashable interface
    virtual size_t computeHash() const override;
    
private:
    // Dependencies
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Shared, preloaded geometry/textures for special blocks (thruster, cockpit).
    // Declared before m_grids so it outlives every Grid that references it.
    std::unique_ptr<BlockResourceCache> m_blockResources;

    // Grid ownership
    std::vector<std::shared_ptr<Grid>> m_grids;

    // Fast lookup: Collider -> Grid mapping for sensor queries
    std::unordered_map<Collider*, std::weak_ptr<Grid>> m_colliderToGrid;

    // Fast lookup: uniqueId -> Grid, maintained by createGrid/removeGrid
    std::unordered_map<uint64_t, std::weak_ptr<Grid>> m_idToGrid;

    uint64_t m_nextGridId{0};

    // Grid splitting logic
    std::unique_ptr<GridSplitter> m_gridSplitter;
};