// GridSubsystem.h
#pragma once

#include "Grid.h"
#include "GridSplitter.h"
#include "../utils/IHashable.h"
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
    
    // Grid lifecycle
    std::weak_ptr<Grid> createGrid(
        const glm::dvec3& position,
        const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0)
    );
    
    void removeGrid(std::weak_ptr<Grid> grid);
    
    // Grid updates (called by GameBase each frame)
    void updateAllGraphics(const glm::dvec3& cameraPos);
    bool handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    
    // Grid operations (used by tools)
    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    // Access
    const std::vector<std::shared_ptr<Grid>>& getGrids() const { return m_grids; }
    
    /**
     * @brief Convert sensor collider overlaps to Grid pointers
     * @param sensor SensorCollider to query for overlapping grids
     * @return Vector of Grid weak_ptrs that overlap with the sensor
     */
    std::vector<std::weak_ptr<Grid>> getGridsFromOverlaps(const SensorCollider* sensor) const;

    // IHashable interface
    virtual size_t computeHash() const override;
    
private:
    // Dependencies
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;
    
    // Grid ownership
    std::vector<std::shared_ptr<Grid>> m_grids;

    // Fast lookup: Collider -> Grid mapping for sensor queries
    std::unordered_map<Collider*, std::weak_ptr<Grid>> m_colliderToGrid;
    
    // Grid splitting logic
    std::unique_ptr<GridSplitter> m_gridSplitter;
    
    // Helper for GridSplitter callback
    Grid* findGridById(uint64_t gridId);
};