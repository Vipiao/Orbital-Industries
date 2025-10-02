// GridSplitter.h
#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <chrono>
#include <glm/glm.hpp>
#include "../utils/Generator.h"
#include "../utils/HashFunctions.h"

// Forward declarations
class Grid;
class TimeHandler;

/**
 * @brief Handles grid splitting logic when blocks are removed
 * 
 * Manages deferred grid partition analysis and creation of new grids
 * when a grid becomes disconnected into multiple pieces.
 */
class GridSplitter {
public:
    /**
     * @brief Constructor
     * @param createGridCallback Callback to create new grid: (position, orientation) -> weak_ptr<Grid>
     * @param removeGridCallback Callback to remove grid: (grid) -> void
     * @param findGridByIdCallback Callback to find grid by unique ID: (gridId) -> Grid*
     * @param timeHandler Pointer to time handler for timing checks
     */
    GridSplitter(
        std::function<std::weak_ptr<Grid>(const glm::dvec3&, const glm::dquat&)> createGridCallback,
        std::function<void(std::weak_ptr<Grid>)> removeGridCallback,
        std::function<Grid*(uint64_t)> findGridByIdCallback,
        TimeHandler* timeHandler
    );
    
    ~GridSplitter();
    
    /**
     * @brief Schedule a grid split check for later processing
     * @param sourceGrid Grid that may need splitting
     * @param edgeCoords Coordinates near removed blocks to check for disconnection
     */
    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
    
    /**
     * @brief Process pending grid splits with time budget
     * @param endTime Time point when processing should stop
     * @return true if more work remains, false if all splits processed
     */
    bool handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    
private:
    // Callbacks for grid operations
    std::function<std::weak_ptr<Grid>(const glm::dvec3&, const glm::dquat&)> m_createGridCallback;
    std::function<void(std::weak_ptr<Grid>)> m_removeGridCallback;
    std::function<Grid*(uint64_t)> m_findGridByIdCallback;
    
    TimeHandler* m_timeHandler;
    
    // Pending split operations (gridId -> edge coordinates)
    std::unordered_map<uint64_t, std::unordered_set<glm::ivec3, Hash::IVec3Hash>> m_pendingGridSplits;
    
    // Async generator state
    std::unique_ptr<Generator<bool>> m_pendingSplitsGenerator;
    
    // Async processing methods
    Generator<bool> handlePendingSplitsAsync();
    Generator<bool> performGridSplitAsync(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords);
};