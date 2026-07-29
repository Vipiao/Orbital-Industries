// GridSplitter.h
#pragma once

#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <functional>
#include <chrono>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "GridSplitPiece.h"
#include "utils/Generator.h"
#include "utils/HashFunctions.h"

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
     * @param allocateGridIdCallback Allocate a fresh grid id for a split piece
     * @param createGridCallback Create a grid with a given id: (id, position, orientation)
     * @param timeHandler Pointer to time handler for timing checks
     */
    GridSplitter(
        std::function<uint64_t()> allocateGridIdCallback,
        std::function<std::weak_ptr<Grid>(uint64_t, const glm::dvec3&, const glm::dquat&)>
            createGridCallback,
        TimeHandler* timeHandler
    );

    // Realise a split on sourceGrid: create each piece with its id, move its cells over
    // (preserving type, vertices, colour, thruster throttle), and give it the source's
    // motion at the piece's centre of mass. Momentum-preserving; the largest partition
    // is not a piece and stays as the source. Used by local detection and decoded
    // commands alike.
    void applySplit(const std::shared_ptr<Grid>& sourceGrid,
                    const std::vector<GridSplitPiece>& pieces);

    ~GridSplitter();

    /**
     * @brief Schedule a grid split check for later processing
     * @param sourceGrid Grid that may need splitting
     * @param edgeCoords Coordinates near removed blocks to check for disconnection
     */
    void scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid,
                                const std::vector<glm::ivec3>& edgeCoords);

    /**
     * @brief Process pending grid splits with time budget
     * @param endTime Time point when processing should stop
     * @return true if more work remains, false if all splits processed
     */
    bool handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Splits realised since the last drain, in the order they completed.
    std::vector<GridSplitResult> drainCompletedSplits();

private:
    // A scheduled split check: the grid to analyze and the edge coordinates
    // near removed blocks
    struct PendingSplit {
        std::weak_ptr<Grid> grid;
        std::unordered_set<glm::ivec3, Hash::IVec3Hash> edgeCoords;
    };

    // Callbacks for grid operations
    std::function<uint64_t()> m_allocateGridIdCallback;
    std::function<std::weak_ptr<Grid>(uint64_t, const glm::dvec3&, const glm::dquat&)>
        m_createGridCallback;

    TimeHandler* m_timeHandler;

    // Newly scheduled splits, keyed by grid unique id so edge coords deduplicate per grid
    std::unordered_map<uint64_t, PendingSplit> m_pendingGridSplits;

    // Realised splits awaiting drain by the caller.
    std::vector<GridSplitResult> m_completedSplits;

    // Current drain, processed one grid at a time from the back
    std::vector<PendingSplit> m_splitQueue;
    std::unique_ptr<Generator<bool>> m_activeSplitGenerator;

    // Budget for the current work slice; the split coroutine checks it locally
    // and yields only when the budget is spent
    std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime{};

    Generator<bool> performGridSplitAsync(std::weak_ptr<Grid> sourceGridWeak,
                                          std::vector<glm::ivec3> edgeCoords);
};
