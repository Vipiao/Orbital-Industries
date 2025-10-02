// GridSubsystem.cpp
#include "GridSubsystem.h"
#include "../physics/PhysicsEngine.h"
#include "../graphics/GraphicsEngine.h"
#include "../utils/JobManager.h"
#include "../utils/TimeHandler.h"
#include "../utils/HashFunctions.h"
#include <algorithm>
#include <iostream>

GridSubsystem::GridSubsystem(
    PhysicsEngine* physics,
    GraphicsEngine* graphics,
    JobManager* jobManager,
    TimeHandler* timeHandler)
    : m_physics(physics)
    , m_graphics(graphics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
{
    if (!m_physics || !m_graphics || !m_jobManager || !m_timeHandler) {
        throw std::runtime_error("GridSubsystem: All dependencies must be non-null");
    }
    
    // Create GridSplitter with callbacks to this subsystem
    m_gridSplitter = std::make_unique<GridSplitter>(
        // Create grid callback
        [this](const glm::dvec3& pos, const glm::dquat& ori) {
            return createGrid(pos, ori);
        },
        // Remove grid callback
        [this](std::weak_ptr<Grid> grid) {
            removeGrid(grid);
        },
        // Find grid by ID callback
        [this](uint64_t gridId) {
            return findGridById(gridId);
        },
        m_timeHandler
    );
}

GridSubsystem::~GridSubsystem() {
    // Clear all grids (will trigger Grid destructors)
    m_grids.clear();
}

std::weak_ptr<Grid> GridSubsystem::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    auto grid = std::make_shared<Grid>(
        m_physics,
        m_graphics,
        m_jobManager,
        m_timeHandler,
        position,
        orientation
    );
    
    std::weak_ptr<Grid> gridPtr = grid;
    m_grids.push_back(std::move(grid));
    
    return gridPtr;
}

void GridSubsystem::removeGrid(std::weak_ptr<Grid> gridWeak) {
    auto grid = gridWeak.lock();
    if (!grid) return; // Grid already destroyed
    
    auto it = std::find_if(m_grids.begin(), m_grids.end(),
        [grid](const std::shared_ptr<Grid>& item) {
            return item.get() == grid.get();
        });
    
    if (it != m_grids.end()) {
        m_grids.erase(it);
    }
}

void GridSubsystem::updateAllGraphics(const glm::dvec3& cameraPos) {
    for (auto& grid : m_grids) {
        grid->updateGraphics(cameraPos);
    }
}

bool GridSubsystem::handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    return m_gridSplitter->handlePendingSplits(endTime);
}

void GridSubsystem::scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    m_gridSplitter->scheduleGridSplitCheck(sourceGrid, edgeCoords);
}

Grid* GridSubsystem::findGridById(uint64_t gridId) {
    for (const auto& grid : m_grids) {
        if (grid->uniqueId == gridId) {
            return grid.get();
        }
    }
    return nullptr;
}

size_t GridSubsystem::computeHash() const {
    size_t hash = 0;
    
    // Hash all grids in deterministic order (by uniqueId)
    std::vector<Grid*> sortedGrids;
    sortedGrids.reserve(m_grids.size());
    for (const auto& grid : m_grids) {
        sortedGrids.push_back(grid.get());
    }
    std::sort(sortedGrids.begin(), sortedGrids.end(),
        [](Grid* a, Grid* b) { return a->uniqueId < b->uniqueId; });
    
    for (Grid* grid : sortedGrids) {
        hash = Hash::combineHashes(hash, grid->computeHash());
    }
    
    return hash;
}