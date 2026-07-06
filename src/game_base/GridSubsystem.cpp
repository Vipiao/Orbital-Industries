// GridSubsystem.cpp
#include "GridSubsystem.h"
#include "BlockResourceCache.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/SensorCollider.h"
#include "../physics/GridCollider.h"
#include "graphics/GraphicsEngine.h"
#include "utils/JobManager.h"
#include "utils/TimeHandler.h"
#include "utils/HashFunctions.h"
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

    // Preload shared special-block models once, up front, so placing a block
    // never triggers a synchronous OBJ/texture load (no frame spike).
    m_blockResources = std::make_unique<BlockResourceCache>(m_graphics);

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
    // Clear collider mapping before destroying grids to avoid dangling keys
    m_colliderToGrid.clear();

    // Clear all grids (will trigger Grid destructors)
    m_grids.clear();
}

std::weak_ptr<Grid> GridSubsystem::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    auto grid = std::make_shared<Grid>(
        m_physics,
        m_graphics,
        m_jobManager,
        m_timeHandler,
        m_blockResources.get(),
        position,
        orientation
    );
    
    m_grids.push_back(std::move(grid));
    std::weak_ptr<Grid> gridPtr = m_grids.back();
    
    // Register collider->grid mapping for O(1) sensor queries
    if (auto gridLocked = gridPtr.lock()) {
        if (auto collider = gridLocked->getCollider().lock()) {
            m_colliderToGrid[collider.get()] = gridPtr;
        } else {
            // Should never happen - Grid just created should have valid collider
            throw std::runtime_error("GridSubsystem::createGrid: Grid has no collider");
        }
    }
    
    return gridPtr;
}

void GridSubsystem::removeGrid(std::weak_ptr<Grid> gridWeak) {
    auto grid = gridWeak.lock();
    if (!grid) return; // Grid already destroyed

    // CRITICAL: Remove from collider mapping BEFORE erasing from m_grids
    // Order matters because Grid destructor destroys the collider
    if (auto collider = grid->getCollider().lock()) {
        m_colliderToGrid.erase(collider.get());
    } else {
        // Collider already destroyed - this shouldn't happen in normal operation
        // but we can continue safely since the map entry is already gone or irrelevant
    }
    
    // Now safe to remove grid (and destroy its collider)
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
 
std::vector<std::weak_ptr<Grid>> GridSubsystem::getGridsFromOverlaps(const SensorCollider* sensor) const {
    std::vector<std::weak_ptr<Grid>> grids;
    
    if (!sensor) {
        return grids;
    }
    
    for (Collider* collider : sensor->getOverlappingColliders()) {
        // O(1) lookup in hash map
        auto it = m_colliderToGrid.find(collider);
        if (it != m_colliderToGrid.end()) {
            // Found a grid for this collider
            grids.push_back(it->second);
        }
    }

    return grids;
}

std::weak_ptr<Grid> GridSubsystem::getGridFromCollider(Collider* collider) const {
    auto it = m_colliderToGrid.find(collider);
    if (it == m_colliderToGrid.end()) {
        return {};
    }
    return it->second;
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