// GridSubsystem.cpp
#include "GridSubsystem.h"
#include "BlockResourceCache.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../physics/SensorCollider.h"
#include "../physics/GridCollider.h"
#include "graphics/GraphicsEngine.h"
#include "utils/JobManager.h"
#include "utils/TimeHandler.h"
#include "utils/HashFunctions.h"
#include <algorithm>
#include <cassert>
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
        [this]() { return allocateGridId(); },
        [this](uint64_t id, const glm::dvec3& pos, const glm::dquat& ori) {
            return createGrid(id, pos, ori);
        },
        m_timeHandler
    );
}

GridSubsystem::~GridSubsystem() {
    // Clear mappings before destroying grids to avoid dangling keys
    m_colliderToGrid.clear();
    m_idToGrid.clear();

    // Clear all grids (will trigger Grid destructors)
    m_grids.clear();
}

std::weak_ptr<Grid> GridSubsystem::createGrid(const glm::dvec3& position, const glm::dquat& orientation) {
    return createGrid(allocateGridId(), position, orientation);
}

std::weak_ptr<Grid> GridSubsystem::createGrid(uint64_t uniqueId, const glm::dvec3& position,
                                              const glm::dquat& orientation) {
    m_nextGridId = std::max(m_nextGridId, uniqueId + 1);

    auto grid = std::make_shared<Grid>(
        uniqueId,
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
        m_idToGrid[gridLocked->uniqueId] = gridPtr;
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
    
    // Every grid enters through createGrid, so its id must be mapped.
    [[maybe_unused]] size_t erasedIds = m_idToGrid.erase(grid->uniqueId);
    assert(erasedIds == 1);

    // Now safe to remove grid (and destroy its collider)
    auto it = std::find_if(m_grids.begin(), m_grids.end(),
        [grid](const std::shared_ptr<Grid>& item) {
            return item.get() == grid.get();
        });

    if (it != m_grids.end()) {
        m_grids.erase(it);
    }
}

std::weak_ptr<Grid> GridSubsystem::getGridById(uint64_t id) const {
    auto it = m_idToGrid.find(id);
    return it != m_idToGrid.end() ? it->second : std::weak_ptr<Grid>{};
}

void GridSubsystem::stepUpdateGraphicsAll(const glm::dvec3& cameraPos) {
    for (auto& grid : m_grids) {
        grid->updateGraphics(cameraPos);
    }
}

bool GridSubsystem::handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    return m_gridSplitter->handlePendingSplits(endTime);
}

std::vector<glm::ivec3> GridSubsystem::removeCell(uint64_t gridId, const glm::ivec3& coord) {
    std::shared_ptr<Grid> grid{getGridById(gridId).lock()};
    if (!grid) {
        return {};
    }
    return grid->removeCell(coord);
}

void GridSubsystem::despawnGrid(uint64_t gridId) {
    removeGrid(getGridById(gridId));
}

bool GridSubsystem::modifyCell(uint64_t gridId, const glm::ivec3& coord, int cornerIndex,
                               const glm::ivec3& direction) {
    std::shared_ptr<Grid> grid{getGridById(gridId).lock()};
    if (!grid) {
        return false;
    }
    return grid->nudgeCellVertex(coord, cornerIndex, direction);
}

void GridSubsystem::scheduleSplitCheck(uint64_t gridId,
                                       const std::vector<glm::ivec3>& seedCoords) {
    std::shared_ptr<Grid> grid{getGridById(gridId).lock()};
    if (!grid || seedCoords.empty()) {
        return;
    }
    // Each seed and its neighbours are where a split could show up. Including the seed
    // itself covers a modified cell (still present); an emptied cell simply has no cell
    // there and is ignored by the analysis.
    static const glm::ivec3 s_directions[6]{{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                                            {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
    std::vector<glm::ivec3> edgeCoords{};
    edgeCoords.reserve(seedCoords.size() * 7);
    for (const glm::ivec3& seed : seedCoords) {
        edgeCoords.push_back(seed);
        for (const glm::ivec3& direction : s_directions) {
            edgeCoords.push_back(seed + direction);
        }
    }
    scheduleGridSplitCheck(grid, edgeCoords);
}

void GridSubsystem::applySplit(uint64_t sourceGridId,
                               const std::vector<GridSplitPiece>& pieces) {
    std::shared_ptr<Grid> sourceGrid{getGridById(sourceGridId).lock()};
    if (!sourceGrid) {
        return;
    }
    m_gridSplitter->applySplit(sourceGrid, pieces);
}

void GridSubsystem::scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    m_gridSplitter->scheduleGridSplitCheck(sourceGrid, edgeCoords);
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

std::weak_ptr<Grid> GridSubsystem::getGridFromBody(const std::weak_ptr<RigidBody>& body) const {
    std::shared_ptr<RigidBody> lockedBody{body.lock()};
    if (!lockedBody) {
        return {};
    }
    std::shared_ptr<Collider> collider{lockedBody->getPrimaryCollider().lock()};
    return collider ? getGridFromCollider(collider.get()) : std::weak_ptr<Grid>{};
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