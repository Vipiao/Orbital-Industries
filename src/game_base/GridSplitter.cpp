// GridSplitter.cpp
#include "GridSplitter.h"
#include "Grid.h"
#include "StructuralBlock.h"
#include "GridCell.h"
#include "CellType.h"
#include "thruster/ThrusterBlock.h"
#include "cockpit/CockpitBlock.h"
#include "utils/PartitionCalculator.h"
#include "utils/TimeHandler.h"
#include "../physics/RigidBody.h"
#include <algorithm>
#include <array>
#include <cassert>
#include <iostream>
#include <utility>

GridSplitter::GridSplitter(
    std::function<uint64_t()> allocateGridIdCallback,
    std::function<std::weak_ptr<Grid>(uint64_t, const glm::dvec3&, const glm::dquat&)>
        createGridCallback,
    TimeHandler* timeHandler)
    : m_allocateGridIdCallback(allocateGridIdCallback)
    , m_createGridCallback(createGridCallback)
    , m_timeHandler(timeHandler)
{
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
}

GridSplitter::~GridSplitter() {
    // Generator will be cleaned up automatically
}

void GridSplitter::scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGridWeak,
                                          const std::vector<glm::ivec3>& edgeCoords) {
    auto sourceGrid = sourceGridWeak.lock();
    if (!sourceGrid || edgeCoords.empty()) {
        return;
    }

    // Add edge coordinates to pending splits, automatically deduplicating
    PendingSplit& pending = m_pendingGridSplits[sourceGrid->uniqueId];
    pending.grid = sourceGridWeak;
    pending.edgeCoords.insert(edgeCoords.begin(), edgeCoords.end());
}

bool GridSplitter::handlePendingSplits(
    std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    m_endTime = endTime;

    // The queue refills only between drains; splits scheduled while a drain is
    // running wait for the next one.
    if (!m_activeSplitGenerator && m_splitQueue.empty()) {
        if (m_pendingGridSplits.empty()) {
            return false; // No work to do
        }
        // Queue in descending grid id so pop_back processes grids in creation
        // order, independent of map iteration order.
        std::vector<uint64_t> gridIds;
        gridIds.reserve(m_pendingGridSplits.size());
        for (const auto& pair : m_pendingGridSplits) {
            gridIds.push_back(pair.first);
        }
        std::sort(gridIds.begin(), gridIds.end(), std::greater<uint64_t>{});
        for (uint64_t gridId : gridIds) {
            m_splitQueue.push_back(std::move(m_pendingGridSplits[gridId]));
        }
        m_pendingGridSplits.clear();
    }

    while (m_timeHandler->now() < m_endTime) {
        if (!m_activeSplitGenerator) {
            if (m_splitQueue.empty()) {
                return false; // Drain complete
            }
            PendingSplit split = std::move(m_splitQueue.back());
            m_splitQueue.pop_back();
            std::vector<glm::ivec3> edgeVector{split.edgeCoords.begin(), split.edgeCoords.end()};
            m_activeSplitGenerator = std::make_unique<Generator<bool>>(
                performGridSplitAsync(std::move(split.grid), std::move(edgeVector)));
        }

        // Runs until the split completes or the time budget is spent. The done
        // check guards against resuming a finished coroutine (undefined
        // behavior), which an exception unwinding a previous slice can leave
        // behind.
        if (*m_activeSplitGenerator) {
            ++(*m_activeSplitGenerator);
        }

        if (!*m_activeSplitGenerator) {
            m_activeSplitGenerator.reset();
        }
    }

    return m_activeSplitGenerator != nullptr || !m_splitQueue.empty();
}

Generator<bool> GridSplitter::performGridSplitAsync(std::weak_ptr<Grid> sourceGridWeak,
                                                    std::vector<glm::ivec3> edgeCoords) {
    std::shared_ptr<Grid> sourceGrid = sourceGridWeak.lock();
    if (!sourceGrid || edgeCoords.empty()) {
        co_return;
    }

    // While nothing has been mutated yet, budget checks release the strong
    // reference during suspension so a grid removed in the pause can be
    // destroyed; the split aborts on resume if that happened.
    if (m_timeHandler->now() >= m_endTime) {
        sourceGrid.reset();
        co_yield true;
        sourceGrid = sourceGridWeak.lock();
        if (!sourceGrid) co_return;
    }

    // Step 1: Analyze partitions using the full cell registry (structural + thrusters)
    auto result = PartitionCalculator<GridCell*>::analyzePartitions(
        &sourceGrid->getCellRegistry(),
        edgeCoords,
        [](GridCell* const& cell) -> std::vector<glm::ivec3> {
            std::vector<glm::ivec3> neighbors;
            cell->forEachConnectedNeighbor([&](const glm::ivec3& neighbor) {
                neighbors.push_back(neighbor);
            });
            return neighbors;
        }
    );

    // Step 2: If no split detected, nothing to do
    if (!result.hasSplit || result.partitions.size() <= 1) {
        co_return;
    }

    if (m_timeHandler->now() >= m_endTime) {
        sourceGrid.reset();
        co_yield true;
        sourceGrid = sourceGridWeak.lock();
        if (!sourceGrid) co_return;
    }

    std::cout << "Grid split detected! " << result.partitions.size()
              << " partitions found." << std::endl;

    // The largest partition stays with the source grid; the rest become pieces.
    size_t largestPartitionIndex = 0;
    size_t largestSize = result.partitions[0].size();
    for (size_t i = 1; i < result.partitions.size(); ++i) {
        if (result.partitions[i].size() > largestSize) {
            largestSize = result.partitions[i].size();
            largestPartitionIndex = i;
        }
    }

    // Last chance to abort: applySplit mutates grids, so from here the strong
    // reference is held across suspensions and the operation always completes.
    if (m_timeHandler->now() >= m_endTime) {
        sourceGrid.reset();
        co_yield true;
        sourceGrid = sourceGridWeak.lock();
        if (!sourceGrid) co_return;
    }

    // Assign each non-largest partition a fresh id; the shared applySplit realises them.
    std::vector<GridSplitPiece> pieces;
    pieces.reserve(result.partitions.size() - 1);
    for (size_t i = 0; i < result.partitions.size(); ++i) {
        if (i == largestPartitionIndex) {
            continue;
        }
        pieces.push_back({m_allocateGridIdCallback(), result.partitions[i]});
    }

    applySplit(sourceGrid, pieces);
    m_completedSplits.push_back({sourceGrid->uniqueId, std::move(pieces)});
    std::cout << "Created split with new pieces" << std::endl;
}

void GridSplitter::applySplit(const std::shared_ptr<Grid>& sourceGrid,
                              const std::vector<GridSplitPiece>& pieces) {
    if (!sourceGrid || pieces.empty()) {
        return;
    }
    std::shared_ptr<RigidBody> sourceBody{sourceGrid->getRigidBody().lock()};
    if (!sourceBody) {
        return;
    }

    // Captured once from the un-mutated source: every piece inherits this motion.
    // The velocity field is anchored at the source's centre of mass, which must be
    // read before any cells move (removals shift it).
    glm::dvec3 originalVelocity{sourceBody->m_velocity};
    glm::dvec3 originalAngularVelocity{sourceBody->getAngularVelocityWorld()};
    glm::dquat originalOrientation{sourceBody->getOrientation()};
    glm::dvec3 originalCenterOfMass{sourceBody->getWorldCenterOfMass()};
    // Every piece is built in this frame. Removing the piece's cells shifts the
    // source's centre of mass but must leave its origin alone, or the pieces
    // already built would no longer line up with the ones still to come.
    const glm::dvec3 originalOrigin{sourceBody->getPosition()};

    for (size_t ii = 0; ii < pieces.size(); ii++) {
        assert(sourceBody->getPosition() == originalOrigin &&
               "source origin drifted while splitting; piece placement would be wrong");
        // A piece keeps the source's origin frame, so its cells keep their lattice
        // coords and world placement; only the mass properties are its own.
        std::shared_ptr<Grid> newGrid{
            m_createGridCallback(pieces[ii].m_newGridId, originalOrigin,
                                 originalOrientation).lock()};
        assert(newGrid);

        for (const glm::ivec3& coord : pieces[ii].m_coords) {
            GridCell* cell{sourceGrid->getCellFromRegistry(coord)};
            if (!cell) {
                continue;
            }
            switch (cell->type) {
            case CellType::STRUCTURAL_BLOCK: {
                const StructuralBlock* block{static_cast<const StructuralBlock*>(cell)};
                std::array<glm::ivec3, 8> savedVertices{block->m_localVertices};
                glm::dvec4 savedColor{block->m_color};
                sourceGrid->removeCell(coord);
                newGrid->addCell(coord);
                newGrid->modifyCell(coord, savedVertices);
                newGrid->setColor(coord, savedColor);
                break;
            }
            case CellType::THRUSTER: {
                const ThrusterBlock* thruster{static_cast<const ThrusterBlock*>(cell)};
                glm::dquat orientation{thruster->m_orientation};
                double thrustLevel{thruster->m_thrustLevel};
                sourceGrid->removeCell(coord);
                newGrid->addThruster(coord, orientation);
                // A burning engine that breaks off keeps burning.
                newGrid->setThrusterLevel(coord, thrustLevel);
                break;
            }
            case CellType::COCKPIT: {
                glm::dquat orientation{static_cast<const CockpitBlock*>(cell)->m_orientation};
                sourceGrid->removeCell(coord);
                newGrid->addCockpit(coord, orientation);
                break;
            }
            case CellType::THRUSTER_SECONDARY:
            case CellType::COCKPIT_SECONDARY:
                break;  // moved with their anchor cell
            }
        }

        std::shared_ptr<RigidBody> newBody{newGrid->getRigidBody().lock()};
        assert(newBody);
        assert(newBody->getMass() > 0.0);
        newBody->m_velocity = originalVelocity +
            glm::cross(originalAngularVelocity,
                       newBody->getWorldCenterOfMass() - originalCenterOfMass);
        newBody->setAngularVelocityBody(glm::conjugate(originalOrientation) * originalAngularVelocity);
    }

    // The source lost mass; refresh its angular momentum from the preserved velocity.
    sourceBody->setAngularVelocityBody(glm::conjugate(sourceBody->getOrientation()) *
                                       originalAngularVelocity);
}

std::vector<GridSplitResult> GridSplitter::drainCompletedSplits() {
    return std::exchange(m_completedSplits, {});
}
