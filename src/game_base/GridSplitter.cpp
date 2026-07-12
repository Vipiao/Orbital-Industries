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
#include <cassert>
#include <iostream>

GridSplitter::GridSplitter(
    std::function<std::weak_ptr<Grid>(const glm::dvec3&, const glm::dquat&)> createGridCallback,
    std::function<void(std::weak_ptr<Grid>)> removeGridCallback,
    TimeHandler* timeHandler)
    : m_createGridCallback(createGridCallback)
    , m_removeGridCallback(removeGridCallback)
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
    struct PartitionPhysics {
        glm::dvec3 centerOfMass;
        glm::dvec3 velocity;
        double mass;
    };

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

    // Step 3: Pre-calculate physics properties for each partition
    auto sourceBodyWeak = sourceGrid->getRigidBody();
    auto sourceBody = sourceBodyWeak.lock();
    if (!sourceBody) {
        std::cout << "Source grid has no rigid body - cannot split" << std::endl;
        co_return;
    }
    glm::dvec3 originalCenterOfMass = sourceBody->m_position;
    glm::dvec3 originalVelocity = sourceBody->m_velocity;

    // Calculate angular velocity from angular momentum
    glm::dvec3 originalAngularVelocity = sourceBody->getAngularVelocityWorld();
    glm::dquat originalOrientation = sourceBody->m_orientation;

    std::vector<PartitionPhysics> partitionPhysics(result.partitions.size());

    for (size_t i = 0; i < result.partitions.size(); ++i) {
        const std::vector<glm::ivec3>& partition = result.partitions[i];

        // Calculate center of mass for this partition
        glm::dvec3 weightedSum(0.0);
        double totalMass = 0.0;

        for (const glm::ivec3& coord : partition) {
            const GridCell* cell = sourceGrid->getCellFromRegistry(coord);
            if (!cell) continue;

            auto [blockMass, localCOM, inertiaTensor] = cell->getMassProperties();
            if (blockMass <= 0.0) continue; // ThrusterSecondaryCell has zero mass

            glm::dvec3 blockPosition = sourceGrid->gridToWorld(glm::dvec3(coord) + localCOM);
            weightedSum += blockPosition * blockMass;
            totalMass += blockMass;
        }

        // A partition of only massless cells would poison the rigid body with NaNs
        assert(totalMass > 0.0);
        glm::dvec3 partitionCenterOfMass = weightedSum / totalMass;

        // Calculate velocity using rigid body kinematics: v_p = v_t + ω × (cm_p - cm_t)
        glm::dvec3 relativePosition = partitionCenterOfMass - originalCenterOfMass;
        glm::dvec3 partitionVelocity =
            originalVelocity + glm::cross(originalAngularVelocity, relativePosition);

        partitionPhysics[i] = {partitionCenterOfMass, partitionVelocity, totalMass};

        std::cout << "Partition " << i << ": " << partition.size() << " cells, mass=" << totalMass
                  << ", CM=(" << partitionCenterOfMass.x << "," << partitionCenterOfMass.y << ","
                  << partitionCenterOfMass.z << ")"
                  << ", vel=(" << partitionVelocity.x << "," << partitionVelocity.y << ","
                  << partitionVelocity.z << ")" << std::endl;
    }

    // Step 4: Find the largest partition (this stays with the original grid)
    size_t largestPartitionIndex = 0;
    size_t largestSize = result.partitions[0].size();

    for (size_t i = 1; i < result.partitions.size(); ++i) {
        if (result.partitions[i].size() > largestSize) {
            largestSize = result.partitions[i].size();
            largestPartitionIndex = i;
        }
    }

    std::cout << "Largest partition (index " << largestPartitionIndex << ") has "
              << largestSize << " cells" << std::endl;

    // Last chance to abort: from here on the split mutates grids, so the strong
    // reference is held across suspensions and the operation always completes,
    // even if the source grid is removed externally mid-split.
    if (m_timeHandler->now() >= m_endTime) {
        sourceGrid.reset();
        co_yield true;
        sourceGrid = sourceGridWeak.lock();
        if (!sourceGrid) co_return;
    }

    // Step 5: Create new grids for all partitions except the largest
    std::vector<Grid*> newGrids;

    for (size_t i = 0; i < result.partitions.size(); ++i) {
        if (i == largestPartitionIndex) {
            continue; // Skip largest partition - it stays with original grid
        }

        const std::vector<glm::ivec3>& partition = result.partitions[i];

        // Create new grid using callback - position will be set after adding cells
        auto newGridWeak = m_createGridCallback(glm::dvec3(0.0), originalOrientation);
        auto newGrid = newGridWeak.lock();
        // The callback must return a live grid; everything below dereferences it
        assert(newGrid);

        std::cout << "Moving " << partition.size() << " cells to new grid" << std::endl;

        // Move cells from source grid to new grid
        size_t cellsProcessed = 0;
        for (const glm::ivec3& cellCoord : partition) {
            if (++cellsProcessed % 5 == 0 && m_timeHandler->now() >= m_endTime) {
                co_yield true;
            }

            GridCell* cell = sourceGrid->getCellFromRegistry(cellCoord);
            if (!cell) continue; // Already removed (e.g. thruster secondary handled via anchor)

            switch (cell->type) {
            case CellType::STRUCTURAL_BLOCK: {
                const StructuralBlock* block = static_cast<const StructuralBlock*>(cell);
                std::array<glm::ivec3, 8> savedVertices = block->m_localVertices;
                glm::dvec4 savedColor = block->m_color;
                sourceGrid->removeCell(cellCoord);
                newGrid->addCell(cellCoord);
                newGrid->modifyCell(cellCoord, savedVertices);
                newGrid->setColor(cellCoord, savedColor);
                break;
            }
            case CellType::THRUSTER: {
                const ThrusterBlock* thruster = static_cast<const ThrusterBlock*>(cell);
                const glm::dquat ori = thruster->m_orientation;
                const double thrustLevel = thruster->m_thrustLevel;
                sourceGrid->removeCell(cellCoord);
                newGrid->addThruster(cellCoord, ori);
                // The throttle is simulation state: a burning engine that breaks
                // off keeps burning.
                newGrid->setThrusterLevel(cellCoord, thrustLevel);
                break;
            }
            case CellType::COCKPIT: {
                const glm::dquat ori = static_cast<const CockpitBlock*>(cell)->m_orientation;
                sourceGrid->removeCell(cellCoord);
                newGrid->addCockpit(cellCoord, ori);
                break;
            }
            case CellType::THRUSTER_SECONDARY:
            case CellType::COCKPIT_SECONDARY:
                break; // handled when their anchor cell is processed above
            }
        }

        // Set physics properties using pre-calculated values
        auto newBodyWeak = newGrid->getRigidBody();
        auto newBody = newBodyWeak.lock();
        // Without a body the new grid would be left at the origin with no velocity
        assert(newBody);
        if (newBody) {
            // Transform center of mass to world space and set position
            glm::dvec3 worldCenterOfMass = partitionPhysics[i].centerOfMass;
            newBody->m_position = worldCenterOfMass;
            newBody->m_velocity = partitionPhysics[i].velocity;

            // Set angular momentum instead of angular velocity
            glm::dvec3 angularVelocityBody =
                glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
            newBody->setAngularVelocityBody(angularVelocityBody);
            newBody->m_orientation = originalOrientation;
        }

        newGrids.push_back(newGrid.get()); // Keep raw pointer for local processing
    }

    // Recalculate original partition local angular momentum as its mass has now changed.
    glm::dvec3 angularVelocityBody =
        glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
    sourceBody->setAngularVelocityBody(angularVelocityBody);

    std::cout << "Created " << newGrids.size() << " new grids from split" << std::endl;
}
