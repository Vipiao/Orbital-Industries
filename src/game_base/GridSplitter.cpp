// GridSplitter.cpp
#include "GridSplitter.h"
#include "Grid.h"
#include "StructuralBlock.h"
#include "GridCell.h"
#include "CellType.h"
#include "thruster/ThrusterBlock.h"
#include "cockpit/CockpitBlock.h"
#include "../utils/PartitionCalculator.h"
#include "../utils/TimeHandler.h"
#include "../physics/RigidBody.h"
#include <iostream>

GridSplitter::GridSplitter(
    std::function<std::weak_ptr<Grid>(const glm::dvec3&, const glm::dquat&)> createGridCallback,
    std::function<void(std::weak_ptr<Grid>)> removeGridCallback,
    std::function<Grid*(uint64_t)> findGridByIdCallback,
    TimeHandler* timeHandler)
    : m_createGridCallback(createGridCallback)
    , m_removeGridCallback(removeGridCallback)
    , m_findGridByIdCallback(findGridByIdCallback)
    , m_timeHandler(timeHandler)
{
    if (!m_timeHandler) {
        throw std::runtime_error("TimeHandler cannot be null");
    }
}

GridSplitter::~GridSplitter() {
    // Generator will be cleaned up automatically
}

void GridSplitter::scheduleGridSplitCheck(std::weak_ptr<Grid> sourceGridWeak, const std::vector<glm::ivec3>& edgeCoords) {
    auto sourceGrid = sourceGridWeak.lock();
    if (!sourceGrid || edgeCoords.empty()) {
        return;
    }
    
    // Add edge coordinates to pending splits, automatically deduplicating
    auto& pendingEdges = m_pendingGridSplits[sourceGrid.get()->uniqueId];
    pendingEdges.insert(edgeCoords.begin(), edgeCoords.end());
}

bool GridSplitter::handlePendingSplits(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    if (!m_pendingSplitsGenerator) {
        if (m_pendingGridSplits.empty()) {
            return false; // No work to do
        }
        m_pendingSplitsGenerator = std::make_unique<Generator<bool>>(handlePendingSplitsAsync());
        ++(*m_pendingSplitsGenerator); // Start the generator
    }
    
    while (*m_pendingSplitsGenerator && m_timeHandler->now() < endTime) {
        ++(*m_pendingSplitsGenerator);
    }
    
    if (!*m_pendingSplitsGenerator) {
        // Generator finished
        m_pendingSplitsGenerator.reset();
        return false; // Work complete
    }
    
    return true; // More work needed
}

Generator<bool> GridSplitter::handlePendingSplitsAsync() {
    if (m_pendingGridSplits.empty()) {
        co_return;
    }
    
    // Snapshot pending splits to avoid race conditions
    auto pendingSplitsSnapshot = std::move(m_pendingGridSplits);
    // m_pendingGridSplits is now empty and ready for new entries

    // Process snapshotted splits
    for (const auto& pair : pendingSplitsSnapshot) {
        uint64_t gridId = pair.first;
        const auto& edgeCoords = pair.second;

        // Find the grid by ID using callback
        Grid* sourceGrid = m_findGridByIdCallback(gridId);
        
        if (!sourceGrid) continue; // Grid was deleted
        
        // Convert unordered_set back to vector for the analysis function
        std::vector<glm::ivec3> edgeVector(edgeCoords.begin(), edgeCoords.end());
        
        // Use async grid split generator
        auto splitGenerator = performGridSplitAsync(sourceGrid, edgeVector);
        ++splitGenerator; // Start the generator
        
        while (splitGenerator) {
            co_yield true; // Yield control for time checking
            ++splitGenerator;
        }
    }
}

Generator<bool> GridSplitter::performGridSplitAsync(Grid* sourceGrid, const std::vector<glm::ivec3>& edgeCoords) {
    std::vector<Grid*> newGrids;

    struct PartitionPhysics {
        glm::dvec3 centerOfMass;
        glm::dvec3 velocity;
        double mass;
    };
    
    if (!sourceGrid || edgeCoords.empty()) {
        co_return;
    }

    co_yield true; // Allow time check before expensive operation
    
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

    co_yield true; // Allow time check after expensive analysis
    
    // Step 2: If no split detected, return empty vector
    if (!result.hasSplit || result.partitions.size() <= 1) {
        co_return;
    }
    
    std::cout << "Grid split detected! " << result.partitions.size() << " partitions found." << std::endl;
    
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

    co_yield true; // Allow time check before physics calculations
    
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
        
        glm::dvec3 partitionCenterOfMass = weightedSum / totalMass;
        
        // Calculate velocity using rigid body kinematics: v_p = v_t + ω × (cm_p - cm_t)
        glm::dvec3 relativePosition = partitionCenterOfMass - originalCenterOfMass;
        glm::dvec3 partitionVelocity = originalVelocity + glm::cross(originalAngularVelocity, relativePosition);
        
        partitionPhysics[i] = {partitionCenterOfMass, partitionVelocity, totalMass};
        
        std::cout << "Partition " << i << ": " << partition.size() << " cells, mass=" << totalMass 
                  << ", CM=(" << partitionCenterOfMass.x << "," << partitionCenterOfMass.y << "," << partitionCenterOfMass.z << ")"
                  << ", vel=(" << partitionVelocity.x << "," << partitionVelocity.y << "," << partitionVelocity.z << ")" << std::endl;
    }

    co_yield true; // Allow time check before grid operations
    
    // Step 4: Find the largest partition (this stays with the original grid)
    size_t largestPartitionIndex = 0;
    size_t largestSize = result.partitions[0].size();
    
    for (size_t i = 1; i < result.partitions.size(); ++i) {
        if (result.partitions[i].size() > largestSize) {
            largestSize = result.partitions[i].size();
            largestPartitionIndex = i;
        }
    }
    
    std::cout << "Largest partition (index " << largestPartitionIndex << ") has " << largestSize << " cells" << std::endl;
    
    // Step 5: Create new grids for all partitions except the largest

    co_yield true; // Allow time check before creating new grids
    
    for (size_t i = 0; i < result.partitions.size(); ++i) {
        if (i == largestPartitionIndex) {
            continue; // Skip largest partition - it stays with original grid
        }
        
        const std::vector<glm::ivec3>& partition = result.partitions[i];
        
        // Create new grid using callback - position will be set after adding cells
        auto newGridWeak = m_createGridCallback(glm::dvec3(0.0), originalOrientation);
        auto newGrid = newGridWeak.lock();
        
        std::cout << "Moving " << partition.size() << " cells to new grid" << std::endl;
        
        co_yield true; // Allow time check before moving cells

        // Move cells from source grid to new grid
        size_t cellsProcessed = 0;
        for (const glm::ivec3& cellCoord : partition) {
            GridCell* cell = sourceGrid->getCellFromRegistry(cellCoord);
            if (!cell) continue; // Already removed (e.g. thruster secondary handled via anchor)

            if (cell->type == CellType::STRUCTURAL_BLOCK) {
                const StructuralBlock* block = static_cast<const StructuralBlock*>(cell);
                std::array<glm::ivec3, 8> savedVertices = block->m_localVertices;
                glm::dvec4 savedColor = block->m_color;

                sourceGrid->removeCell(cellCoord);
                newGrid->addCell(cellCoord);
                newGrid->modifyCell(cellCoord, savedVertices);
                newGrid->setColor(cellCoord, savedColor);
            } else if (cell->type == CellType::THRUSTER) {
                // removeCell handles both anchor and secondary; addThruster re-creates both
                const glm::dquat thrusterOrientation = static_cast<const ThrusterBlock*>(cell)->m_orientation;
                sourceGrid->removeCell(cellCoord);
                newGrid->addThruster(cellCoord, thrusterOrientation);
            } else if (cell->type == CellType::COCKPIT) {
                const glm::dquat cockpitOrientation = static_cast<const CockpitBlock*>(cell)->m_orientation;
                sourceGrid->removeCell(cellCoord);
                newGrid->addCockpit(cellCoord, cockpitOrientation);
            }
            // THRUSTER_SECONDARY / COCKPIT_SECONDARY: skip — handled when anchor is processed above

            if (++cellsProcessed % 5 == 0) {
                co_yield true;
            }
        }

        // Set physics properties using pre-calculated values
        auto newBodyWeak = newGrid->getRigidBody();
        auto newBody = newBodyWeak.lock();
        if (newBody) {
            // Transform center of mass to world space and set position
            glm::dvec3 worldCenterOfMass = partitionPhysics[i].centerOfMass;
            newBody->m_position = worldCenterOfMass;
            newBody->m_velocity = partitionPhysics[i].velocity;
            
            // Set angular momentum instead of angular velocity
            glm::dvec3 angularVelocityBody = glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
            newBody->setAngularVelocityBody(angularVelocityBody);
            newBody->m_orientation = originalOrientation;
        }
        
        newGrids.push_back(newGrid.get()); // Keep raw pointer for local processing
    }

    // Recalculate original partition local angular momentum as its mass has now changed.
    glm::dvec3 angularVelocityBody = glm::conjugate(sourceBody->m_orientation) * originalAngularVelocity;
    sourceBody->setAngularVelocityBody(angularVelocityBody);
    
    std::cout << "Created " << newGrids.size() << " new grids from split" << std::endl;
}