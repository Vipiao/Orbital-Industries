// Grid.cpp

#include <set>
#include "Grid.h"
#include "GridGraphics.h"
#include "CellType.h"
#include <glm/gtc/matrix_transform.hpp>
#include "../physics/PhysicsEngine.h"
#include "../physics/GridCollider.h"
#include "utils/MassInertiaCalculator.h"
#include "utils/TimeHandler.h"
#include "utils/JobManager.h"
#include "debug/DebugGlobals.h"
#include "../physics/CubeCollider.h"
#include <limits>
#include <iostream>
#include "debug/DebugRenderer.h"
#include "../game_base/JobPriorities.h"
#include "../physics/PolyhedronCollider.h"
#include "utils/GridGeometry.h"
#include "utils/PolyhedronProcessor.h"
#include "utils/GeometryUtils.h"

// Initialize static counter
Grid::Grid(uint64_t uniqueId, PhysicsEngine* physics, GraphicsEngine* graphics,
           JobManager* jobManager, TimeHandler* timeHandler, BlockResourceCache* blockResources,
           const glm::dvec3& position, const glm::dquat& orientation)
    : uniqueId(uniqueId), m_jobManager(jobManager), m_timeHandler(timeHandler), m_physics(physics)
{
    if (!m_jobManager || !m_timeHandler) {
        throw std::invalid_argument("JobManager and TimeHandler cannot be null");
    }

    // Create grid collider via CollisionDetector factory
    m_colliderWeak = m_physics->getCollisionDetector().addGridCollider(
        position,
        orientation,
        jobManager,
        timeHandler,
        JobPriorities::GRID_CELL_CLASSIFICATION);

    // Store back-reference to Grid in the collider for sensor queries
    if (auto collider = m_colliderWeak.lock()) {
        collider->set_pointer<Grid>(this);
    }
    
    // Create rigid body (without collider)
    m_rigidBody = m_physics->addRigidBody(
        position,
        orientation,
        0.0,               // Mass (will be updated when cells are added)
        glm::dmat3(0.0),   // Inertia tensor (will be updated when cells are added)
        false);            // Not static
    
    // Attach collider to rigid body. The collider's lattice shares the body's
    // origin frame, so the attachment offset is zero for the grid's lifetime.
    m_physics->attachCollider(m_rigidBody, m_colliderWeak);

    // Create graphics subsystem
    m_gridGraphics = std::make_unique<GridGraphics>(graphics, jobManager, blockResources);

    // Initial graphics update
    // Use a default distant camera position for initial update
    updateGraphics(glm::dvec3(0.0, 0.0, 100.0));
}

Grid::~Grid() {
    // Order matters: disconnect references BEFORE destroying collider
    if (auto collider = m_colliderWeak.lock()) {
        collider->remove_pointer<Grid>();
    }
    
    // Disconnect and remove physics components
    if (!m_rigidBody.expired()) {
        m_physics->removeRigidBody(m_rigidBody);
    }

    // Now safe to destroy collider (no more references to it)
    m_physics->getCollisionDetector().removeCollider(m_colliderWeak);

    // Cancel all pending jobs
    for (auto& jobHandle : m_pendingJobs) {
        if (!jobHandle.expired()) {
            m_jobManager->cancel(jobHandle);
        }
    }

    // Cancel any pending analysis job
    cancelStructuralAnalysis();
}

std::weak_ptr<Collider> Grid::getCollider() const {
    return m_colliderWeak;
}

// Add a cell to the grid
void Grid::addCell(const glm::ivec3& coord) {
    // If cell already exists, return
    if (hasCell(coord)) return;

    // Add cell to collider using factory method
    auto collider = m_colliderWeak.lock();
    if (!collider) {
        throw std::runtime_error("Grid::addCell: Collider has been destroyed");
    }
    collider->addCubeCell(coord, 1.0);

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();
    
    // Create structural block
    StructuralBlock block(coord);
    
    // Move block into cells map and register in cell registry
    auto [it, _] = m_cells.emplace(coord, std::move(block));
    m_cellRegistry[coord] = &it->second;

    // Schedule structural analysis
    scheduleStructuralAnalysis();

    // Update neighbor connections after all other setup
    updateNeighborConnections(coord);
    
    // Preserve angular velocity across mass change
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("Grid::addCell: RigidBody has been destroyed");
    }
    
    glm::dvec3 originalAngularVelocity = rigidBody->getAngularVelocityBody();
    glm::dvec3 angularVelocityWorld = rigidBody->getAngularVelocityWorld();

    // Add mass contribution
    updateCellMassContribution(coord, 1.0, angularVelocityWorld);

    rigidBody->setAngularVelocityBody(originalAngularVelocity);

    // Schedule mesh updates for this cell and neighbors
    scheduleMeshUpdatesForCellAndNeighbors(coord);
    m_structureVersion++;
}

void Grid::addCell(const glm::ivec3& coord, const std::array<glm::ivec3, 8>& vertices,
                   const glm::dvec4& color) {
    if (hasCell(coord)) {
        return;  // an occupied coord keeps the block already there, shape and colour
    }
    addCell(coord);
    if (canModifyCell(coord, vertices)) {
        modifyCell(coord, vertices);
    }
    setColor(coord, color);
}

// Remove a cell (structural block or special block) from the grid.
// Returns all coordinates that were removed.
std::vector<glm::ivec3> Grid::removeCell(const glm::ivec3& coord) {
    auto regIt = m_cellRegistry.find(coord);
    if (regIt == m_cellRegistry.end()) return {};

    GridCell* cell = regIt->second;

    // Secondary cells delegate mass ownership to their anchor; structural blocks and
    // anchor cells are their own physics representative.
    const glm::ivec3 anchorCoord{cell->getAnchorCoord()};

    cancelStructuralAnalysis();

    // Preserve angular velocity across the mass change — same for all block types.
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) throw std::runtime_error("Grid::removeCell: RigidBody has been destroyed");
    const glm::dvec3 angVel = rigidBody->getAngularVelocityBody();
    updateCellMassContribution(anchorCoord, -1.0, rigidBody->getAngularVelocityWorld());
    rigidBody->setAngularVelocityBody(angVel);

    std::vector<glm::ivec3> removed;

    if (cell->type == CellType::STRUCTURAL_BLOCK) {
        // Schedule mesh updates before removing so neighbors still exist
        scheduleMeshUpdatesForCellAndNeighbors(coord);
        removeNeighborConnections(coord);

        if (auto collider = m_colliderWeak.lock()) collider->removeCell(coord);
        m_gridGraphics->removeCell(coord);
        m_cellRegistry.erase(coord);
        m_cells.erase(coord);

        removed = {coord};
    } else {
        auto* anchor = static_cast<BlockAnchor*>(m_cellRegistry.at(anchorCoord));
        const std::vector<glm::ivec3> secondaries = anchor->secondaryCoords();

        removeNeighborConnections(anchorCoord);
        for (const auto& sc : secondaries) removeNeighborConnections(sc);

        if (auto collider = m_colliderWeak.lock()) {
            collider->removeCell(anchorCoord);
            for (const auto& sc : secondaries) collider->removeCell(sc);
        }

        m_gridGraphics->removeBlockInstance(anchorCoord);

        m_cellRegistry.erase(anchorCoord);
        for (const auto& sc : secondaries) {
            m_cellRegistry.erase(sc);
            m_secondaryCells.erase(sc);
        }

        // Type-specific erase from the anchor's owning map
        if (anchor->type == CellType::THRUSTER)            m_thrusterCells.erase(anchorCoord);
        else if (anchor->type == CellType::COCKPIT)        m_cockpitCells.erase(anchorCoord);
        else if (anchor->type == CellType::REACTION_WHEEL) m_reactionWheelCells.erase(anchorCoord);

        removed = {anchorCoord};
        removed.insert(removed.end(), secondaries.begin(), secondaries.end());
    }

    scheduleStructuralAnalysis();
    m_structureVersion++;
    return removed;
}

template <typename BlockT>
void Grid::addBlock(std::unordered_map<glm::ivec3, BlockT, Hash::IVec3Hash>& anchors,
                    const glm::ivec3& anchorCoord, const glm::dquat& orientation) {
    const std::vector<glm::ivec3> offsets{BlockT::footprintOffsets(orientation)};

    // Every cell the block fills must be free
    for (const glm::ivec3& offset : offsets) {
        if (m_cellRegistry.count(anchorCoord + offset)) return;
    }

    auto collider = m_colliderWeak.lock();
    if (!collider) throw std::runtime_error("Grid::addBlock: Collider has been destroyed");
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) throw std::runtime_error("Grid::addBlock: RigidBody has been destroyed");

    cancelStructuralAnalysis();

    for (const glm::ivec3& offset : offsets) {
        collider->addCubeCell(anchorCoord + offset, 1.0);
    }

    // Emplace into owning maps — unordered_map guarantees stable references even after rehash
    auto [anchorIt, _a] = anchors.emplace(anchorCoord, BlockT{anchorCoord, orientation});
    m_cellRegistry[anchorCoord] = &anchorIt->second;

    for (const glm::ivec3& secondary : anchorIt->second.secondaryCoords()) {
        auto [secIt, _s] = m_secondaryCells.emplace(
            secondary, SecondaryCell{secondary, anchorCoord});
        m_cellRegistry[secondary] = &secIt->second;
    }

    m_gridGraphics->addBlockInstance(BlockT::TYPE, anchorCoord, orientation, BlockT::MODEL_CENTRE);

    scheduleStructuralAnalysis();

    for (const glm::ivec3& offset : offsets) {
        updateNeighborConnections(anchorCoord + offset);
    }

    // Preserve angular velocity across the mass change
    const glm::dvec3 angularVelocityBody{rigidBody->getAngularVelocityBody()};
    updateCellMassContribution(anchorCoord, 1.0, rigidBody->getAngularVelocityWorld());
    rigidBody->setAngularVelocityBody(angularVelocityBody);
    m_structureVersion++;
}

void Grid::addThruster(const glm::ivec3& anchorCoord, const glm::dquat& orientation) {
    addBlock(m_thrusterCells, anchorCoord, orientation);
}

void Grid::setThrusterLevel(const glm::ivec3& anchorCoord, double level) {
    auto it = m_thrusterCells.find(anchorCoord);
    if (it == m_thrusterCells.end() || it->second.m_thrustLevel == level) {
        return;
    }
    it->second.m_thrustLevel = level;
    // Write-through to the plume: pilot commands rewrite every thruster each
    // step, so the change gate above keeps GPU uploads to actual transitions.
    m_gridGraphics->setPlumeThrust(anchorCoord, level);
}

void Grid::addCockpit(const glm::ivec3& anchorCoord, const glm::dquat& orientation) {
    addBlock(m_cockpitCells, anchorCoord, orientation);
}

void Grid::addReactionWheel(const glm::ivec3& anchorCoord, const glm::dquat& orientation) {
    addBlock(m_reactionWheelCells, anchorCoord, orientation);
}

void Grid::setReactionWheelCommand(const glm::ivec3& anchorCoord, const glm::dvec3& command) {
    auto it = m_reactionWheelCells.find(anchorCoord);
    if (it == m_reactionWheelCells.end()) {
        return;
    }
    it->second.m_torqueCommand = command;
}

bool Grid::nudgeCellVertex(const glm::ivec3& coord, int cornerIndex,
                           const glm::ivec3& direction) {
    if (cornerIndex < 0 || cornerIndex >= 8) {
        return false;  // index arrives over the wire; guard the array access
    }
    auto it = m_cells.find(coord);
    if (it == m_cells.end()) {
        return false;
    }
    // Nudge from the current vertices, so repeated nudges compose on this state.
    std::array<glm::ivec3, 8> newVertices = it->second.m_localVertices;
    newVertices[cornerIndex] += direction;
    if (!canModifyCell(coord, newVertices)) {
        return false;  // rejects out-of-bounds or otherwise invalid shapes
    }
    return modifyCell(coord, newVertices);
}

bool Grid::canModifyCell(const glm::ivec3& coord, const std::array<glm::ivec3, 8>& newVertices) const {
    // Check if cell exists
    if (!hasCell(coord)) {
        return false;
    }
    
    // Validate the vertices using static validation function
    return StructuralBlock::validateVertices(newVertices);
}

bool Grid::modifyCell(const glm::ivec3& coord, const std::array<glm::ivec3, 8>& newVertices) {
    
    // Get the existing structural block
    auto it = m_cells.find(coord);
    StructuralBlock& block = it->second;
    
    // Cancel existing analysis to prevent accessing modified cells
    cancelStructuralAnalysis();

    // Preserve angular velocity across entire modification
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("Grid::modifyCell: RigidBody has been destroyed");
    }
    
    glm::dvec3 originalAngularVelocity = rigidBody->getAngularVelocityBody();
    // One fixed angular velocity for the whole modification: the remove and re-add
    // below both shift the centre of mass, and both fixes must see the same spin.
    glm::dvec3 angularVelocityWorld = rigidBody->getAngularVelocityWorld();

    // Remove old mass contribution
    updateCellMassContribution(coord, -1.0, angularVelocityWorld);

    // Update the block's vertices
    bool success = block.setVertices(newVertices);
    if (!success) {
        // This should never happen since we validated, but if it does it's a serious error
        return false;
    }
    
    // Get new collision axes and vertices
    auto axes = block.getAxes();
    // Convert integer vertices to normalized collision vertices
    std::vector<glm::dvec3> vertices;
    vertices.reserve(8);
    for (const auto& vertex : block.m_localVertices) {
        vertices.push_back(glm::dvec3(vertex) / double(StructuralBlock::MAX_SIZE) - glm::dvec3(0.5));
    }
    
    // Remove old cell from collider and add new one using factory method
    auto collider = m_colliderWeak.lock();
    if (!collider) {
        throw std::runtime_error("Grid::modifyCell: Collider has been destroyed");
    }
    collider->removeCell(coord);
    collider->addPolyhedronCell(coord, vertices, axes.faceAxis, axes.edgeAxis, axes.edges);
    
    // Schedule mesh updates for this cell and neighbors
    scheduleMeshUpdatesForCellAndNeighbors(coord);

    // Add new mass contribution
    updateCellMassContribution(coord, 1.0, angularVelocityWorld);

    // Restore angular velocity after entire modification
    rigidBody->setAngularVelocityBody(originalAngularVelocity);

    // Update neighbor connections since geometry changed
    updateNeighborConnections(coord);
    
    // Schedule structural analysis
    scheduleStructuralAnalysis();

    m_structureVersion++;
    return true;
}

void Grid::setColor(const glm::ivec3& coord, const glm::dvec4& newColor) {
    // Check if cell exists
    auto it = m_cells.find(coord);
    if (it == m_cells.end()) {
        return; // Cell doesn't exist
    }
    
    StructuralBlock& block = it->second;
    
    // Update the color
    block.m_color = newColor;
    
    // Schedule mesh update for color change
    scheduleMeshUpdateForCell(coord);

    // No need to update physics/mass since only color changed
    m_structureVersion++;
}

void Grid::trackJob(std::weak_ptr<Job> jobHandle) {
    // Clean up expired handles periodically to prevent unbounded growth
    if (m_pendingJobs.size() % 50 == 0) {
        m_pendingJobs.erase(std::remove_if(m_pendingJobs.begin(), m_pendingJobs.end(),
            [](const std::weak_ptr<Job>& handle) { return handle.expired(); }), m_pendingJobs.end());
    }
    
    m_pendingJobs.push_back(jobHandle);
}

StructuralBlock* Grid::getCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    return (it != m_cells.end()) ? &it->second : nullptr;
}

GridCell* Grid::getCellFromRegistry(const glm::ivec3& coord) {
    auto it = m_cellRegistry.find(coord);
    return (it != m_cellRegistry.end()) ? it->second : nullptr;
}

int Grid::getGridSSBOIndex() const {
    return m_gridGraphics ? m_gridGraphics->getSSBOIndex() : -1;
}

void Grid::updateNeighborConnections(const glm::ivec3& coord) {
    // Direction mapping: Right, Left, Front, Back, Top, Bottom
    static const glm::ivec3 directions[6] = {
        {1, 0, 0},   // Right  (index 0)
        {-1, 0, 0},  // Left   (index 1)
        {0, 1, 0},   // Front  (index 2)
        {0, -1, 0},  // Back   (index 3)
        {0, 0, 1},   // Top    (index 4)
        {0, 0, -1}   // Bottom (index 5)
    };

    // Opposite direction mapping
    static const int oppositeDir[6] = {1, 0, 3, 2, 5, 4};

    auto cellIt = m_cellRegistry.find(coord);
    if (cellIt == m_cellRegistry.end()) return;
    GridCell* cell = cellIt->second;

    // Get this cell's vertices in grid space
    std::vector<glm::dvec3> cellVertices = cell->getLocalVertices();
    for (auto& vertex : cellVertices) vertex += glm::dvec3(coord);

    // Update this cell's neighbor pointers and update neighbors to point back
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        auto neighborIt = m_cellRegistry.find(neighborCoord);

        bool hasConnection = false;
        GridCell* neighbor = nullptr;

        if (neighborIt != m_cellRegistry.end()) {
            neighbor = neighborIt->second;

            // Get neighbor's vertices in grid space
            std::vector<glm::dvec3> neighborVertices = neighbor->getLocalVertices();
            for (auto& vertex : neighborVertices) vertex += glm::dvec3(neighborCoord);

            // Use direction as face normal
            double overlapArea = GeometryUtils::calculateSurfaceOverlapArea(
                cellVertices, neighborVertices, glm::dvec3(directions[i]));
            hasConnection = (overlapArea > 0.05);
        }

        cell->neighbors[i] = hasConnection ? neighbor : nullptr;

        // If neighbor exists, make it point back to this cell
        if (neighbor) {
            neighbor->neighbors[oppositeDir[i]] = hasConnection ? cell : nullptr;
        }
    }
}

void Grid::removeNeighborConnections(const glm::ivec3& coord) {
    auto cellIt = m_cellRegistry.find(coord);
    if (cellIt == m_cellRegistry.end()) return;
    GridCell* cell = cellIt->second;

    // Remove this cell from all its neighbors' pointer arrays
    for (int i = 0; i < 6; ++i) {
        GridCell* neighbor = cell->neighbors[i];
        if (neighbor) {
            for (int j = 0; j < 6; ++j) {
                if (neighbor->neighbors[j] == cell) {
                    neighbor->neighbors[j] = nullptr;
                }
            }
        }
    }
}

void Grid::visualizeStructuralIntegrity() {
    // Clear existing cost-based debug spheres
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("cost_cell_");
    }
    
    // Update debug visualization using current weakness values
    if (DebugGlobals::getDebugRenderer()) {
        for (const auto& pair : m_cells) {
            const glm::ivec3& coord = pair.first;
            const StructuralBlock& cell = pair.second;
            
            if (cell.structuralWeakness > 1.0) {
                glm::dvec3 cellWorldPos = gridToWorld(glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5));
                std::string sphereName = "cost_cell_" + std::to_string(coord.x) + "_" + 
                                       std::to_string(coord.y) + "_" + std::to_string(coord.z);
                double radius = 0.0 + glm::pow((cell.structuralWeakness - 1.0) * 0.02, 1./3.);
                DebugGlobals::getDebugRenderer()->createSphere(sphereName, cellWorldPos, radius);
            }
        }
    }
    
    std::cout << "Structural analysis visualization updated for " << m_cells.size() << " cells" << std::endl;
}

void Grid::scheduleStructuralAnalysis() {
    // Schedule new analysis job with low priority (don't cancel existing or reset state)
    m_analysisJob = m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
        return performStructuralAnalysisUntil(endTime);
    }, JobPriorities::STRUCTURAL_ANALYSIS);
}

void Grid::cancelStructuralAnalysis() {
    if (!m_analysisJob.expired()) {
        m_jobManager->cancel(m_analysisJob);
        m_currentAnalysisIteration = 0;
    }
}

bool Grid::performStructuralAnalysisUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    
    // Check if analysis is complete
    if (m_currentAnalysisIteration >= MAX_ANALYSIS_ITERATIONS) {
        return false; // Analysis complete
    }

    //return false;
    
    // Create stochastic analyzer if it doesn't exist
    if (!m_stochasticAnalyzer) {
        m_stochasticAnalyzer = std::make_unique<StochasticAnalyzer<StructuralBlock>>(m_cells);
    }
    
    // Run analysis until time runs out or iteration completes
    bool needsMoreTime = m_stochasticAnalyzer->performAnalysisUntil(endTime, *m_timeHandler);
    //return false;
    if (!needsMoreTime) {
        // Current iteration completed, update running averages
        for (auto& pair : m_cells) {
            StructuralBlock& cell = pair.second;
            double currentCost = static_cast<double>(cell.cost);
            
            if (cell.structuralWeakness >= 0.0) {
                // Update running average: new = old * (1-f) + current * f
                double oldWeakness = cell.structuralWeakness;
                double newWeakness = oldWeakness * (1.0 - WEAKNESS_BLEND_FACTOR) + 
                                   currentCost * WEAKNESS_BLEND_FACTOR;
                cell.structuralWeakness = newWeakness;
            } else {
                // First data point (was -1)
                cell.structuralWeakness = currentCost;
            }
        }
        
        // Move to next iteration
        m_currentAnalysisIteration++;
        
        // Reset analyzer state for next iteration (reuses same object)
        m_stochasticAnalyzer->resetAnalysis();
        
        std::cout << "Completed structural analysis iteration " << m_currentAnalysisIteration 
                  << "/" << MAX_ANALYSIS_ITERATIONS << " for " << m_cells.size() << " cells" << std::endl;
        
        // Check if all iterations complete
        if (m_currentAnalysisIteration >= MAX_ANALYSIS_ITERATIONS) {
            std::cout << "Background structural analysis complete for grid with " << m_cells.size() << " cells" << std::endl;
            m_currentAnalysisIteration = 0;
            return false; // Analysis complete
        }
    }
    
    return true; // More work needed (either current iteration or more iterations)
}

// Check if a cell of any type exists at the given coordinates
bool Grid::hasCell(const glm::ivec3& coord) const {
    return m_cellRegistry.find(coord) != m_cellRegistry.end();
}

// Check if the grid has no cells of any type
bool Grid::isEmpty() const {
    return m_cellRegistry.empty();
}

void Grid::updateCellMassContribution(const glm::ivec3& coord, double sign,
                                      const glm::dvec3& angularVelocityWorld) {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("Grid::updateCellMassContribution: RigidBody has been destroyed");
    }

    assert(RigidBodyDetail::isFinite(angularVelocityWorld) &&
           "caller must supply the finite angular velocity the edit happens at");
    const glm::dvec3 originBefore{rigidBody->getPosition()};
    glm::dvec3 oldWorldCenterOfMass{rigidBody->getWorldCenterOfMass()};

    // Lambda to get cell properties with sign applied (works for all cell types)
    auto getProperties = [this, sign](const glm::ivec3& cellCoord) -> std::tuple<double, glm::dvec3, glm::dmat3> {
        auto regIt = m_cellRegistry.find(cellCoord);
        if (regIt != m_cellRegistry.end()) {
            auto [mass, localCOM, inertia] = regIt->second->getMassProperties();
            return {mass * sign, localCOM, inertia * sign};
        }
        return {0.0, glm::dvec3(0.0), glm::dmat3(0.0)};
    };

    // Update mass contribution using coordinate-based calculation
    double mass{rigidBody->getMass()};
    glm::dvec3 centerOfMassLocal{rigidBody->getCenterOfMassLocal()};
    glm::dmat3 inertiaTensor{rigidBody->getInertiaTensor()};
    MassInertiaCalculator::calculateInertiaForCoords(
        {coord}, getProperties, &mass, &centerOfMassLocal, &inertiaTensor);

    // Momentum conservation: the stored linear velocity tracks the center of mass,
    // so as it shifts it takes on that new point's velocity, spinning at the
    // caller-fixed angular velocity. The origin stays put.
    glm::dvec3 newWorldCenterOfMass{rigidBody->getPosition() +
                                    rigidBody->getOrientationMatrix() * centerOfMassLocal};
    rigidBody->m_velocity += glm::cross(angularVelocityWorld,
                                        newWorldCenterOfMass - oldWorldCenterOfMass);

    rigidBody->setMassProperties(mass, centerOfMassLocal, inertiaTensor);

    // The whole point of anchoring the body frame at the lattice origin: cells may
    // come and go, but every offset stored against the origin stays valid.
    assert(rigidBody->getPosition() == originBefore &&
           "a mass change must never move the body origin");
}

// Convert world coordinates to grid-local coordinates
glm::dvec3 Grid::worldToGrid(const glm::dvec3& worldPos) const {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert world to grid coordinates: Rigid body not found");
    }
    
    return GridGeometry::worldToGrid(worldPos, rigidBody->getPosition(), rigidBody->getOrientation());
}

// Convert grid-local coordinates to world coordinates
glm::dvec3 Grid::gridToWorld(const glm::dvec3& gridPos) const {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert grid to world coordinates: Rigid body not found");
    }
    
    return GridGeometry::gridToWorld(gridPos, rigidBody->getPosition(), rigidBody->getOrientation());
}

void Grid::getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition, glm::dquat& outOrientation) const {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("Grid::getInterpolatedTransform: Rigid body not found");
    }
    
    rigidBody->getInterpolatedTransform(timeRemainder, outPosition, outOrientation);
}

RayIntersectionResult Grid::intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const {
    auto collider = m_colliderWeak.lock();
    if (!collider) {
        return RayIntersectionResult();
    }
    return collider->intersectRay(rayStart, rayEnd);
}

double Grid::getApproximateRadius() const {
    auto collider = m_colliderWeak.lock();
    if (collider && !collider->getCells().empty()) {
        // Use the collider's bounding box to estimate radius (AABB already updated by collision detector)
        
        glm::dvec3 bboxSize = collider->m_AABBMax - collider->m_AABBMin;
        // Use half the maximum dimension as approximate radius
        double maxDimension = glm::max(glm::max(bboxSize.x, bboxSize.y), bboxSize.z);
        return maxDimension * 0.5;
    }
    
    // Fallback to radius = 1.0
    return 1.0;
}


// Updated - Update mesh transform based on physics state
void Grid::updateGraphics(const glm::dvec3& cameraPos) {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        return;
    }
    
    glm::dvec3 angVelAxis = rigidBody->getAngularVelocityWorld();
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        // If angular velocity is effectively zero, use a safe default axis
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }

    uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();
    
    m_gridGraphics->updateGraphics(
        cameraPos,
        rigidBody->getWorldCenterOfMass(),
        rigidBody->getOrientation(),
        rigidBody->m_velocity,
        rigidBody->getAngularVelocityWorld(),
        rigidBody->getCenterOfMassLocal(),
        currentPhysicsTimeStep,
        getApproximateRadius()
    );
}

size_t Grid::computeHash() const {
    size_t hash = 0;

    // Hash rigid body state (most important)
    auto rigidBody = m_rigidBody.lock();
    if (rigidBody) {
        hash = Hash::combineHashes(hash, rigidBody->computeHash());
    }

    // Hash all cells via the combined registry (IVec3Hash keys → deterministic iteration order)
    for (const auto& [coord, cell] : m_cellRegistry) {
        hash = Hash::combineHashes(hash, cell->computeHash());
    }

    return hash;
}

PolyhedronProcessor::MeshData Grid::generateFilteredMeshData(const glm::ivec3& coord) {
    // 1: Get the normalized vertices at the coord
    auto it = m_cells.find(coord);
    if (it == m_cells.end()) {
        return PolyhedronProcessor::MeshData{}; // Empty mesh for non-existent cell
    }
    
    const StructuralBlock& block = it->second;
    std::vector<glm::dvec3> normalizedVertices(8);
    for (int i = 0; i < 8; ++i) {
        normalizedVertices[i] = glm::dvec3(block.m_localVertices[i]) / double(StructuralBlock::MAX_SIZE);
    }
    
    // 2 & 3: Iterate over all neighbor coords and create 2D hidden map
    std::array<std::array<bool, 8>, 6> vertexHiddenByNeighbor;
    for (auto& row : vertexHiddenByNeighbor) row.fill(false); // All vertices start as not hidden
    
    static const glm::ivec3 directions[6] = {
        {1, 0, 0}, {-1, 0, 0},   // +X, -X
        {0, 1, 0}, {0, -1, 0},   // +Y, -Y
        {0, 0, 1}, {0, 0, -1}    // +Z, -Z
    };
    
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        auto neighborIt = m_cells.find(neighborCoord);
        
        if (neighborIt == m_cells.end()) {
            continue; // No neighbor, vertices remain visible
        }
        
        const StructuralBlock& neighborBlock = neighborIt->second;
        std::vector<glm::dvec3> neighborVertices(8);
        for (int v = 0; v < 8; ++v) {
            neighborVertices[v] = glm::dvec3(neighborBlock.m_localVertices[v]) / double(StructuralBlock::MAX_SIZE);
        }
        
        // Call checkPolyhedronBorderIntersection
        auto hiddenFlags = PolyhedronProcessor::checkPolyhedronBorderIntersection(
            coord, normalizedVertices, neighborCoord, neighborVertices);
        
        // Store which vertices are hidden by this specific neighbor
        for (int v = 0; v < 8; ++v) {
            vertexHiddenByNeighbor[i][v] = hiddenFlags[v];
        }
    }
    
    // 4: Generate triangle indices using getTriangleIndices
    std::vector<glm::ivec3> verticesVec(block.m_localVertices.begin(), block.m_localVertices.end());
    auto allTriangleIndices = PolyhedronProcessor::getTriangleIndices(verticesVec, StructuralBlock::MAX_SIZE);
    
    // Get all triangles that are NOT completely hidden by any single neighbor
    std::vector<std::array<glm::dvec3, 3>> visibleTriangles;
    
    for (const auto& triangleIndices : allTriangleIndices) {
        bool isTriangleHidden = false;
        
        // Check if any single neighbor hides ALL three triangle vertices
        for (int neighborDir = 0; neighborDir < 6; ++neighborDir) {
            bool allVerticesHiddenByThisNeighbor = true;
            
            for (int triVertex = 0; triVertex < 3; ++triVertex) {
                int cubeVertexIndex = triangleIndices[triVertex];
                if (!vertexHiddenByNeighbor[neighborDir][cubeVertexIndex]) {
                    allVerticesHiddenByThisNeighbor = false;
                    break;
                }
            }
            
            if (allVerticesHiddenByThisNeighbor) {
                isTriangleHidden = true;
                break;
            }
        }
        
        if (!isTriangleHidden) {
            // Convert triangle indices back to actual triangle vertices
            std::array<glm::dvec3, 3> triangle = {
                normalizedVertices[triangleIndices[0]],
                normalizedVertices[triangleIndices[1]], 
                normalizedVertices[triangleIndices[2]]
            };
            visibleTriangles.push_back(triangle);
        }
    }
    
    // 5: Wrap the remaining triangles back into mesh data
    return PolyhedronProcessor::generateMeshData(visibleTriangles);
}

void Grid::scheduleMeshUpdateForCell(const glm::ivec3& coord) {
    // Add only this cell to pending updates
    m_pendingMeshUpdates.insert(coord);
    
    // Schedule job only if not already running
    if (m_meshUpdateJob.expired()) {
        m_meshUpdateJob = m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
            return processPendingMeshUpdates(endTime);
        }, JobPriorities::GRAPHICS_UPDATE);
        trackJob(m_meshUpdateJob);
    }
}

void Grid::scheduleMeshUpdatesForCellAndNeighbors(const glm::ivec3& coord) {
    // Add this cell and its 6 neighbors to pending updates
    m_pendingMeshUpdates.insert(coord);
    
    static const glm::ivec3 directions[6] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}
    };
    
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        if (hasCell(neighborCoord)) {
            m_pendingMeshUpdates.insert(neighborCoord);
        }
    }
    
    // Schedule job only if not already running
    if (m_meshUpdateJob.expired()) {
        m_meshUpdateJob = m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
            return processPendingMeshUpdates(endTime);
        }, JobPriorities::GRAPHICS_UPDATE);
        trackJob(m_meshUpdateJob);
    }
}

bool Grid::processPendingMeshUpdates(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    auto it = m_pendingMeshUpdates.begin();
    while (it != m_pendingMeshUpdates.end() && m_timeHandler->now() < endTime) {
        glm::ivec3 coord = *it;

        // Only structural blocks use the mesh system; thrusters use instance rendering
        auto cellIt = m_cells.find(coord);
        if (cellIt != m_cells.end()) {
            auto meshData = generateFilteredMeshData(coord);
            const auto& cell = cellIt->second;
            if (m_gridGraphics->hasGraphicsCell(coord)) {
                m_gridGraphics->updateCell(coord, meshData, cell.m_color);
            } else {
                m_gridGraphics->addCell(coord, StructuralBlock::TYPE, meshData, cell.m_color);
            }
        }

        it = m_pendingMeshUpdates.erase(it);
    }

    return !m_pendingMeshUpdates.empty(); // More work if set not empty
}