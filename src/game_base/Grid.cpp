// Grid.cpp - Updated

#include <set>
#include "Grid.h"
#include <glm/gtc/matrix_transform.hpp>
#include "../physics/PhysicsEngine.h"
#include "../utils/MassInertiaCalculator.h"
#include "../debug/DebugGlobals.h"
#include "../physics/CubeCollider.h"
#include <limits>
#include <iostream>
#include "../debug/DebugRenderer.h"
#include "../game_base/JobPriorities.h"
#include "../physics/PolyhedronCollider.h"

// Initialize static counter
uint64_t Grid::s_nextUniqueId = 0;

// Updated - Constructor now initializes with physics and graphics references
Grid::Grid(PhysicsEngine* physics, GraphicsEngine* graphics, JobManager* jobManager,
           TimeHandler* timeHandler, const glm::dvec3& position, const glm::dquat& orientation) 
    : uniqueId(s_nextUniqueId++), m_physics(physics), m_jobManager(jobManager), m_timeHandler(timeHandler)
{
    if (!m_jobManager || !m_timeHandler) {
        throw std::invalid_argument("JobManager and TimeHandler cannot be null");
    }

    // Create graphics subsystem
    m_gridGraphics = std::make_unique<GridGraphics>(graphics, jobManager);

    // Create grid collider for the grid
    m_collider = std::make_unique<GridCollider>(position, orientation, nullptr, jobManager, timeHandler);
    
    // Create rigid body in physics engine
    m_rigidBody = m_physics->addRigidBody(
        position,
        orientation,
        0.0,  // Mass
        glm::dmat3(0.0),  // Inertia tensor
        false, // Not static
        m_collider.get() // Pass the sphere collider
    );
    
    // Set initial collider offset and update transform
    if (m_rigidBody) {
        m_rigidBody->m_colliderOffset = m_centerOfMass;
    }
    m_physics->updateColliderTransform(m_rigidBody);

    // Initial graphics update
    // Use a default distant camera position for initial update
    updateGraphics(glm::dvec3(0.0, 0.0, 100.0));
}

Grid::~Grid() {
    // Clean up resources
    if (m_rigidBody) {
        m_physics->removeRigidBody(m_rigidBody);
    }

    // Cancel all pending jobs
    for (auto& jobHandle : m_pendingJobs) {
        if (!jobHandle.expired()) {
            m_jobManager->cancel(jobHandle);
        }
    }

    // Cancel any pending analysis job
    cancelStructuralAnalysis();
}

// Add a cell to the grid
void Grid::addCell(const glm::ivec3& coord) {
    // If cell already exists, return
    if (hasCell(coord)) return;

    // Add cell to collider - use PolyhedronCollider with optimized cube generation
    std::vector<glm::dvec3> vertices = CubeCollider::generateCubeVertices(1.0);
    std::vector<glm::dvec3> axes = CubeCollider::generateCubeAxes();
    
    auto cubeCollider = std::make_unique<PolyhedronCollider>(
        glm::dvec3(0.0),           // position
        glm::dquat(1.0, 0.0, 0.0, 0.0), // orientation  
        vertices,                  // local vertices
        axes,                      // face axes
        axes                       // edge axes (same as face for cubes)
    );
    m_collider->addCell(coord, std::move(cubeCollider));

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();
    
    // Create structural block and generate mesh data
    StructuralBlock block(coord);
    auto meshData = block.generateTriangleMeshData();
    
    // Move block into cells map
    m_cells.emplace(coord, std::move(block));

    // Schedule structural analysis
    scheduleStructuralAnalysis();

    // Add to graphics subsystem
    m_gridGraphics->addCell(coord, StructuralBlock::TYPE, meshData, block.m_color);

    // Update neighbor connections after all other setup
    updateNeighborConnections(coord);
    
    // Preserve angular velocity across mass change
    glm::dvec3 originalAngularVelocity = m_rigidBody->getAngularVelocityBody();
    
    // Add mass contribution
    updateCellMassContribution(coord, 1.0);
    
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);
}

// Remove a cell from the grid
void Grid::removeCell(const glm::ivec3& coord) {
    // If cell doesn't exist, return
    if (!hasCell(coord)) return;

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();

    removeNeighborConnections(coord);

    // Preserve angular velocity across mass change
    glm::dvec3 originalAngularVelocity = m_rigidBody->getAngularVelocityBody();
    
    // Remove mass contribution before removing the cell
    updateCellMassContribution(coord, -1.0);
    
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);
    
    // Remove from collider
    m_collider->removeCell(coord);
    
    // Remove from graphics subsystem
    m_gridGraphics->removeCell(coord);

    // Remove neighbor connections before removing from map
    removeNeighborConnections(coord);
    
    // Remove cell from map
    m_cells.erase(coord);

    // Schedule structural analysis
    scheduleStructuralAnalysis();
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
    glm::dvec3 originalAngularVelocity = m_rigidBody->getAngularVelocityBody();

    // Remove old mass contribution
    updateCellMassContribution(coord, -1.0);

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
    
    // Remove old cell from collider and add new one
    m_collider->removeCell(coord);
    
    auto newCollider = std::make_unique<PolyhedronCollider>(
        glm::dvec3(0.0),           // position
        glm::dquat(1.0, 0.0, 0.0, 0.0), // orientation  
        vertices,                  // local vertices
        axes.faceAxis,             // face axes
        axes.edgeAxis              // edge axes
    );
    m_collider->addCell(coord, std::move(newCollider));
    
    // Generate new mesh data and update graphics
    auto meshData = block.generateTriangleMeshData();
    m_gridGraphics->removeCell(coord);
    m_gridGraphics->addCell(coord, StructuralBlock::TYPE, meshData, block.m_color);

    // Add new mass contribution
    updateCellMassContribution(coord, 1.0);
     
    // Restore angular velocity after entire modification
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);
    
    // Schedule structural analysis
    scheduleStructuralAnalysis();
    
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
    
    // Update only graphics (remove and re-add with new color)
    auto meshData = block.generateTriangleMeshData();
    m_gridGraphics->removeCell(coord);
    m_gridGraphics->addCell(coord, StructuralBlock::TYPE, meshData, block.m_color);
    
    // No need to update physics/mass since only color changed
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
    
    StructuralBlock* cell = getCell(coord);
    if (!cell) return;
    
    // Update this cell's neighbor pointers and update neighbors to point back
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        StructuralBlock* neighbor = getCell(neighborCoord);
        
        cell->neighbors[i] = neighbor;
        
        // If neighbor exists, make it point back to this cell
        if (neighbor) {
            neighbor->neighbors[oppositeDir[i]] = cell;
        }
    }
}

void Grid::removeNeighborConnections(const glm::ivec3& coord) {
    StructuralBlock* cell = getCell(coord);
    if (!cell) return;
    
    // Remove this cell from all its neighbors' pointer arrays
    for (int i = 0; i < 6; ++i) {
        GridCell* neighbor = cell->neighbors[i];
        if (neighbor) {
            // Find this cell in neighbor's array and set to nullptr
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
    
    extern int hit_count;
    int hh = hit_count;
    
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

// Check if a cell exists at the given coordinates
bool Grid::hasCell(const glm::ivec3& coord) const {
    return m_cells.find(coord) != m_cells.end();
}

// Check if the grid is empty (has no cells)
bool Grid::isEmpty() const {
    return m_cells.empty();
}

void Grid::updateCellMassContribution(const glm::ivec3& coord, double sign) {
    if (!m_rigidBody) return;
    
    glm::dvec3 oldCM = m_centerOfMass;
    
    // Lambda to get cell properties with sign applied
    auto getProperties = [this, sign](const glm::ivec3& cellCoord) -> std::tuple<double, glm::dvec3, glm::dmat3> {
        auto cellIt = m_cells.find(cellCoord);
        if (cellIt != m_cells.end()) {
            auto [mass, localCOM, inertia] = cellIt->second.getMassProperties();
            return std::make_tuple(mass * sign, localCOM, inertia * sign);
        }
        return std::make_tuple(0.0, glm::dvec3(0.0), glm::dmat3(0.0));
    };
    
    // Update mass contribution using coordinate-based calculation
    MassInertiaCalculator::calculateInertiaForCoords(
        {coord}, getProperties,
        &m_rigidBody->m_mass, &m_centerOfMass, &m_rigidBody->m_inertiaTensor);
    
    // Update physics body with momentum conservation
    glm::dvec3 cmShift = m_centerOfMass - oldCM;
    m_rigidBody->m_position += m_rigidBody->m_orientation * cmShift;
    m_rigidBody->m_velocity += glm::cross(m_rigidBody->getAngularVelocityWorld(), m_rigidBody->m_orientation * cmShift);
    
    updateRigidBodyInverses();
}

void Grid::updateRigidBodyInverses() {
    if (!m_rigidBody) return;

    // Update inverse values with safety checks
    m_rigidBody->m_invMass = (m_rigidBody->m_mass > 1e-15) ? (1.0 / m_rigidBody->m_mass) : std::numeric_limits<double>::max();

    // Update inverse inertia tensor with safety check
    double determinant = glm::determinant(m_rigidBody->m_inertiaTensor);
    m_rigidBody->m_invInertiaTensor = (determinant > 1e-15) ? glm::inverse(m_rigidBody->m_inertiaTensor) : glm::dmat3(std::numeric_limits<double>::max());

    m_rigidBody->m_colliderOffset = m_centerOfMass;
}

// Convert world coordinates to grid-local coordinates
glm::dvec3 Grid::worldToGrid(const glm::dvec3& worldPos) const {
    if (!m_rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert world to grid coordinates: Rigid body not found");
    }
    
    return GridGeometry::worldToGrid(worldPos, m_rigidBody->m_position, m_rigidBody->m_orientation, m_centerOfMass);
}

// Convert grid-local coordinates to world coordinates
glm::dvec3 Grid::gridToWorld(const glm::dvec3& gridPos) const {
    if (!m_rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert grid to world coordinates: Rigid body not found");
    }
    
    return GridGeometry::gridToWorld(gridPos, m_rigidBody->m_position, m_rigidBody->m_orientation, m_centerOfMass);
}

// Add your gridTraversal implementation to Grid.cpp
std::vector<glm::ivec3> Grid::gridTraversal(glm::dvec3 startPos, glm::dvec3 endPos) {
   return GridGeometry::gridTraversal(startPos, endPos);
}

double Grid::getApproximateRadius() const {
    if (m_collider && !m_collider->getCells().empty()) {
        // Use the collider's bounding box to estimate radius (AABB already updated by collision detector)
        
        glm::dvec3 bboxSize = m_collider->m_AABBMax - m_collider->m_AABBMin;
        // Use half the maximum dimension as approximate radius
        double maxDimension = glm::max(glm::max(bboxSize.x, bboxSize.y), bboxSize.z);
        return maxDimension * 0.5;
    }
    
    // Fallback to radius = 1.0
    return 1.0;
}

int Grid::getGraphicsMeshId() const {
    // Delegate to graphics subsystem to get mesh ID
    return m_gridGraphics ? m_gridGraphics->getMeshId() : -1;
}

// Updated - Update mesh transform based on physics state
void Grid::updateGraphics(const glm::dvec3& cameraPos) {
    if (!m_rigidBody) {
        return;
    }
    
    RigidBody* body = m_rigidBody;

    glm::dvec3 angVelAxis = body->getAngularVelocityWorld();
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
        body->m_position,
        body->m_orientation,
        body->m_velocity,
        body->getAngularVelocityWorld(),
        m_centerOfMass,
        currentPhysicsTimeStep,
        getApproximateRadius()
    );
}

size_t Grid::computeHash() const {
    size_t hash = 0;
    
    // Hash rigid body state (most important)
    if (m_rigidBody) {
        hash = combineHashes(hash, m_rigidBody->computeHash());
    }
    
    // Hash center of mass
    hash = combineHashes(hash, DVec3Hash{}(m_centerOfMass));
    
    // Hash all cells in deterministic order
    std::vector<std::pair<glm::ivec3, const StructuralBlock*>> sortedCells;
    sortedCells.reserve(m_cells.size());
    for (const auto& pair : m_cells) {
        sortedCells.emplace_back(pair.first, &pair.second);
    }
    std::sort(sortedCells.begin(), sortedCells.end(), 
              [](const auto& a, const auto& b) { return a.second->uniqueId < b.second->uniqueId; });
    
    for (const auto& pair : sortedCells) {
        hash = combineHashes(hash, pair.second->computeHash());
    }
    
    return hash;
}