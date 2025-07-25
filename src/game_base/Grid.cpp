// Grid.cpp - Updated

#include <set>
#include "Grid.h"
#include <glm/gtc/matrix_transform.hpp>
#include "../physics/PhysicsEngine.h"
#include "MassInertiaCalculator.h"
#include "../debug/DebugGlobals.h"
#include "../physics/CubeCollider.h"
#include <limits>
#include <iostream>
#include "../debug/DebugRenderer.h"
#include "../game_base/JobPriorities.h"

void GridCell::forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const {
    for (int i = 0; i < 6; ++i) {
        if (neighbors[i]) {
            callback(neighbors[i]->coordinates);
        }
    }
}

// Updated - Constructor now initializes with physics and graphics references
Grid::Grid(PhysicsEngine* physics, GraphicsEngine* graphics, JobManager* jobManager,
           TimeHandler* timeHandler, const glm::dvec3& position, const glm::dquat& orientation) 
    : m_physics(physics), m_jobManager(jobManager), m_timeHandler(timeHandler)
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

    // Cancel any pending analysis job
    cancelStructuralAnalysis();
}

// Add a cell to the grid
void Grid::addCell(const glm::ivec3& coord, CellType type) {
    // If cell already exists, return
    if (hasCell(coord)) return;

    // Add cell to collider
    auto cubeCollider = std::make_unique<CubeCollider>(glm::dvec3(0.0), glm::dquat(1.0, 0.0, 0.0, 0.0), 1.0);
    m_collider->addCell(coord, std::move(cubeCollider));

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();
    
    // Add cell to map immediately
    m_cells.emplace(coord, GridCell{coord, this, type});

    // Schedule structural analysis
    scheduleStructuralAnalysis();

    // Add to graphics subsystem
    m_gridGraphics->addCell(coord, type);

    // Update neighbor connections after all other setup
    updateNeighborConnections(coord);
    
    // Recalculate center of mass (will automatically choose incremental vs full)
    recalculateMassAndInertiaIncremental({coord});
}

// Remove a cell from the grid
void Grid::removeCell(const glm::ivec3& coord) {
    // If cell doesn't exist, return
    if (!hasCell(coord)) return;

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();

    removeNeighborConnections(coord);
    // Recalculate mass and inertia incrementally before removing the cell
    recalculateMassAndInertiaIncremental({coord}, true);
    
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

GridCell* Grid::getCell(const glm::ivec3& coord) {
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
    
    GridCell* cell = getCell(coord);
    if (!cell) return;
    
    // Update this cell's neighbor pointers and update neighbors to point back
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        GridCell* neighbor = getCell(neighborCoord);
        
        cell->neighbors[i] = neighbor;
        
        // If neighbor exists, make it point back to this cell
        if (neighbor) {
            neighbor->neighbors[oppositeDir[i]] = cell;
        }
    }
}

void Grid::removeNeighborConnections(const glm::ivec3& coord) {
    GridCell* cell = getCell(coord);
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
            const GridCell& cell = pair.second;
            
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
        m_stochasticAnalyzer = std::make_unique<StochasticAnalyzer<GridCell>>(m_cells);
    }
    
    // Run analysis until time runs out or iteration completes
    bool needsMoreTime = m_stochasticAnalyzer->performAnalysisUntil(endTime, *m_timeHandler);
    //return false;
    if (!needsMoreTime) {
        // Current iteration completed, update running averages
        for (auto& pair : m_cells) {
            GridCell& cell = pair.second;
            double currentCost = static_cast<double>(cell.getCost());
            
            if (cell.structuralWeakness >= 0.0) {
                // Update running average: new = old * (1-f) + current * f
                cell.structuralWeakness = cell.structuralWeakness * (1.0 - WEAKNESS_BLEND_FACTOR) + 
                                        currentCost * WEAKNESS_BLEND_FACTOR;
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

void Grid::recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& cellCoords, bool isRemoval) {
    if (!m_rigidBody) return;
    
    // If we have no existing mass or no cells to process, do full recalculation
    if (cellCoords.empty() || m_rigidBody->m_mass < 1e-15) {
        recalculateMassAndInertia();
        return;
    }
    
    if (cellCoords.empty()) return;

    // Constants for block properties
    const double blockMass = 60.0;
    const double blockWidth = 1.0;
    const double blockBaseInertiaValue = (blockMass / 6.0) * blockWidth * blockWidth; // I = (m/6)*w² for cube
    const glm::dmat3 blockBaseInertiaTensor = glm::dmat3(blockBaseInertiaValue); // Identity * scalar
    
    glm::dvec3 oldCM = m_centerOfMass;
    
    // Calculate incremental update directly on rigid body properties
    MassInertiaCalculator::calculateTensorInertiaIncremental(
        cellCoords,
        [this, blockMass, blockBaseInertiaTensor, isRemoval](const glm::ivec3& coord) {
            bool exists = m_cells.find(coord) != m_cells.end();
            double massSign = isRemoval ? -1.0 : 1.0;
            glm::dmat3 inertiaSign = isRemoval ? glm::dmat3(-1.0) : glm::dmat3(1.0);
            return MassInertiaCalculator::TensorObjectData{glm::dvec3(coord) + glm::dvec3{0.5},
                                                    exists ? blockMass * massSign : 0.0, 
                                                    exists ? blockBaseInertiaTensor * inertiaSign : glm::dmat3(0.0)};
        },
        &m_rigidBody->m_mass, &m_centerOfMass, &m_rigidBody->m_inertiaTensor);

    // Update physics body with momentum conservation
    glm::dvec3 cmShift = m_centerOfMass - oldCM;
    m_rigidBody->m_position += m_rigidBody->m_orientation * cmShift;

    // Calculate angular velocity from momentum for velocity correction
    glm::dmat3 orientationMatrix = glm::mat3_cast(m_rigidBody->m_orientation);
    m_rigidBody->m_velocity += glm::cross(m_rigidBody->getAngularVelocityWorld(), m_rigidBody->m_orientation * cmShift);
    updateRigidBodyInverses();
}

void Grid::recalculateMassAndInertia() {
    if (m_cells.empty()) return;

    glm::dvec3 oldCM = m_centerOfMass;
    const double blockMass = 60.0;
    const double blockWidth = 1.0;
    const double blockBaseInertiaValue = (blockMass / 6.0) * blockWidth * blockWidth; // I = (m/6)*w² for cube
    const glm::dmat3 blockBaseInertiaTensor = glm::dmat3(blockBaseInertiaValue); // Identity * scalar
    
    MassInertiaCalculator::calculateTensorInertia(
        m_cells, 
        [=](const auto& pair) { return MassInertiaCalculator::TensorObjectData{glm::dvec3(pair.first) + glm::dvec3{0.5}, blockMass, blockBaseInertiaTensor}; },
        &m_rigidBody->m_mass, &m_centerOfMass, &m_rigidBody->m_inertiaTensor);
    glm::dvec3 change = m_centerOfMass - oldCM;
    if (glm::length(change) > 0.00001) {
        m_rigidBody->m_position += m_rigidBody->m_orientation * change;

        m_rigidBody->m_velocity += glm::cross(m_rigidBody->getAngularVelocityWorld(), m_rigidBody->m_orientation * change);
    }
    updateRigidBodyInverses();
    m_rigidBody->invalidateInertiaTensor();
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
