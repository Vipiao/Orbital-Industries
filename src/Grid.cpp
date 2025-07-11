// Grid.cpp - Updated

#include <set>
#include "Grid.h"
#include <glm/gtc/matrix_transform.hpp>
#include "MassInertiaCalculator.h"
#include "DebugGlobals.h"
#include <limits>
#include <iostream>
#include "DebugRenderer.h"

std::vector<glm::ivec3> GridCell::getConnectedNeighbors() const {
    std::vector<glm::ivec3> neighbors;
    
    // Check all 6 directions
    static const glm::ivec3 directions[6] = {
        {1, 0, 0},   // Right
        {-1, 0, 0},  // Left
        {0, 1, 0},   // Front
        {0, -1, 0},  // Back
        {0, 0, 1},   // Top
        {0, 0, -1}   // Bottom
    };
    
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coordinates + directions[i];
        if (parentGrid && parentGrid->hasCell(neighborCoord)) {
            neighbors.push_back(neighborCoord);
        }
    }
    
    return neighbors;
}

// Updated - Constructor now initializes with physics and graphics references
Grid::Grid(PhysicsEngine* physics, GraphicsEngine* graphics,
           const glm::dvec3& position, const glm::dquat& orientation) 
    : m_physics(physics)
{
    // Create graphics subsystem
    m_gridGraphics = std::make_unique<GridGraphics>(graphics);

    // Create grid collider for the grid
    m_collider = std::make_unique<GridCollider>(position, orientation);
    
    // Create rigid body in physics engine
    m_rigidBody = m_physics->addRigidBody(
        position,
        orientation,
        1.0,  // Mass
        0.4,  // Moment of inertia
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
}

// Add a cell to the grid
void Grid::addCell(const glm::ivec3& coord, CellType type) {
    // If cell already exists, return
    if (hasCell(coord)) return;

    // Add cell to collider
    m_collider->addCell(coord, 1.0);  // Use 1.0 width for cube cells
    
    // Add cell to map immediately
    m_cells.emplace(coord, GridCell{coord, this, type});

    // Add to graphics subsystem
    m_gridGraphics->addCell(coord, type);
    
    // Recalculate center of mass (will automatically choose incremental vs full)
    recalculateMassAndInertiaIncremental({coord});
}

// Remove a cell from the grid
void Grid::removeCell(const glm::ivec3& coord) {
    // If cell doesn't exist, return
    if (!hasCell(coord)) return;

    // Recalculate mass and inertia incrementally before removing the cell
    recalculateMassAndInertiaIncremental({coord}, true);
    
    // Remove from collider
    m_collider->removeCell(coord);
    
    // Remove from graphics subsystem
    m_gridGraphics->removeCell(coord);
    
    // Remove cell from map
    m_cells.erase(coord);
}

void Grid::analyzeStructuralIntegrity(TimeHandler* timeHandler) {
    if (!timeHandler) {
        std::cerr << "Error: TimeHandler is required for structural analysis" << std::endl;
        return;
    }
    
    // Clear existing cost-based debug spheres
    if (DebugGlobals::getDebugRenderer()) {
        DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("cost_cell_");
    }
    
    // Clear structural weakness from previous analysis
    for (auto& pair : m_cells) {
        pair.second.structuralWeakness = 0.0;
    }
    
    const int numIterations = 8; // Run analysis 5 times to reduce randomness
    
    std::cout << "Starting stochastic structural analysis with " << numIterations 
              << " iterations on " << m_cells.size() << " cells..." << std::endl;
    
    for (int iteration = 0; iteration < numIterations; ++iteration) {
        // Create new stochastic analyzer for this iteration
        m_stochasticAnalyzer = std::make_unique<StochasticAnalyzer<GridCell>>(m_cells);
        
        // Set time limit for analysis (e.g., 16ms per iteration)
        auto timeLimit = timeHandler->now() + std::chrono::milliseconds(160000);
        m_stochasticAnalyzer->setTimeLimit(timeLimit);
        
        // Run analysis to completion for this iteration
        auto analysisGenerator = m_stochasticAnalyzer->performAnalysis(*timeHandler);
        
        while (analysisGenerator) {
            analysisGenerator();
            ++analysisGenerator;
        }
        
        // Accumulate structural weakness from this iteration
        for (auto& pair : m_cells) {
            pair.second.structuralWeakness += pair.second.getCost();
        }
        
        std::cout << "Completed iteration " << (iteration + 1) << "/" << numIterations << std::endl;
    }
    
    // Update debug visualization
    if (DebugGlobals::getDebugRenderer()) {
        for (const auto& pair : m_cells) {
            const glm::ivec3& coord = pair.first;
            const GridCell& cell = pair.second;
            
            double averageWeakness = numIterations > 0 ? cell.structuralWeakness / numIterations : 0.0;
            if (averageWeakness > 1.0) {
                glm::dvec3 cellWorldPos = gridToWorld(glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5));
                std::string sphereName = "cost_cell_" + std::to_string(coord.x) + "_" + 
                                       std::to_string(coord.y) + "_" + std::to_string(coord.z);
                double radius = 0.0 + glm::pow((averageWeakness - 1.0) * 0.01, 1./3.);
                DebugGlobals::getDebugRenderer()->createSphere(sphereName, cellWorldPos, radius);
            }
        }
    }
    
    std::cout << "Stochastic structural analysis complete! Averaged results from " 
              << numIterations << " iterations on " << m_cells.size() << " cells." << std::endl;
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
    const double blockBaseInertia = (2.0/3.0) * blockMass * 0.5 * 0.5 * 0.5;
    
    glm::dvec3 oldCM = m_centerOfMass;
    
    // Calculate incremental update directly on rigid body properties
    MassInertiaCalculator::calculateScalarInertiaIncremental(
        cellCoords,
        [this, blockMass, blockBaseInertia, isRemoval](const glm::ivec3& coord) { 
            bool exists = m_cells.find(coord) != m_cells.end();
            double massSign = isRemoval ? -1.0 : 1.0;
            double inertiaSign = isRemoval ? -1.0 : 1.0;
            return MassInertiaCalculator::ObjectData{glm::dvec3(coord) + glm::dvec3{0.5}, 
                                                    exists ? blockMass * massSign : 0.0, 
                                                    exists ? blockBaseInertia * inertiaSign : 0.0};
        },
        &m_rigidBody->m_mass, &m_centerOfMass, &m_rigidBody->m_momentOfInertia);

    // Update physics body with momentum conservation
    glm::dvec3 cmShift = m_centerOfMass - oldCM;
    m_rigidBody->m_position += m_rigidBody->m_orientation * cmShift;
    m_rigidBody->m_velocity += glm::cross(m_rigidBody->m_angularVelocity, m_rigidBody->m_orientation * cmShift);
    updateRigidBodyInverses();
}

void Grid::recalculateMassAndInertia() {
    if (m_cells.empty()) return;

    glm::dvec3 oldCM = m_centerOfMass;
    const double blockMass = 60.0;
    const double blockBaseInertia = (2.0/3.0) * blockMass * 0.5 * 0.5 * 0.5;
    
    MassInertiaCalculator::calculateScalarInertia(
        m_cells, 
        [=](const auto& pair) { return MassInertiaCalculator::ObjectData{glm::dvec3(pair.first) + glm::dvec3{0.5}, blockMass, blockBaseInertia}; },
        &m_rigidBody->m_mass, &m_centerOfMass, &m_rigidBody->m_momentOfInertia);
    glm::dvec3 change = m_centerOfMass - oldCM;
    if (glm::length(change) > 0.00001) {
        m_rigidBody->m_position += m_rigidBody->m_orientation * change;
        m_rigidBody->m_velocity += glm::cross(m_rigidBody->m_angularVelocity, m_rigidBody->m_orientation * change);
    }
    updateRigidBodyInverses();
}

void Grid::updateRigidBodyInverses() {
    if (!m_rigidBody) return;

    // Update inverse values with safety checks
    m_rigidBody->m_invMass = (m_rigidBody->m_mass > 1e-15) ? (1.0 / m_rigidBody->m_mass) : std::numeric_limits<double>::max();
    m_rigidBody->m_invMomentOfInertia = (m_rigidBody->m_momentOfInertia > 1e-15) ? (1.0 / m_rigidBody->m_momentOfInertia) : std::numeric_limits<double>::max();

    m_rigidBody->m_colliderOffset = m_centerOfMass;
}

// Process all queued graphics updates.
void Grid::processGraphicsQueue() {
    m_gridGraphics->processGraphicsQueue();
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
        // Use the collider's bounding box to estimate radius
        // Force an AABB update to get current bounds
        m_collider->updateTransformAndAABB();
        
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
    
    PhysicsEngine::RigidBody* body = m_rigidBody;

    glm::dvec3 angVelAxis = body->m_angularVelocity;
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
        body->m_angularVelocity,
        m_centerOfMass,
        currentPhysicsTimeStep,
        getApproximateRadius()
    );
}

bool Grid::hasGraphicsUpdates() const {
    return m_gridGraphics->hasGraphicsUpdates();
}