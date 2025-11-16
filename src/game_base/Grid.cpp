// Grid.cpp

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
#include "../utils/GridGeometry.h"
#include "../utils/PolyhedronProcessor.h"
#include "../utils/GeometryUtils.h"

// Initialize static counter
uint64_t Grid::s_nextUniqueId = 0;

// Updated - Constructor now initializes with physics and graphics references
Grid::Grid(PhysicsEngine* physics, GraphicsEngine* graphics, JobManager* jobManager,
           TimeHandler* timeHandler, const glm::dvec3& position, const glm::dquat& orientation) 
    : uniqueId(s_nextUniqueId++), m_jobManager(jobManager), m_timeHandler(timeHandler), m_physics(physics)
{
    if (!m_jobManager || !m_timeHandler) {
        throw std::invalid_argument("JobManager and TimeHandler cannot be null");
    }

    // Create grid collider via CollisionDetector factory
    m_colliderWeak = m_physics->getCollisionDetector().addGridCollider(
        position,
        orientation,
        jobManager,
        timeHandler);

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
    
    // Attach collider to rigid body
    // localPosition is negative of center of mass (collider position relative to COM)
    m_physics->attachCollider(
        m_rigidBody, 
        m_colliderWeak,
        -m_centerOfMass,                             // localPosition (collider relative to COM)
        glm::dquat(1.0, 0.0, 0.0, 0.0),             // localOrientation (identity)
        false);                                      // isTrigger = false

    // Create graphics subsystem
    m_gridGraphics = std::make_unique<GridGraphics>(graphics, jobManager);

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
    if (m_rigidBody) {
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
    
    // Move block into cells map
    m_cells.emplace(coord, std::move(block));

    // Schedule structural analysis
    scheduleStructuralAnalysis();

    // Update neighbor connections after all other setup
    updateNeighborConnections(coord);
    
    // Preserve angular velocity across mass change
    glm::dvec3 originalAngularVelocity = m_rigidBody->getAngularVelocityBody();
    
    // Add mass contribution
    updateCellMassContribution(coord, 1.0);
    
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);

    // Schedule mesh updates for this cell and neighbors
    scheduleMeshUpdatesForCellAndNeighbors(coord);
}

// Remove a cell from the grid
void Grid::removeCell(const glm::ivec3& coord) {
    // If cell doesn't exist, return
    if (!hasCell(coord)) return;

    // Cancel existing analysis to prevent accessing deleted cells
    cancelStructuralAnalysis();

    // Schedule mesh updates BEFORE removing (while neighbors still exist)
    scheduleMeshUpdatesForCellAndNeighbors(coord);

    removeNeighborConnections(coord);

    // Preserve angular velocity across mass change
    glm::dvec3 originalAngularVelocity = m_rigidBody->getAngularVelocityBody();
    
    // Remove mass contribution before removing the cell
    updateCellMassContribution(coord, -1.0);
    
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);
    
    // Remove from collider
    auto collider = m_colliderWeak.lock();
    if (collider) {
        collider->removeCell(coord);
    }
    
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
    updateCellMassContribution(coord, 1.0);
     
    // Restore angular velocity after entire modification
    m_rigidBody->setAngularVelocityBody(originalAngularVelocity);

    // Update neighbor connections since geometry changed
    updateNeighborConnections(coord);
    
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
    
    // Schedule mesh update for color change
    scheduleMeshUpdateForCell(coord);
    
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

    // Get this cell's vertices in grid space
    std::vector<glm::dvec3> cellVertices = cell->getVertices();
    for (auto& vertex : cellVertices) {
        vertex += glm::dvec3(coord);  // Transform to grid coordinates
    }
    
    // Update this cell's neighbor pointers and update neighbors to point back
    for (int i = 0; i < 6; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        StructuralBlock* neighbor = getCell(neighborCoord);
        
        bool hasConnection = false;
        
        if (neighbor) {
            // Get neighbor's vertices in grid space
            std::vector<glm::dvec3> neighborVertices = neighbor->getVertices();
            for (auto& vertex : neighborVertices) {
                vertex += glm::dvec3(neighborCoord);  // Transform to grid coordinates
            }
            
            // Use direction as face normal
            glm::dvec3 faceNormal = glm::dvec3(directions[i]);
            
            // Calculate overlap area
            double overlapArea = GeometryUtils::calculateSurfaceOverlapArea(
                cellVertices, neighborVertices, faceNormal);
            
            // Set connection if area exceeds threshold
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

    // Update attachment local position to match new center of mass
    if (!m_rigidBody->m_attachments.empty()) {
        // Find the grid collider attachment and update its local position
        for (auto& attachment : m_rigidBody->m_attachments) {
            auto collider = attachment->collider.lock();
            if (collider && collider.get() == m_colliderWeak.lock().get()) {
                attachment->localPosition = -m_centerOfMass;
                break;
            }
        }
    }
}

void Grid::updateRigidBodyInverses() {
    if (!m_rigidBody) return;

    // Update inverse values with safety checks
    m_rigidBody->m_invMass = (m_rigidBody->m_mass > 1e-15) ? (1.0 / m_rigidBody->m_mass) : std::numeric_limits<double>::max();

    // Update inverse inertia tensor with safety check
    double determinant = glm::determinant(m_rigidBody->m_inertiaTensor);
    m_rigidBody->m_invInertiaTensor = (determinant > 1e-15) ? glm::inverse(m_rigidBody->m_inertiaTensor) : glm::dmat3(std::numeric_limits<double>::max());
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

void Grid::getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition, glm::dquat& outOrientation) const {
    if (!m_rigidBody) {
        throw std::runtime_error("Grid::getInterpolatedTransform: Rigid body not found");
    }
    
    m_rigidBody->getInterpolatedTransform(timeRemainder, outPosition, outOrientation);
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
        hash = Hash::combineHashes(hash, m_rigidBody->computeHash());
    }
    
    // Hash center of mass
    hash = Hash::combineHashes(hash, Hash::DVec3Hash{}(m_centerOfMass));
    
    // Hash all cells in deterministic order
    std::vector<std::pair<glm::ivec3, const StructuralBlock*>> sortedCells;
    sortedCells.reserve(m_cells.size());
    for (const auto& pair : m_cells) {
        sortedCells.emplace_back(pair.first, &pair.second);
    }
    std::sort(sortedCells.begin(), sortedCells.end(), 
              [](const auto& a, const auto& b) { return a.second->uniqueId < b.second->uniqueId; });
    
    for (const auto& pair : sortedCells) {
        hash = Hash::combineHashes(hash, pair.second->computeHash());
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
        
        if (hasCell(coord)) {
            // Generate filtered mesh and update graphics
            auto meshData = generateFilteredMeshData(coord);
            const auto& cell = m_cells.at(coord);
            // Check if graphics cell exists - if not, add it; otherwise update it
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