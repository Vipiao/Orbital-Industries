// Grid.cpp - Updated

#include <set>
#include "Grid.h"
#include <glm/gtc/matrix_transform.hpp>
#include "MassInertiaCalculator.h"
#include "DebugGlobals.h"
#include "StochasticAnalyzer.h"
#include <limits>
#include <iostream>
#include "DebugGlobals.h"
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

// Comparator for std::set to work with glm::ivec3
struct IVec3Comparator {
    bool operator()(const glm::ivec3& a, const glm::ivec3& b) const {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    }
};

// Static member initialization
bool Grid::s_faceMeshDataLoaded = false;
std::vector<AssetMeshData> Grid::s_faceMeshData;

int Grid::s_colorTextureUnit = -1;
int Grid::s_normalTextureUnit = -1;
bool Grid::s_texturesLoaded = false;

// Face transform lookup table initialization
const Grid::FaceTransform Grid::s_faceTransforms[6] = {
    // Right (+X)
    {glm::dvec3(0, 1, 0), glm::radians(90.0)},
    // Left (-X)
    {glm::dvec3(0, 1, 0), glm::radians(-90.0)},
    // Front (+Y) 
    {glm::dvec3(1, 0, 0), glm::radians(-90.0)},
    // Back (-Y)
    {glm::dvec3(1, 0, 0), glm::radians(90.0)},
    // Up (+Z) - No rotation needed
    {glm::dvec3(1, 0, 0), 0.0},
    // Down (-Z)
    {glm::dvec3(1, 0, 0), glm::radians(180.0)}
};

// Face mesh loading function
void Grid::loadFaceMeshData() {
    if (s_faceMeshDataLoaded) return;
    
    // Load the face mesh using AssimpLoader - a face in positive Z direction
    AssimpLoader::load("../media/blender/01_face.obj", &s_faceMeshData);
    s_faceMeshDataLoaded = true;
}

// Updated - Constructor now initializes with physics and graphics references
Grid::Grid(PhysicsEngine* physics, GraphicsEngine* graphics, 
           const glm::dvec3& position, const glm::dquat& orientation) 
    : m_physics(physics), m_graphics(graphics)
{
    // Ensure face mesh data is loaded
    loadFaceMeshData();
    
    // Create mesh in graphics engine
    m_meshId = m_graphics->createMesh();

    // Initialize next update time to 0 to force initial update
    m_nextUpdatePhysicsTimeStep = 0;
    m_lastCheckedPhysicsTimeStep = 0;

    // Load textures if needed
    if (!s_texturesLoaded && !s_faceMeshData.empty()) {
        try {
            MeshHandler::Texture colorTexture = m_graphics->createTexture("../media/color_512x512_occluded.png");
            s_colorTextureUnit = colorTexture.m_textureUnit;
            
            MeshHandler::Texture normalTexture = m_graphics->createTexture("../media/normal_combined_512x512.png");
            s_normalTextureUnit = normalTexture.m_textureUnit;
            
            s_texturesLoaded = true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to load textures: " << e.what() << std::endl;
        }
    }

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
    updateGraphics();
}

Grid::~Grid() {
    // Clean up resources
    if (m_meshId >= 0) {
        m_graphics->removeMesh(m_meshId);
    }
    
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
    
    // Queue this cell for graphics update
    m_graphicsUpdateQueue.push(coord);
    
    // Queue neighbors for potential graphics updates
    queueNeighborsForUpdate(coord);
    
    // Recalculate center of mass (will automatically choose incremental vs full)
    recalculateMassAndInertiaIncremental({coord});
}

// Remove a cell from the grid
void Grid::removeCell(const glm::ivec3& coord) {
    // If cell doesn't exist, return
    if (!hasCell(coord)) return;

    // Recalculate mass and inertia incrementally before removing the cell
    recalculateMassAndInertiaIncremental({coord}, true);
    
    // Get the cell before removal
    auto cellIt = m_cells.find(coord);
    if (cellIt != m_cells.end()) {

        // Remove cell from collider
        m_collider->removeCell(coord);

        // Remove all face triangles from the mesh
        for (int face = 0; face < 6; face++) {
            if (!cellIt->second.faceTriangleIds[face].empty()) {
                m_graphics->m_meshHandler->removeTrianglesFromMesh(
                    m_meshId, &cellIt->second.faceTriangleIds[face]);
            }
        }
    }
    
    // Queue neighbors for potential graphics updates before removing the cell
    queueNeighborsForUpdate(coord);
    
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
                double radius = 0.1 + (averageWeakness - 1.0) * 0.1;
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

// Queue neighboring cells for graphics update
void Grid::queueNeighborsForUpdate(const glm::ivec3& coord) {
    // Define the 6 neighbor directions (±X, ±Y, ±Z)
    static const glm::ivec3 neighbors[6] = {
        {1, 0, 0}, {-1, 0, 0}, 
        {0, 1, 0}, {0, -1, 0}, 
        {0, 0, 1}, {0, 0, -1}
    };
    
    // Add each neighbor to the update queue
    for (int i = 0; i < 6; i++) {
        glm::ivec3 neighborCoord = coord + neighbors[i];
        
        // If the neighbor exists, queue it for update
        if (hasCell(neighborCoord)) {
            m_graphicsUpdateQueue.push(neighborCoord);
        }
    }
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

// Check if a face is visible (not covered by another cell)
bool Grid::isFaceVisible(const glm::ivec3& coord, int faceIndex) const {
    // Define the 6 neighbor directions (±X, ±Y, ±Z)
    static const glm::ivec3 neighbors[6] = {
        {1, 0, 0}, {-1, 0, 0},  // Right, Left (X)
        {0, 1, 0}, {0, -1, 0},  // Front, Back (Y)
        {0, 0, 1}, {0, 0, -1}   // Up, Down (Z)
    };
    
    // Check the neighbor in the direction of the face
    glm::ivec3 neighborCoord = coord + neighbors[faceIndex];
    
    // Face is visible if neighbor is empty
    return !hasCell(neighborCoord);
}

// Get transformation matrix for a face
glm::dmat4 Grid::getFaceTransform(int faceIndex, const glm::ivec3& coord) {
    glm::dmat4 transform = glm::dmat4(1.0);
    
    // Position at grid coordinate
    transform = glm::translate(transform, glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5));
    
    // Apply rotation based on face lookup table
    const FaceTransform& faceTransform = s_faceTransforms[faceIndex];
    if (faceTransform.angle != 0.0) {
        transform = glm::rotate(transform, faceTransform.angle, faceTransform.axis);
    }
    
    // Some faces need additional rotation to align properly
    // This depends on the specific face.obj orientation
    if (faceIndex == 0 || faceIndex == 1) { // Right/Left faces
        //transform = glm::rotate(transform, glm::radians(90.0), glm::dvec3(1, 0, 0));
    }

    transform = glm::scale(transform, glm::dvec3{0.5, 0.5, 0.5});
    
    return transform;
}

// Process all queued graphics updates.
void Grid::processGraphicsQueue() {
    // Process the graphics update queue
    std::set<glm::ivec3, IVec3Comparator> processed; // To avoid processing the same cell multiple times
    
    while (!m_graphicsUpdateQueue.empty()) {
        glm::ivec3 coord = m_graphicsUpdateQueue.front();
        m_graphicsUpdateQueue.pop();
        
        // Skip if already processed
        if (processed.find(coord) != processed.end()) {
            continue;
        }
        processed.insert(coord);
        
        // Only update if the cell exists
        if (hasCell(coord)) {
            updateCellGraphics(coord);
        }
    }
}

// Convert world coordinates to grid-local coordinates
glm::dvec3 Grid::worldToGrid(const glm::dvec3& worldPos) const {
    if (!m_rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert world to grid coordinates: Rigid body not found");
    }
    
    // Transform: 
    // 1. Translate relative to body position
    // 2. Rotate by conjugate of body orientation
    // 3. Add center of mass offset
    return glm::conjugate(m_rigidBody->m_orientation) * (worldPos - m_rigidBody->m_position) + m_centerOfMass;
}

// Convert grid-local coordinates to world coordinates
glm::dvec3 Grid::gridToWorld(const glm::dvec3& gridPos) const {
    if (!m_rigidBody) {
        throw std::runtime_error("ERROR: Failed to convert grid to world coordinates: Rigid body not found");
    }
    
    // Transform:
    // 1. Subtract center of mass
    // 2. Apply body orientation
    // 3. Add body position
    return m_rigidBody->m_position + m_rigidBody->m_orientation * (gridPos - m_centerOfMass);
}

// Add your gridTraversal implementation to Grid.cpp
std::vector<glm::ivec3> Grid::gridTraversal(glm::dvec3 startPos, glm::dvec3 endPos) {
   // In case the direction is such that the end cell might be missed.
   // For example {-0.5, 0.5, 0.5}, {0.0, 0.0, 0.5}
   glm::ivec3 newOrigin = glm::floor(endPos);
   glm::dvec3 startPosRel{ startPos - static_cast<glm::dvec3>(newOrigin) };
   glm::dvec3 endPosRel{ endPos - static_cast<glm::dvec3>(newOrigin) };
   constexpr double shift{ 1.e-6 };
   if (endPosRel.x == 0.) {
      endPosRel.x = shift;
   }
   if (endPosRel.y == 0.) {
      endPosRel.y = shift;
   }
   if (endPosRel.z == 0.) {
      endPosRel.z = shift;
   }
   //
   std::vector<glm::ivec3> cells;
   glm::dvec3 dir{ endPosRel - startPosRel };
   glm::ivec3 step{ glm::sign(dir) };
   glm::dvec3 nextBoundary{ glm::floor(startPosRel) + glm::dvec3{ step } };
   glm::dvec3 tMax{};
   glm::dvec3 tDelta{};// = glm::abs(step / dir);
   for (int i = 0; i < 3; ++i) {
      // Avoid division by zero for axis-aligned rays
      if (dir[i] > 0.) {
         tMax[i] = (nextBoundary[i] - startPosRel[i]) / dir[i];
         tDelta[i] = glm::abs(step[i] / dir[i]);
      } else if (dir[i] < 0.) {
         tMax[i] = (nextBoundary[i] - startPosRel[i] + 1) / dir[i];
         tDelta[i] = glm::abs(step[i] / dir[i]);
      } else {
         tMax[i] = std::numeric_limits<double>::infinity();
         tDelta[i] = std::numeric_limits<double>::infinity();
      }
   }
   glm::ivec3 cell = glm::floor(startPosRel);
   //glm::dvec3 adjustedEndPos = endPosRel + 1e-10 * glm::normalize(dir);
   glm::ivec3 endCell = glm::floor(endPosRel);
#ifndef NDEBUG // Debug mode.
   uint64_t maxIt{ static_cast<uint64_t>(glm::length(dir) + 1.) * 3 };
   uint64_t iteration{ 0 };
#endif
   while (true) {
#ifndef NDEBUG // Debug mode.
      if (iteration++ > maxIt) {
         throw std::runtime_error("gridTraversal too many iterations.");
      }
#endif
      cells.push_back(cell + newOrigin);
      if (cell == endCell) {
         break;
      }
      if (tMax.x < tMax.y) {
         if (tMax.x < tMax.z) {
            cell.x += step.x;
            tMax.x += tDelta.x;
         } else {
            cell.z += step.z;
            tMax.z += tDelta.z;
         }
      } else {
         if (tMax.y < tMax.z) {
            cell.y += step.y;
            tMax.y += tDelta.y;
         } else {
            cell.z += step.z;
            tMax.z += tDelta.z;
         }
      }
   }
   return cells;
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

bool Grid::shouldUpdateGPU() const {
    uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();
    
    // Check if physics time step has incremented since last check
    if (currentPhysicsTimeStep > m_lastCheckedPhysicsTimeStep) {
        // Advance our cached state by the number of physics steps that have passed
        uint64_t stepsDelta = currentPhysicsTimeStep - m_lastCheckedPhysicsTimeStep;
        
        // Update cached position with velocity
        m_lastSentRigidBodyPosition += m_lastSentRigidBodyVelocity * static_cast<double>(stepsDelta);
        
        // Update cached orientation with angular velocity quaternion
        for (uint64_t i = 0; i < stepsDelta; ++i) {
            m_lastSentRigidBodyOrientation = m_lastSentRigidBodyAngularVelocityQuat * m_lastSentRigidBodyOrientation;
        }
        m_lastSentRigidBodyOrientation = glm::normalize(m_lastSentRigidBodyOrientation);
        
        m_lastCheckedPhysicsTimeStep = currentPhysicsTimeStep;
    }
    
    if (!m_rigidBody) {
        return false;
    }
    
    PhysicsEngine::RigidBody* body = m_rigidBody;
    
    // Check if it's time for a scheduled update
    if (currentPhysicsTimeStep >= m_nextUpdatePhysicsTimeStep) {
        return true;
    }
    
    // Check position difference (now using interpolated cached position)
    double positionDiff = glm::length(body->m_position - m_lastSentRigidBodyPosition);
    if (positionDiff > POSITION_THRESHOLD) {
        return true;
    }
    
    // Check orientation difference using radius-based threshold (now using interpolated cached orientation)
    double orientationDot = glm::abs(glm::dot(body->m_orientation, m_lastSentRigidBodyOrientation));
    // Clamp to handle numerical precision issues
    orientationDot = glm::clamp(orientationDot, 0.0, 1.0);
    double halfAngleDiff = glm::acos(orientationDot);
    double angleDiff = 2.0 * halfAngleDiff;
    
    // Calculate radius-based orientation threshold
    double radius = getApproximateRadius();
    double orientationThreshold = ORIENTATION_THRESHOLD_BASE / radius;
    
    if (angleDiff > orientationThreshold) {
        return true;
    }
    
    // No significant change detected
    return false;
}

// Update the graphics for a specific cell
void Grid::updateCellGraphics(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it == m_cells.end()) return;
    
    GridCell& cell = it->second;
    
    // First, remove any existing face meshes for this cell
    for (int face = 0; face < 6; face++) {
        if (!cell.faceTriangleIds[face].empty()) {
            m_graphics->m_meshHandler->removeTrianglesFromMesh(m_meshId, &cell.faceTriangleIds[face]);
            cell.faceTriangleIds[face].clear();
        }
    }
    
    // Then add faces that should be visible
    for (int face = 0; face < 6; face++) {
        if (isFaceVisible(coord, face)) {
            // Get transformation for this face
            glm::dmat4 transform = getFaceTransform(face, coord);
            
            // Apply transformation to face mesh data
            std::vector<glm::dvec3> positions;
            std::vector<glm::dvec3> normals;
            std::vector<glm::dvec3> tangents;
            std::vector<glm::dvec2> uvs;
            
            // Get base face data (pointing in +Z direction)
            const AssetMeshData& faceMesh = s_faceMeshData[0];
            
            // Transform vertices according to face direction
            for (size_t i = 0; i < faceMesh.indices.size(); i++) {
                int idx = faceMesh.indices[i];
                
                // Position
                const auto& pos = faceMesh.positionsData[idx];
                glm::dvec4 transformedPos = transform * glm::dvec4(pos[0], pos[1], pos[2], 1.0);
                positions.push_back(glm::dvec3(transformedPos));
                
                // Normal 
                const auto& norm = faceMesh.normalsData[idx];
                glm::dvec4 transformedNorm = transform * glm::dvec4(norm[0], norm[1], norm[2], 0.0);
                normals.push_back(glm::normalize(glm::dvec3(transformedNorm)));
                
                // Tangent
                const auto& tang = faceMesh.tangentsData[idx];
                glm::dvec4 transformedTang = transform * glm::dvec4(tang[0], tang[1], tang[2], 0.0);
                tangents.push_back(glm::normalize(glm::dvec3(transformedTang)));
                
                // UV coordinates (don't need transformation)
                const auto& texUV = faceMesh.uvsData[idx];
                uvs.push_back(glm::dvec2(texUV[0], texUV[1]));
            }
            
            // Add this face to the mesh
            cell.faceTriangleIds[face] = m_graphics->m_meshHandler->appendTrianglesToMesh(
                m_meshId, &positions, &normals, &tangents, &uvs);
        }
    }
}

// Updated - Update mesh transform based on physics state
void Grid::updateGraphics() {
    if (m_meshId < 0 || !m_rigidBody) {
        return;
    }

    // Only update GPU if there's a significant change
    if (!shouldUpdateGPU()) {
        return;
    } else {
        std::cout << "Updated graphics for " << this->m_cells.size() << std::endl;
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
    
    m_graphics->updateMeshTransform(
        m_meshId,
        body->m_position - m_centerOfMass,
        body->m_velocity,
        body->m_orientation,
        angVelAxis,
        angVelMagnitude,
        m_centerOfMass,                  // Updated - Set center of rotation to rigid body position
        glm::dvec3(1.0, 1.0, 1.0),      // Default scale
        s_colorTextureUnit,
        s_normalTextureUnit,
        currentPhysicsTimeStep
    );

    // Update tracking variables with current rigid body state
    m_lastSentRigidBodyPosition = body->m_position;
    m_lastSentRigidBodyOrientation = body->m_orientation;
    m_lastSentRigidBodyVelocity = body->m_velocity;
    
    // Convert angular velocity to quaternion for one physics step
    if (angVelMagnitude > 0.00001) {
        m_lastSentRigidBodyAngularVelocityQuat = glm::angleAxis(angVelMagnitude, angVelAxis);
    } else {
        m_lastSentRigidBodyAngularVelocityQuat = glm::dquat(1.0, 0.0, 0.0, 0.0); // Identity quaternion (no rotation)
    }
    
    // Update the last checked time step to current
    m_lastCheckedPhysicsTimeStep = currentPhysicsTimeStep;
    
    // Schedule next mandatory update
    m_nextUpdatePhysicsTimeStep = currentPhysicsTimeStep + TIME_THRESHOLD;
}