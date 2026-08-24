// GridCollider.cpp
#include "GridCollider.h"
#include <cassert>
#include "CubeCollider.h"
#include "utils/PolyhedronProcessor.h"
#include "PolyhedronCollider.h"
#include "utils/JobManager.h"
#include "utils/TimeHandler.h"
#include <glm/gtx/transform.hpp>
#include <algorithm>
#include "utils/PolyhedronProcessor.h"
#include "utils/GeometryUtils.h"
#include "utils/GridGeometry.h"

GridCollider::GridCollider(const glm::dvec3& position,
                          const glm::dquat& orientation,
                          JobManager* jobManager,
                          TimeHandler* timeHandler,
                          int classificationJobPriority)
    : Collider(position, orientation)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
    , m_classificationJobPriority(classificationJobPriority)
{
    if (!m_jobManager || !m_timeHandler) {
        throw std::invalid_argument("GridCollider requires JobManager and TimeHandler");
    }
}

void GridCollider::addCubeCell(const glm::ivec3& coord, double width) {
    std::vector<glm::dvec3> vertices = PolyhedronProcessor::generateCubeVertices(width);
    std::vector<glm::dvec3> axes = PolyhedronProcessor::generateCubeAxes();
    std::vector<std::array<int, 2>> edges = PolyhedronProcessor::generateCubeEdges();
    
    auto collider = std::make_unique<PolyhedronCollider>(
        glm::dvec3(0.0),
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        vertices,
        axes,
        axes, // For cubes, face axes and edge axes are the same
        edges); // Standard cube edge topology
    
    addCell(coord, std::move(collider));
}

void GridCollider::addPolyhedronCell(const glm::ivec3& coord,
                                     const std::vector<glm::dvec3>& localVertices,
                                     const std::vector<glm::dvec3>& localFaceAxes,
                                     const std::vector<glm::dvec3>& localEdgeAxes,
                                     const std::vector<std::array<int, 2>>& localEdges) {
    auto collider = std::make_unique<PolyhedronCollider>(
        glm::dvec3(0.0),
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        localVertices,
        localFaceAxes,
        localEdgeAxes,
        localEdges);
    
    addCell(coord, std::move(collider));
}

GridCollider::~GridCollider() {
    if (!m_classificationJob.expired()) {
        m_jobManager->cancel(m_classificationJob);
    }
}

void GridCollider::updateSimpleAABB(uint64_t currentTimestep) {
    // Check if simple AABB is still valid
    if (currentTimestep <= m_simpleAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    if (m_cells.empty()) {
        // If no cells, set a minimal AABB
        m_AABBMin = m_position - glm::dvec3(0.1);
        m_AABBMax = m_position + glm::dvec3(0.1);
        m_simpleAABBValidUntilTime = currentTimestep;
        return;
    }

    // TODO: Currently the simple and advanced both use and advanced calculation.
    // Simple could be done much cheaper calculating a bigger cube shaped
    // Collider centered around the position.

    // Update cached corners if needed
    if (currentTimestep > m_cornersValidUntilTime) {
        updateLocalCorners();
        m_cornersValidUntilTime = std::numeric_limits<uint64_t>::max();
    }
    
    // Transform first corner to initialize world AABB
    glm::dvec3 worldCorner = m_position + m_orientation * m_localCorners[0];
    m_AABBMin = worldCorner;
    m_AABBMax = worldCorner;
    
    // Transform remaining corners and expand world AABB
    for (int i = 1; i < 8; ++i) {
        worldCorner = m_position + m_orientation * m_localCorners[i];
        m_AABBMin = glm::min(m_AABBMin, worldCorner);
        m_AABBMax = glm::max(m_AABBMax, worldCorner);
    }

    // Mark simple AABB as valid for this timestep
    m_simpleAABBValidUntilTime = currentTimestep;
}

void GridCollider::updateAdvancedAABB(uint64_t currentTimestep) {
    // Check if advanced AABB is still valid
    if (currentTimestep <= m_advancedAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    // Mark advanced AABB as valid for this timestep
    m_advancedAABBValidUntilTime = currentTimestep;

    if (m_cells.empty()) {
        // If no cells, set a minimal AABB
        m_AABBMin = m_position - glm::dvec3(0.1);
        m_AABBMax = m_position + glm::dvec3(0.1);
        return;
    }
    
    // Use simple AABB as placeholder
    updateSimpleAABB(currentTimestep);
}

bool GridCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

RayIntersectionResult GridCollider::intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const {
    // Assume ray is already in grid-local space
    glm::dvec3 localRayStart = rayStart;
    glm::dvec3 localRayEnd = rayEnd;
    
    // Use grid traversal to find cells the ray passes through
    std::vector<glm::ivec3> cellsToCheck = GridGeometry::gridTraversal(localRayStart, localRayEnd);
    
    // Test intersection with each cell encountered during traversal (in order)
    for (const glm::ivec3& cellCoord : cellsToCheck) {
        // Check if this cell has a collider
        const Collider* cellCollider = getCell(cellCoord);
        if (!cellCollider) {
            continue; // No collider at this cell
        }
        
        // Transform grid-local ray to cell-local space
        glm::dvec3 cellCenter = glm::dvec3(cellCoord) + glm::dvec3(0.5, 0.5, 0.5);
        glm::dvec3 cellLocalRayStart = localRayStart - cellCenter;
        glm::dvec3 cellLocalRayEnd = localRayEnd - cellCenter;
        
        // Test ray intersection with this cell's collider (in cell-local space)
        RayIntersectionResult hit = cellCollider->intersectRay(cellLocalRayStart, cellLocalRayEnd);
        
        // First valid hit is closest hit (cells are in traversal order)
        if (hit.t >= 0.0) {
            // Normal is already in correct space (cell and grid have same orientation)
            return hit; // Early exit - first hit is closest
        }
    }
    
    return RayIntersectionResult(); // No hits found
}

void GridCollider::addCell(const glm::ivec3& coord, std::unique_ptr<Collider> collider) {
    assert(!m_cells.count(coord) && "addCell: coordinate already occupied");
    // Position the collider at the grid coordinate
    glm::dvec3 worldPos = gridToWorld(glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5));
    collider->m_position = worldPos;
    collider->m_orientation = m_orientation;

    // Create and set metadata on the collider
    auto metadata = std::make_unique<CellMetadata>();
    collider->set_pointer<CellMetadata>(metadata.get());

    // Set up dependent positioning
    collider->m_dependentPosition = this;
    collider->m_dependentOffset = glm::dvec3(coord) + glm::dvec3(0.5, 0.5, 0.5);
    
    // Store pointer before move for classification map and metadata ownership
    Collider* cellPtr = collider.get();
    m_cellMetadata[cellPtr] = std::move(metadata);
    m_cells[coord] = std::move(collider);

    // Add to corner cells map (default classification)
    m_cornerCells[coord] = cellPtr;

    // Update axis counts
    m_xAxisCounts[coord.x]++;
    m_yAxisCounts[coord.y]++;
    m_zAxisCounts[coord.z]++;
    
    // Update local AABB
    if (m_cells.size() == 1) { // This is the first cell
        // First cell - initialize AABB
        m_localAABBMin = coord;
        m_localAABBMax = coord;
        m_cornersValidUntilTime = 0; // Invalidate corners
    } else {
        // Expand AABB to include new cell
        glm::ivec3 newMin = glm::min(m_localAABBMin, coord);
        glm::ivec3 newMax = glm::max(m_localAABBMax, coord);
        
        if (newMin != m_localAABBMin || newMax != m_localAABBMax) {
            m_localAABBMin = newMin;
            m_localAABBMax = newMax;
            m_cornersValidUntilTime = 0; // Invalidate corners
        }
    }

    updateFilterNormalsForCell(coord);

    // Increment shape change timestamp to invalidate collision cache
    m_shapeChangeTimestamp++;

    // Queue this coordinate and its face + edge neighbors for classification update
    static const int numDirections = 26;
    static const glm::ivec3 directions[numDirections] = {
        // Face neighbors (6)
        { 1, 0, 0}, {-1, 0, 0},   // ±X
        { 0, 1, 0}, { 0,-1, 0},   // ±Y
        { 0, 0, 1}, { 0, 0,-1},   // ±Z

        // Edge neighbors (12)
        { 1, 1, 0}, { 1,-1, 0}, {-1, 1, 0}, {-1,-1, 0},   // XY edges
        { 1, 0, 1}, { 1, 0,-1}, {-1, 0, 1}, {-1, 0,-1},   // XZ edges
        { 0, 1, 1}, { 0, 1,-1}, { 0,-1, 1}, { 0,-1,-1},   // YZ edges

        // Corner neighbors (8)
        { 1, 1, 1}, { 1, 1,-1}, { 1,-1, 1}, { 1,-1,-1},
        {-1, 1, 1}, {-1, 1,-1}, {-1,-1, 1}, {-1,-1,-1}
    };

     
    queueCoordinateForClassification(coord);
    for (int i = 0; i < numDirections; ++i) {
        queueCoordinateForClassification(coord + directions[i]);
    }

    // Update neighborhoods for collision detection optimization
    Collider* newCell = m_cells[coord].get();
    
    // Get or create neighborhood for this coordinate
    auto& neighborhood = m_neighborhoods[coord];

    // Insert center cell as first element
    neighborhood.m_neighbors.insert(neighborhood.m_neighbors.begin(), newCell);
    neighborhood.m_hasCenter = true;
    
    // Update mutual references with all neighboring coordinates
    for (int dx = -NEIGHBORHOOD_RADIUS; dx <= NEIGHBORHOOD_RADIUS; ++dx) {
        for (int dy = -NEIGHBORHOOD_RADIUS; dy <= NEIGHBORHOOD_RADIUS; ++dy) {
            for (int dz = -NEIGHBORHOOD_RADIUS; dz <= NEIGHBORHOOD_RADIUS; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) continue; // Skip center
                
                glm::ivec3 neighborCoord = coord + glm::ivec3(dx, dy, dz);
                auto neighborhoodIt = m_neighborhoods.find(neighborCoord);
                if (neighborhoodIt != m_neighborhoods.end()) {
                    // Always add ourselves to neighbor's list
                    neighborhoodIt->second.m_neighbors.push_back(newCell);
                    
                    // No need to add neighbor's center to our list since it already did when it was created.
                } else {
                    // Create neighbor and add reference to us
                    m_neighborhoods.emplace(neighborCoord, CellNeighborhood()).first->second.m_neighbors.push_back(newCell);
                }
            }
        }
    }
}

void GridCollider::removeCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        // Get pointer before erasing for neighborhood updates
        Collider* removedCell = it->second.get();

        // Check if this cell is on the border before removing
        bool onBorder = (coord.x == m_localAABBMin.x || coord.x == m_localAABBMax.x ||
                        coord.y == m_localAABBMin.y || coord.y == m_localAABBMax.y ||
                        coord.z == m_localAABBMin.z || coord.z == m_localAABBMax.z);
        
        // Update axis counts
        m_xAxisCounts[coord.x]--;
        if (m_xAxisCounts[coord.x] == 0) {
            m_xAxisCounts.erase(coord.x);
        }
        m_yAxisCounts[coord.y]--;
        if (m_yAxisCounts[coord.y] == 0) {
            m_yAxisCounts.erase(coord.y);
        }
        m_zAxisCounts[coord.z]--;
        if (m_zAxisCounts[coord.z] == 0) {
            m_zAxisCounts.erase(coord.z);
        }
        
        // Only recalculate AABB if the removed cell was on the border
        if (onBorder && !m_xAxisCounts.empty()) {
            auto xRange = std::minmax_element(m_xAxisCounts.begin(), m_xAxisCounts.end());
            auto yRange = std::minmax_element(m_yAxisCounts.begin(), m_yAxisCounts.end());
            auto zRange = std::minmax_element(m_zAxisCounts.begin(), m_zAxisCounts.end());
            m_localAABBMin = glm::ivec3(xRange.first->first, yRange.first->first, zRange.first->first);
            m_localAABBMax = glm::ivec3(xRange.second->first, yRange.second->first, zRange.second->first);
            m_cornersValidUntilTime = 0; // Invalidate corners
        }

        updateFilterNormalsAfterRemoval(coord);

        // Increment shape change timestamp to invalidate collision cache
        m_shapeChangeTimestamp++;

        // Remove from classification maps
        m_cornerCells.erase(coord);
        m_edgeCells.erase(coord);

        // Queue this coordinate and its face + edge neighbors for classification update
        static const int numDirections = 26;
        static const glm::ivec3 directions[numDirections] = {
            // Face neighbors (6)
            { 1, 0, 0}, {-1, 0, 0},   // ±X
            { 0, 1, 0}, { 0,-1, 0},   // ±Y
            { 0, 0, 1}, { 0, 0,-1},   // ±Z

            // Edge neighbors (12)
            { 1, 1, 0}, { 1,-1, 0}, {-1, 1, 0}, {-1,-1, 0},   // XY edges
            { 1, 0, 1}, { 1, 0,-1}, {-1, 0, 1}, {-1, 0,-1},   // XZ edges
            { 0, 1, 1}, { 0, 1,-1}, { 0,-1, 1}, { 0,-1,-1},   // YZ edges

            // Corner neighbors (8)
            { 1, 1, 1}, { 1, 1,-1}, { 1,-1, 1}, { 1,-1,-1},
            {-1, 1, 1}, {-1, 1,-1}, {-1,-1, 1}, {-1,-1,-1}
        };

        for (int i = 0; i < numDirections; ++i) {
            queueCoordinateForClassification(coord + directions[i]);
        }

        // Update neighborhoods for collision detection optimization
        for (int dx = -NEIGHBORHOOD_RADIUS; dx <= NEIGHBORHOOD_RADIUS; ++dx) {
            for (int dy = -NEIGHBORHOOD_RADIUS; dy <= NEIGHBORHOOD_RADIUS; ++dy) {
                for (int dz = -NEIGHBORHOOD_RADIUS; dz <= NEIGHBORHOOD_RADIUS; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue; // Skip center

                    glm::ivec3 neighborCoord = coord + glm::ivec3(dx, dy, dz);
                    auto neighborhoodIt = m_neighborhoods.find(neighborCoord);
                    if (neighborhoodIt != m_neighborhoods.end()) {
                        // Remove this cell from neighbor's list
                        auto& neighbors = neighborhoodIt->second.m_neighbors;
                        neighbors.erase(std::remove(neighbors.begin(), neighbors.end(), removedCell), neighbors.end());

                        // Clean up neighbor if it's now empty (inline cleanup)
                        if (neighborhoodIt->second.m_neighbors.empty()) {
                            m_neighborhoods.erase(neighborhoodIt);
                        }
                    }
                }
            }
        }

        // Remove this neighborhood
        auto selfNeighborhoodIt = m_neighborhoods.find(coord);
        if (selfNeighborhoodIt != m_neighborhoods.end()) {
            // Remove self from own neighbors list and clear center flag
            auto& selfNeighbors = selfNeighborhoodIt->second.m_neighbors;
            selfNeighbors.erase(std::remove(selfNeighbors.begin(), selfNeighbors.end(), removedCell), selfNeighbors.end());
            selfNeighborhoodIt->second.m_hasCenter = false;

            // Only remove neighborhood if no neighbors left
            if (selfNeighbors.empty()) {
                m_neighborhoods.erase(selfNeighborhoodIt);
            }
        } else {
            throw std::runtime_error("GridCollider::removeCell: Neighborhood not found for removed cell coordinate");
        }

        // Erase after all uses of removedCell — m_cells.erase destroys the Collider
        m_cellMetadata.erase(removedCell);
        m_cells.erase(it);
    }
}

bool GridCollider::hasCell(const glm::ivec3& coord) const {
    return m_cells.find(coord) != m_cells.end();
}

Collider* GridCollider::getCell(const glm::ivec3& coord) {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        return it->second.get();
    }
    return nullptr;
}

const Collider* GridCollider::getCell(const glm::ivec3& coord) const {
    auto it = m_cells.find(coord);
    if (it != m_cells.end()) {
        return it->second.get();
    }
    return nullptr;
}

void GridCollider::updateSubColliderTransformsAndAABB(uint64_t currentTimestep) {
    // TODOO: I think this function is never called. Maybe it can be removed?
    // Update position and orientation of all sub-colliders
    for (const auto& pair : m_cells) {
        const glm::ivec3& coord = pair.first;
        Collider* subCollider = pair.second.get();
        
        // Calculate world position for this grid cell center
        glm::dvec3 cellCenter = glm::dvec3(coord) + glm::dvec3(0.5);
        subCollider->m_position = gridToWorld(cellCenter);
        subCollider->m_orientation = m_orientation;
        
        // Update the sub-collider's AABB
        subCollider->updateSimpleAABB(currentTimestep);
    }
}

glm::dvec3 GridCollider::gridToWorld(const glm::dvec3& gridCoord) const {
    // Convert grid coordinates to world coordinates
    glm::dvec3 localPos = gridCoord;
    
    // Apply orientation and position transform
    return m_position + m_orientation * localPos;
}
glm::dvec3 GridCollider::worldToGrid(const glm::dvec3& worldCoord) const {
    // Inverse transformation: rotate by conjugate orientation, then translate
    return glm::conjugate(m_orientation) * (worldCoord - m_position);
}

void GridCollider::updateFilterNormalsForCell(const glm::ivec3& coord) {
    return; // Temporarily disable filters.

    // Standard grid directions (6-connectivity)
    static const int numDirections = 6;
    static const glm::dvec3 directions[numDirections] = {
        { 1.0,  0.0,  0.0},  // +X
        {-1.0,  0.0,  0.0},  // -X
        { 0.0,  1.0,  0.0},  // +Y
        { 0.0, -1.0,  0.0},  // -Y
        { 0.0,  0.0,  1.0},  // +Z
        { 0.0,  0.0, -1.0}   // -Z
    };
    
    Collider* currentCube = getCell(coord);
    if (!currentCube) return;
    
    // Clear existing filter normals for this cube
    currentCube->clearFilterNormals();
    
    // Check each direction for neighbors
    for (int i = 0; i < numDirections; ++i) {
        glm::ivec3 neighborCoord = coord + glm::ivec3(directions[i]);
        Collider* neighborCube = getCell(neighborCoord);
        
        if (neighborCube) {
            // Add filter normal pointing toward neighbor
            currentCube->addFilterNormal(directions[i]);
            
            // Add filter normal to neighbor pointing back toward current cube
            neighborCube->addFilterNormal(-directions[i]);
        }
    }
}

void GridCollider::updateFilterNormalsAfterRemoval(const glm::ivec3& removedCoord) {
    // Standard grid directions (6-connectivity)
    static const int numDirections = 6;
    static const glm::dvec3 directions[numDirections] = {
        { 1.0,  0.0,  0.0},  // +X
        {-1.0,  0.0,  0.0},  // -X
        { 0.0,  1.0,  0.0},  // +Y
        { 0.0, -1.0,  0.0},  // -Y
        { 0.0,  0.0,  1.0},  // +Z
        { 0.0,  0.0, -1.0}   // -Z
    };
    
    // Update neighbors of the removed cell
    for (int i = 0; i < numDirections; ++i) {
        glm::ivec3 neighborCoord = removedCoord + glm::ivec3(directions[i]);
        Collider* neighborCube = getCell(neighborCoord);
        
        if (neighborCube) {
            // Remove filter normal pointing toward the removed cube
            neighborCube->removeFilterNormal(-directions[i]);
        }
    }
}

double GridCollider::getApproximateRadius() const {
    if (!m_cells.empty()) {
        glm::dvec3 bboxSize = m_AABBMax - m_AABBMin;
        double maxDimension = glm::max(glm::max(bboxSize.x, bboxSize.y), bboxSize.z);
        return maxDimension * 0.5;
    }
    
    // Fallback to radius = 1.0
    return 1.0;
}

void GridCollider::updateLocalCorners() {
    // Expand local AABB by half diagonal of cube to encompass entire cubes
    const double halfDiagonal = std::sqrt(3.0) * 0.5;
    glm::dvec3 expandedMin = glm::dvec3(m_localAABBMin) - glm::dvec3(halfDiagonal - 0.5);
    glm::dvec3 expandedMax = glm::dvec3(m_localAABBMax) + glm::dvec3(halfDiagonal + 0.5);
    
    m_localCorners[0] = glm::dvec3(expandedMin.x, expandedMin.y, expandedMin.z);
    m_localCorners[1] = glm::dvec3(expandedMax.x, expandedMin.y, expandedMin.z);
    m_localCorners[2] = glm::dvec3(expandedMin.x, expandedMax.y, expandedMin.z);
    m_localCorners[3] = glm::dvec3(expandedMax.x, expandedMax.y, expandedMin.z);
    m_localCorners[4] = glm::dvec3(expandedMin.x, expandedMin.y, expandedMax.z);
    m_localCorners[5] = glm::dvec3(expandedMax.x, expandedMin.y, expandedMax.z);
    m_localCorners[6] = glm::dvec3(expandedMin.x, expandedMax.y, expandedMax.z);
    m_localCorners[7] = glm::dvec3(expandedMax.x, expandedMax.y, expandedMax.z);
}

std::vector<Collider*> GridCollider::findCellsInRadius(const glm::dvec3& worldPos, double searchRadius) const {
    std::vector<Collider*> foundColliders;
    
    if (m_cells.empty()) {
        return foundColliders;
    }
    
    // Calculate spatial search bounds
    glm::dvec3 gridSpaceCenter = worldToGrid(worldPos);
    glm::ivec3 minCoord = glm::floor(gridSpaceCenter - glm::dvec3(searchRadius));
    glm::ivec3 maxCoord = glm::floor(gridSpaceCenter + glm::dvec3(searchRadius));
    
    for (int x = minCoord.x; x <= maxCoord.x; ++x) {
        for (int y = minCoord.y; y <= maxCoord.y; ++y) {
            for (int z = minCoord.z; z <= maxCoord.z; ++z) {
                glm::ivec3 targetCoord(x, y, z);
                
                auto cellIt = m_cells.find(targetCoord);
                if (cellIt == m_cells.end()) {
                    continue; // No cell at this coordinate
                }
                
                foundColliders.push_back(cellIt->second.get());
            }
        }
    }
    
    return foundColliders;
}

void GridCollider::queueCoordinateForClassification(const glm::ivec3& coord) {
    // Only add if not already queued
    if (m_queuedCoordinates.find(coord) == m_queuedCoordinates.end()) {
        m_classificationQueue.push(coord);
        m_queuedCoordinates.insert(coord);
        scheduleClassificationJob();
    }
}

void GridCollider::scheduleClassificationJob() {
    if (m_classificationJob.lock()) {
        return; // Job exists (active or cancelled) - don't schedule another
    }
    
    m_classificationJob = m_jobManager->schedule([this](std::chrono::time_point<std::chrono::high_resolution_clock> endTime) -> bool {
        return processClassificationQueue(endTime);
    }, m_classificationJobPriority);
}

bool GridCollider::processClassificationQueue(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    while (!m_classificationQueue.empty() && m_timeHandler->now() < endTime) {
        glm::ivec3 coord = m_classificationQueue.front();
        m_classificationQueue.pop();
        m_queuedCoordinates.erase(coord);
        
        // Check if cell exists in the collider
        Collider* collider = getCell(coord);
        if (!collider) {
            continue; // Skip non-existent cells
        }
        
        // Classify the cell and update its metadata
        CellMetadata::CellClassification newClassification = classifyCell(coord);
        
        CellMetadata* metadata = collider->get_pointer<CellMetadata>();
        if (metadata) {
            CellMetadata::CellClassification oldClassification = metadata->classification;
            metadata->classification = newClassification;

            // Update classification maps to match new classification
            if (oldClassification == CellMetadata::CellClassification::CORNER) {
                m_cornerCells.erase(coord);
            }
            if (oldClassification == CellMetadata::CellClassification::EDGE) {
                m_edgeCells.erase(coord);
            }
            
            if (newClassification == CellMetadata::CellClassification::CORNER) {
                m_cornerCells[coord] = collider;
            }
            if (newClassification == CellMetadata::CellClassification::EDGE) {
                m_edgeCells[coord] = collider;
            }
            // Note: FACE and INNER classifications are not added to any map
            // since they don't participate in collision detection optimization
        }
    }
    
    return !m_classificationQueue.empty(); // Return true if more work needed
}

CellMetadata::CellClassification GridCollider::classifyCell(const glm::ivec3& coord) {

    // 1. Get visible triangles for current cell
    auto visibleTrianglesResult = getVisibleTriangles(coord);
    
    if (visibleTrianglesResult.triangles.empty()) {
        return CellMetadata::CellClassification::INNER; // No visible triangles = completely hidden
    }
    
    // 3. Collect all foreign triangles
    static const int numNeighbors = 26;
    static const glm::ivec3 neighborDirections[numNeighbors] = {
        // Face neighbors (6)
        { 1, 0, 0}, {-1, 0, 0},   // ±X
        { 0, 1, 0}, { 0,-1, 0},   // ±Y
        { 0, 0, 1}, { 0, 0,-1},   // ±Z
    
        // Edge neighbors (12)
        { 1, 1, 0}, { 1,-1, 0}, {-1, 1, 0}, {-1,-1, 0},   // XY edges
        { 1, 0, 1}, { 1, 0,-1}, {-1, 0, 1}, {-1, 0,-1},   // XZ edges
        { 0, 1, 1}, { 0, 1,-1}, { 0,-1, 1}, { 0,-1,-1},   // YZ edges
    
        // Corner neighbors (8)
        { 1, 1, 1}, { 1, 1,-1}, { 1,-1, 1}, { 1,-1,-1},
        {-1, 1, 1}, {-1, 1,-1}, {-1,-1, 1}, {-1,-1,-1}
    };
    
    // 4. Merge local and adjacent foreign triangles using indices
    const double vertexTolerance = 1e-5;
    
    // Start with local vertices and triangles
    std::vector<glm::dvec3> mergedVertices = visibleTrianglesResult.vertices;
    std::vector<std::array<int, 3>> mergedTriangles = visibleTrianglesResult.triangles;
     
    // Phase 1: Identify relevant foreign triangles that share at least 1 vertex with locals
    std::vector<std::array<glm::dvec3, 3>> candidateForeignTriangles;
    for (int i = 0; i < numNeighbors; ++i) {
        glm::ivec3 neighborCoord = coord + neighborDirections[i];
        auto neighborIt = m_cells.find(neighborCoord);
        
        if (neighborIt == m_cells.end()) {
            continue; // Neighbor doesn't exist
        }
        
        // Get neighbor's visible triangles
        auto neighborVisibleResult = getVisibleTriangles(neighborCoord);
        
        // Transform neighbor triangles to current cell's coordinate system
        glm::dvec3 offset = glm::dvec3(neighborDirections[i]);
        
        for (const auto& foreignTriangleIndices : neighborVisibleResult.triangles) {
            // Convert foreign triangle to dvec3
            std::array<glm::dvec3, 3> foreignTriangle;
            for (int v = 0; v < 3; ++v) {
                foreignTriangle[v] = neighborVisibleResult.vertices[foreignTriangleIndices[v]] + offset;
            }
            // Check if this foreign triangle shares ≥1 vertex with ANY local triangle
            bool sharesVertex = false;
            for (const auto& localTriangleIndices : visibleTrianglesResult.triangles) {
                // Convert local triangle indices to dvec3
                std::array<glm::dvec3, 3> localTriangle;
                for (int v = 0; v < 3; ++v) {
                    localTriangle[v] = visibleTrianglesResult.vertices[localTriangleIndices[v]];
                }
                
                // Check if any vertex of local triangle is close to foreign triangle
                bool isTouching = false;
                const double touchingThreshold = 1e-5;
                
                for (int v = 0; v < 3; ++v) {
                    double distance = GeometryUtils::pointToTriangleDistance(
                        localTriangle[v], foreignTriangle[0], foreignTriangle[1], foreignTriangle[2]);
                    if (distance <= touchingThreshold) {
                        isTouching = true;
                        break;
                    }
                }
                
                if (isTouching) {
                    sharesVertex = true;
                    break; // Found shared vertex, no need to check other locals
                }
            }
            if (sharesVertex) {
                candidateForeignTriangles.push_back(foreignTriangle);
            }
        }
    }

    // Phase 2: Merge all candidate foreign triangles using comprehensive vertex mapping
    for (const auto& foreignTriangle : candidateForeignTriangles) {
        std::array<int, 3> newTriangleIndices;
        
        for (int v = 0; v < 3; ++v) {
            // Check if foreign vertex matches ANY local vertex (not just specific triangle)
            bool foundMatch = false;
            for (size_t idx = 0; idx < mergedVertices.size(); ++idx) {
                if (glm::length(mergedVertices[idx] - foreignTriangle[v]) < vertexTolerance) {
                    newTriangleIndices[v] = static_cast<int>(idx);
                    foundMatch = true;
                    break;
                }
            }
            
            if (!foundMatch) {
                // This is a new vertex - add it to merged vertices
                newTriangleIndices[v] = static_cast<int>(mergedVertices.size());
                mergedVertices.push_back(foreignTriangle[v]);
            }
        }
        
        mergedTriangles.push_back(newTriangleIndices);
    }
    
    // 5. Classification using new index-based functions
    // Check if surface is completely flat/concave (FACE classification)
    std::vector<bool> triangleMask(mergedTriangles.size(), false);
    // Set local triangles to true (first N triangles are local)
    for (size_t i = 0; i < visibleTrianglesResult.triangles.size(); ++i) {
        triangleMask[i] = true;
    }
    
    if (PolyhedronProcessor::areTrianglesConvex(mergedVertices, mergedTriangles, triangleMask, false)) {
        return CellMetadata::CellClassification::FACE;
    }

    // Create indices for local vertices only (foreign vertices are just context)
    std::vector<int> localVertexIndices;
    for (int i = 0; i < static_cast<int>(visibleTrianglesResult.vertices.size()); ++i) {
        localVertexIndices.push_back(i);
    }
    
    // Check if surface has at least one convex vertex (CORNER vs EDGE classification)
    if (!PolyhedronProcessor::hasAtLeastOneConvexVertex(mergedVertices, mergedTriangles, localVertexIndices)) {
        return CellMetadata::CellClassification::EDGE;
    }
    return CellMetadata::CellClassification::CORNER;
}

VisibleTrianglesResult GridCollider::getVisibleTriangles(const glm::ivec3& coord) {
    VisibleTrianglesResult result;
    
    // 1: Get the cell and its vertices
    auto it = m_cells.find(coord);
    if (it == m_cells.end()) {
        return result; // Empty result for non-existent cell
    }
    
    // Get the polyhedron collider
    Collider* collider = it->second.get();
    assert(collider->getTypeId() == PolyhedronCollider::TYPE_ID && "getVisibleTriangles requires PolyhedronCollider cells");
    PolyhedronCollider* polyhedron = static_cast<PolyhedronCollider*>(collider);
    
    // Get local vertices and convert to cell coordinate system [0,1]
    std::vector<glm::dvec3> localVertices = polyhedron->getLocalVertices();
    
    std::vector<glm::dvec3> vertices;
    vertices.reserve(localVertices.size());
    for (const auto& localVertex : localVertices) {
        vertices.push_back(localVertex + glm::dvec3(0.5)); // Convert from [-0.5,0.5] to [0,1]
    }
    
    // 2: Check which vertices are hidden by neighbors
    std::vector<std::vector<bool>> vertexHiddenByNeighbor(6, std::vector<bool>(vertices.size(), false));
    
    static const int numDirections = 6;
    static const glm::ivec3 directions[numDirections] = {
        {1, 0, 0}, {-1, 0, 0},   // +X, -X
        {0, 1, 0}, {0, -1, 0},   // +Y, -Y
        {0, 0, 1}, {0, 0, -1}    // +Z, -Z
    };
    
    for (int i = 0; i < numDirections; ++i) {
        glm::ivec3 neighborCoord = coord + directions[i];
        auto neighborIt = m_cells.find(neighborCoord);
        
        if (neighborIt == m_cells.end()) {
            continue; // No neighbor, vertices remain visible
        }
        
        assert(neighborIt->second->getTypeId() == PolyhedronCollider::TYPE_ID && "getVisibleTriangles requires PolyhedronCollider cells");
        PolyhedronCollider* neighborPolyhedron = static_cast<PolyhedronCollider*>(neighborIt->second.get());
        
        std::vector<glm::dvec3> neighborLocalVertices = neighborPolyhedron->getLocalVertices();

        std::vector<glm::dvec3> neighborVertices;
        neighborVertices.reserve(neighborLocalVertices.size());
        for (const auto& localVertex : neighborLocalVertices) {
            neighborVertices.push_back(localVertex + glm::dvec3(0.5)); // Convert from [-0.5,0.5] to [0,1]
        }
        
        // Call checkPolyhedronBorderIntersection (now in PolyhedronProcessor)
        auto hiddenFlags = PolyhedronProcessor::checkPolyhedronBorderIntersection(
            coord, vertices, neighborCoord, neighborVertices);
        
        for (int v = 0; v < static_cast<int>(vertices.size()); ++v) {
            vertexHiddenByNeighbor[i][v] = hiddenFlags[v];
        }
    }
    
    // 3: Get triangle indices and filter visible ones (using dvec3 version)
    auto allTriangleIndices = polyhedron->generateTriangleIndices();
    
    std::vector<std::array<int, 3>> visibleTriangleIndices;
    
    for (const auto& triangleIndices : allTriangleIndices) {
        bool isTriangleHidden = false;
        
        // Check if any single neighbor hides ALL three triangle vertices
        for (int neighborDir = 0; neighborDir < numDirections; ++neighborDir) {
            bool allVerticesHiddenByThisNeighbor = true;
            
            for (int triVertex = 0; triVertex < 3; ++triVertex) {
                int cubeVertexIndex = triangleIndices[triVertex];
                if (cubeVertexIndex < static_cast<int>(vertices.size()) && !vertexHiddenByNeighbor[neighborDir][cubeVertexIndex]) {
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
            visibleTriangleIndices.push_back(triangleIndices);
        }
    }
    
    // 4: Create unique vertex list and remap triangle indices
    std::unordered_map<glm::dvec3, int, Hash::DVec3Hash> vertexToIndex;
    result.vertices.reserve(vertices.size());
    
    for (const auto& triangleIndices : visibleTriangleIndices) {
        std::array<int, 3> newTriangleIndices;
        
        for (int i = 0; i < 3; ++i) {
            glm::dvec3 vertex = vertices[triangleIndices[i]];
            
            auto it = vertexToIndex.find(vertex);
            if (it == vertexToIndex.end()) {
                // Add new unique vertex
                int newIndex = static_cast<int>(result.vertices.size());
                result.vertices.push_back(vertex);
                vertexToIndex[vertex] = newIndex;
                newTriangleIndices[i] = newIndex;
            } else {
                // Use existing vertex index
                newTriangleIndices[i] = it->second;
            }
        }
        
        result.triangles.push_back(newTriangleIndices);
    }
    
    return result;
}