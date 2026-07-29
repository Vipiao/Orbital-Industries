// Grid.h
#pragma once

#include "utils/IHashable.h"
#include "utils/HashFunctions.h"
#include "utils/StochasticAnalyzer.h"
#include "utils/GridGeometry.h"
#include "utils/GeometryUtils.h"
#include "StructuralBlock.h"
#include "SecondaryCell.h"
#include "thruster/ThrusterBlock.h"
#include "cockpit/CockpitBlock.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>
#include <unordered_map>
#include <unordered_set>

// Forward declarations
class PhysicsEngine;
class GraphicsEngine;
class RigidBody;
class Collider;
class GridCollider;
class GridGraphics;
class BlockResourceCache;
class TimeHandler;
class JobManager;
class Job;

class Grid : public IHashable {
public:
    // Unique identifier for deterministic sorting
    const uint64_t uniqueId;

    // Id supplied by the caller (GridSubsystem owns allocation) rather than auto-generated.
    Grid(uint64_t uniqueId, PhysicsEngine* physics, GraphicsEngine* graphics,
         JobManager* jobManager, TimeHandler* timeHandler, BlockResourceCache* blockResources,
         const glm::dvec3& position,
         const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    ~Grid();
    
    // Cell management methods
    void addCell(const glm::ivec3& coord);
    void addThruster(const glm::ivec3& anchorCoord,
                     const glm::dquat& orientation = glm::dquat{1.0, 0.0, 0.0, 0.0});
    void addCockpit(const glm::ivec3& anchorCoord,
                    const glm::dquat& orientation = glm::dquat{1.0, 0.0, 0.0, 0.0});

    // Returns all coords removed (1 for structural block, 2 for thruster anchor+secondary)
    std::vector<glm::ivec3> removeCell(const glm::ivec3& coord);

    bool hasCell(const glm::ivec3& coord) const;
    bool isEmpty() const;

    /**
     * @brief Check if a cell can be modified with new vertices
     * @param coord Grid coordinate of the cell
     * @param newVertices New 8-vertex configuration
     * @return true if modification would be valid, false otherwise
     */
    bool canModifyCell(const glm::ivec3& coord, const std::array<glm::ivec3, 8>& newVertices) const;
    
    /**
     * @brief Modify an existing structural block's shape
     * @param coord Grid coordinate of the cell to modify
     * @param newVertices New 8-vertex configuration
     * @return true if modification was successful, false otherwise
     */
    bool modifyCell(const glm::ivec3& coord, const std::array<glm::ivec3, 8>& newVertices);

    /**
     * @brief Nudge one corner of a cell by a direction, validating the result.
     *
     * Applies the nudge to the cell's current vertices, so repeated nudges compose,
     * and only commits if the resulting shape is valid. cornerIndex must be in [0, 8).
     * @return true if the modification was applied, false otherwise
     */
    bool nudgeCellVertex(const glm::ivec3& coord, int cornerIndex, const glm::ivec3& direction);

    /**
     * @brief Set color of an existing cell and update graphics
     * @param coord Grid coordinate of the cell
     * @param newColor New color to apply
     */
    void setColor(const glm::ivec3& coord, const glm::dvec4& newColor);

    // Bumped by every call that changes cells, thruster/cockpit anchors or a
    // cell's color — never by pose, motion or throttle. A cheap way for a
    // consumer to tell whether anything it cares about has changed since it
    // last looked, without knowing what that consumer is (e.g. a cached
    // fingerprint of the structure, see GridSerializer::structureHash).
    uint64_t getStructureVersion() const { return m_structureVersion; }

    // Structural analysis visualization (analysis runs automatically as background job)
    void visualizeStructuralIntegrity();
    
    // Graphics updates
    void updateGraphics(const glm::dvec3& cameraPos);
    
    std::weak_ptr<RigidBody> getRigidBody() const { return m_rigidBody; }

    // Radius of a sphere that encloses the grid's cells, about its origin. A coarse
    // extent for culling, priority and other whole-grid size estimates.
    double getApproximateRadius() const;

    // Get collider for subsystem queries
    std::weak_ptr<Collider> getCollider() const;

    int getGridSSBOIndex() const;

    // Convert world coordinates to grid-local coordinates
    glm::dvec3 worldToGrid(const glm::dvec3& worldPos) const;
    
    // Convert grid-local coordinates to world coordinates
    glm::dvec3 gridToWorld(const glm::dvec3& gridPos) const;

    /**
     * @brief Get interpolated rigid body transform for rendering/queries
     * @param timeRemainder Interpolation parameter from graphics engine
     * @param outPosition Output interpolated position
     * @param outOrientation Output interpolated orientation
     */
    void getInterpolatedTransform(double timeRemainder, glm::dvec3& outPosition, glm::dquat& outOrientation) const;

    /**
     * @brief Perform ray intersection test against this grid
     * @param rayStart Ray start position in world coordinates
     * @param rayEnd Ray end position in world coordinates  
     * @return Ray intersection result with t parameter and surface normal
     */
    RayIntersectionResult intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const;
    
    // Center of mass in local space. Get m_rigidBody->m_position to get the world center of mass.
    glm::dvec3 m_centerOfMass{0.0, 0.0, 0.0};

    // Structural block access (for StochasticAnalyzer and vertex/color data)
    const std::unordered_map<glm::ivec3, StructuralBlock, Hash::IVec3Hash>& getCells() const { return m_cells; }
    StructuralBlock* getCell(const glm::ivec3& coord);

    // Generic registry of all cells (structural blocks + thruster anchor + thruster secondary)
    const std::unordered_map<glm::ivec3, GridCell*, Hash::IVec3Hash>& getCellRegistry() const { return m_cellRegistry; }
    GridCell* getCellFromRegistry(const glm::ivec3& coord);

    // Thruster anchors (owning map); levels are driven via setThrusterLevel.
    const std::unordered_map<glm::ivec3, ThrusterBlock, Hash::IVec3Hash>& getThrusterCells() const {
        return m_thrusterCells;
    }

    // Cockpit anchors (owning map).
    const std::unordered_map<glm::ivec3, CockpitBlock, Hash::IVec3Hash>& getCockpitCells() const {
        return m_cockpitCells;
    }

    // Set the stored throttle [0, 1] of the thruster anchored at the coord
    // (no-op if there is no thruster there).
    void setThrusterLevel(const glm::ivec3& anchorCoord, double level);

    // IHashable interface
    virtual size_t computeHash() const override;

private:
    uint64_t m_structureVersion{0};

    // Job management
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;
    std::weak_ptr<Job> m_analysisJob;
    
    // Analysis state
    int m_currentAnalysisIteration = 0;
    static constexpr int MAX_ANALYSIS_ITERATIONS = 8;
    static constexpr double WEAKNESS_BLEND_FACTOR = 0.2; // New result weight in running average
    
    // Structural blocks (owning)
    std::unordered_map<glm::ivec3, StructuralBlock, Hash::IVec3Hash> m_cells;

    // Special block anchor cells (owning, one map per concrete type)
    std::unordered_map<glm::ivec3, ThrusterBlock, Hash::IVec3Hash> m_thrusterCells;
    std::unordered_map<glm::ivec3, CockpitBlock,  Hash::IVec3Hash> m_cockpitCells;

    // All secondary cells across every block type (owning, unified map)
    std::unordered_map<glm::ivec3, SecondaryCell, Hash::IVec3Hash> m_secondaryCells;

    // Non-owning registry — all cells of any type, updated in sync with the owning maps
    std::unordered_map<glm::ivec3, GridCell*, Hash::IVec3Hash> m_cellRegistry;

    // Stochastic analysis (structural blocks only)
    std::unique_ptr<StochasticAnalyzer<StructuralBlock>> m_stochasticAnalyzer;
        
    // External system references
    // Mesh update job system
    std::weak_ptr<Job> m_meshUpdateJob;
    std::unordered_set<glm::ivec3, Hash::IVec3Hash> m_pendingMeshUpdates;

    PhysicsEngine* m_physics;
    std::weak_ptr<RigidBody> m_rigidBody;

    std::weak_ptr<GridCollider> m_colliderWeak;
    // Graphics subsystem
    std::unique_ptr<GridGraphics> m_gridGraphics;

    // Structural analysis methods
    bool performStructuralAnalysisUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
    void scheduleStructuralAnalysis();
    void cancelStructuralAnalysis();

    std::vector<std::weak_ptr<Job>> m_pendingJobs;
    void trackJob(std::weak_ptr<Job> jobHandle);

    // Neighbor connection management
    void updateNeighborConnections(const glm::ivec3& coord);
    void removeNeighborConnections(const glm::ivec3& coord);
    
    // Face visibility and mesh management methods
    void updateCellMassContribution(const glm::ivec3& coord, double sign);
    void updateRigidBodyInverses();

    // Mesh generation and filtering
    PolyhedronProcessor::MeshData generateFilteredMeshData(const glm::ivec3& coord);
    void scheduleMeshUpdateForCell(const glm::ivec3& coord);
    void scheduleMeshUpdatesForCellAndNeighbors(const glm::ivec3& coord);
    bool processPendingMeshUpdates(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
};