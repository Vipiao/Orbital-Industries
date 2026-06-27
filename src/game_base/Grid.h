// Grid.h
#pragma once

#include "../utils/IHashable.h"
#include "../utils/HashFunctions.h"
#include "../utils/StochasticAnalyzer.h"
#include "../utils/GridGeometry.h"
#include "../utils/GeometryUtils.h"
#include "StructuralBlock.h"
#include "thruster/ThrusterBlock.h"
#include "thruster/ThrusterSecondaryCell.h"
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
class TimeHandler;
class JobManager;
class Job;

class Grid : public IHashable {
public:
    // Unique identifier for deterministic sorting
    const uint64_t uniqueId;

    // Constructor now takes physics and graphics pointers
    Grid(PhysicsEngine* physics, GraphicsEngine* graphics, JobManager* jobManager,
         TimeHandler* timeHandler, const glm::dvec3& position,
         const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    ~Grid();
    
    // Cell management methods
    void addCell(const glm::ivec3& coord);
    void addThruster(const glm::ivec3& anchorCoord);

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
     * @brief Set color of an existing cell and update graphics
     * @param coord Grid coordinate of the cell
     * @param newColor New color to apply
     */
    void setColor(const glm::ivec3& coord, const glm::dvec4& newColor);

    // Structural analysis visualization (analysis runs automatically as background job)
    void visualizeStructuralIntegrity();
    
    // Graphics updates
    void updateGraphics(const glm::dvec3& cameraPos);
    
    std::weak_ptr<RigidBody> getRigidBody() const { return m_rigidBody; }

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

    // IHashable interface
    virtual size_t computeHash() const override;

private:
    // Job management
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;
    std::weak_ptr<Job> m_analysisJob;
    
    // Analysis state
    int m_currentAnalysisIteration = 0;
    static constexpr int MAX_ANALYSIS_ITERATIONS = 8;
    static constexpr double WEAKNESS_BLEND_FACTOR = 0.2; // New result weight in running average
    
    // Static counter for unique IDs
    static uint64_t s_nextUniqueId;

    // Structural blocks (owning)
    std::unordered_map<glm::ivec3, StructuralBlock, Hash::IVec3Hash> m_cells;

    // Thruster cells (owning)
    std::unordered_map<glm::ivec3, ThrusterBlock, Hash::IVec3Hash> m_thrusterCells;
    std::unordered_map<glm::ivec3, ThrusterSecondaryCell, Hash::IVec3Hash> m_thrusterSecondaryCells;

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
    double getApproximateRadius() const;

    // Mesh generation and filtering
    PolyhedronProcessor::MeshData generateFilteredMeshData(const glm::ivec3& coord);
    void scheduleMeshUpdateForCell(const glm::ivec3& coord);
    void scheduleMeshUpdatesForCellAndNeighbors(const glm::ivec3& coord);
    bool processPendingMeshUpdates(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);
};