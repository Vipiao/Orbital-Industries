// Grid.h
#pragma once

#include "../physics/RigidBody.h"
#include "../physics/GridCollider.h"
#include "MassInertiaCalculator.h"
#include "../utils/IHashable.h"
#include "../utils/HashFunctions.h"
#include "../utils/StochasticAnalyzer.h"
#include "GridGraphics.h"
#include "GridGeometry.h"
#include "CellType.h"
#include "StructuralBlock.h"
#include "../utils/TimeHandler.h"
#include <glm/glm.hpp>
#include "../utils/JobManager.h"
#include <memory>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>

// Forward declarations
class PhysicsEngine;

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
    void addCell(const glm::ivec3& coord, CellType type = CellType::ARMOR);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    bool isEmpty() const;

    // Structural analysis visualization (analysis runs automatically as background job)
    void visualizeStructuralIntegrity();
    
    // Graphics updates
    void updateGraphics(const glm::dvec3& cameraPos);
    
    RigidBody* getRigidBody() const { return m_rigidBody; }

    // Convert world coordinates to grid-local coordinates
    glm::dvec3 worldToGrid(const glm::dvec3& worldPos) const;
    
    // Convert grid-local coordinates to world coordinates
    glm::dvec3 gridToWorld(const glm::dvec3& gridPos) const;

    std::vector<glm::ivec3> gridTraversal(glm::dvec3 startPos, glm::dvec3 endPos);
    
    // Center of mass in local space. Get m_rigidBody->m_position to get the world center of mass.
    glm::dvec3 m_centerOfMass{0.0, 0.0, 0.0};

    // Access to cells for partitioning
    const std::unordered_map<glm::ivec3, StructuralBlock, IVec3Hash>& getCells() const { return m_cells; }
    
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

    // Core data for block grid
    std::unordered_map<glm::ivec3, StructuralBlock, IVec3Hash> m_cells;

    // Stochastic analysis
    std::unique_ptr<StochasticAnalyzer<StructuralBlock>> m_stochasticAnalyzer;
        
    // External system references
    PhysicsEngine* m_physics;
    RigidBody* m_rigidBody{nullptr};
    std::unique_ptr<GridCollider> m_collider;

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
    
    // Internal cell access
    StructuralBlock* getCell(const glm::ivec3& coord);
    
    // Face visibility and mesh management methods
    void recalculateMassAndInertia();
    void recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& cellCoords, bool isRemoval = false);
    void updateRigidBodyInverses();
    double getApproximateRadius() const;
};