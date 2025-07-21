// Grid.h
#pragma once

#include "../physics/RigidBody.h"
#include "../physics/GridCollider.h"
#include "MassInertiaCalculator.h"
#include "../utils/HashFunctions.h"
#include "../utils/StochasticAnalyzer.h"
#include "GridGraphics.h"
#include "GridGeometry.h"
#include "CellType.h"
#include "../utils/TimeHandler.h"
#include <glm/glm.hpp>
#include "../utils/JobManager.h"
#include <memory>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>

// Forward declarations
class PhysicsEngine;

// Forward declaration
class Grid;

// Cell structure to hold face triangle IDs
struct GridCell : public IStochasticCell {
    CellType type;
    glm::ivec3 coordinates; // Store coordinates for this cell
    Grid* parentGrid; // Reference to parent grid for neighbor lookup
    double structuralWeakness = -1.0; // Running average structural weakness (-1 = no data yet)
    
    // Direct neighbor pointers for fast access (Right, Left, Front, Back, Top, Bottom)
    std::array<GridCell*, 6> neighbors{nullptr};
    
    GridCell(const glm::ivec3& coords, Grid* parent, CellType cellType = CellType::ARMOR) 
        : coordinates(coords), parentGrid(parent), type(cellType) {}
    
    // Implement IStochasticCell interface
    virtual void forEachConnectedNeighbor(std::function<void(const glm::ivec3&)> callback) const override;
};

class Grid {
public:
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
    const std::unordered_map<glm::ivec3, GridCell, IVec3Hash>& getCells() const { return m_cells; }
    
private:
    // Job management
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;
    std::weak_ptr<Job> m_analysisJob;
    
    // Analysis state
    int m_currentAnalysisIteration = 0;
    static constexpr int MAX_ANALYSIS_ITERATIONS = 8;
    static constexpr double WEAKNESS_BLEND_FACTOR = 0.2; // New result weight in running average
    
    // Core data for block grid
    std::unordered_map<glm::ivec3, GridCell, IVec3Hash> m_cells;

    // Stochastic analysis
    std::unique_ptr<StochasticAnalyzer<GridCell>> m_stochasticAnalyzer;
        
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

    // Neighbor connection management
    void updateNeighborConnections(const glm::ivec3& coord);
    void removeNeighborConnections(const glm::ivec3& coord);
    
    // Internal cell access
    GridCell* getCell(const glm::ivec3& coord);
    
    // Face visibility and mesh management methods
    void recalculateMassAndInertia();
    void recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& cellCoords, bool isRemoval = false);
    void updateRigidBodyInverses();
    double getApproximateRadius() const;
};