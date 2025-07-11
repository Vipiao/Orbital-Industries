// Grid.h
#pragma once

#include "PhysicsEngine.h"
#include "GridCollider.h"
#include "MassInertiaCalculator.h"
#include "HashFunctions.h"
#include "StochasticAnalyzer.h"
#include "GridGraphics.h"
#include "GridGeometry.h"
#include "CellType.h"
#include "TimeHandler.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>
#include <queue>

// Forward declaration
class Grid;

// Cell structure to hold face triangle IDs
struct GridCell : public IStochasticCell {
    CellType type;
    glm::ivec3 coordinates; // Store coordinates for this cell
    Grid* parentGrid; // Reference to parent grid for neighbor lookup
    double structuralWeakness = 0.0; // Accumulated structural weakness across multiple analysis iterations
    
    GridCell(const glm::ivec3& coords, Grid* parent, CellType cellType = CellType::ARMOR) 
        : coordinates(coords), parentGrid(parent), type(cellType) {}
    
    // Implement IStochasticCell interface
    virtual std::vector<glm::ivec3> getConnectedNeighbors() const override;
    virtual bool isValidForPath() const override { return true; } // All grid cells are valid for pathfinding
};

// Forward declaration
class TimeHandler;

class Grid {
public:
    // Constructor now takes physics and graphics pointers
    Grid(PhysicsEngine* physics, GraphicsEngine* graphics,
         const glm::dvec3& position, 
         const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    ~Grid();
    
    // Cell management methods
    void addCell(const glm::ivec3& coord, CellType type = CellType::ARMOR);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    bool isEmpty() const;

    // Structural analysis
    void analyzeStructuralIntegrity(TimeHandler* timeHandler);
    
    // Split graphics update method
    void updateGraphics(const glm::dvec3& cameraPos);
    void processGraphicsQueue();
    
    // Getters for GameBase
    PhysicsEngine::RigidBody* getRigidBody() const { return m_rigidBody; }
    bool hasGraphicsUpdates() const;

    // Convert world coordinates to grid-local coordinates
    glm::dvec3 worldToGrid(const glm::dvec3& worldPos) const;
    
    // Convert grid-local coordinates to world coordinates
    glm::dvec3 gridToWorld(const glm::dvec3& gridPos) const;

    std::vector<glm::ivec3> gridTraversal(glm::dvec3 startPos, glm::dvec3 endPos);
    
    glm::dvec3 m_centerOfMass{0.0, 0.0, 0.0};
    
private:
    // Core data for block grid
    std::unordered_map<glm::ivec3, GridCell, IVec3Hash> m_cells;

    // Stochastic analysis
    std::unique_ptr<StochasticAnalyzer<GridCell>> m_stochasticAnalyzer;
        
    // External system references
    PhysicsEngine* m_physics;
    PhysicsEngine::RigidBody* m_rigidBody{nullptr};
    std::unique_ptr<GridCollider> m_collider;

    // Graphics subsystem
    std::unique_ptr<GridGraphics> m_gridGraphics;
    
    // Face visibility and mesh management methods
    void recalculateMassAndInertia();
    void recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& cellCoords, bool isRemoval = false);
    void updateRigidBodyInverses();
    double getApproximateRadius() const;
};