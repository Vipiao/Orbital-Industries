// Grid.h
#pragma once

#include "PhysicsEngine.h"
#include "GraphicsEngine.h"
#include "GridCollider.h"
#include "MassInertiaCalculator.h"
#include "HashFunctions.h"
#include "StochasticAnalyzer.h"
#include "TimeHandler.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>
#include <queue>

// Cell type enum
enum class CellType { ARMOR };

// Forward declaration
class Grid;

// Cell structure to hold face triangle IDs
struct GridCell : public IStochasticCell {
    CellType type;
    std::vector<uint32_t> faceTriangleIds[6]; // Triangle IDs for each face direction
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
    void updateGraphics();
    void processGraphicsQueue();
    
    // Getters for GameBase
    PhysicsEngine::RigidBody* getRigidBody() const { return m_rigidBody; }
    bool hasGraphicsUpdates() const { return !m_graphicsUpdateQueue.empty(); }

    // Convert world coordinates to grid-local coordinates
    glm::dvec3 worldToGrid(const glm::dvec3& worldPos) const;
    
    // Convert grid-local coordinates to world coordinates
    glm::dvec3 gridToWorld(const glm::dvec3& gridPos) const;

    std::vector<glm::ivec3> gridTraversal(glm::dvec3 startPos, glm::dvec3 endPos);
    
    glm::dvec3 m_centerOfMass{0.0, 0.0, 0.0};
    
private:
    // Core data for block grid
    std::unordered_map<glm::ivec3, GridCell, IVec3Hash> m_cells;
    std::queue<glm::ivec3> m_graphicsUpdateQueue;

    // Stochastic analysis
    std::unique_ptr<StochasticAnalyzer<GridCell>> m_stochasticAnalyzer;

    // Armor texture.
    static int s_colorTextureUnit;
    static int s_normalTextureUnit;
    static bool s_texturesLoaded;
        
    // External system references
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    PhysicsEngine::RigidBody* m_rigidBody{nullptr};
    std::unique_ptr<GridCollider> m_collider;
    int m_meshId{-1};

    // GPU state tracking for optimization (mutable for const function caching)
    mutable glm::dvec3 m_lastSentRigidBodyPosition{0.0};
    mutable glm::dquat m_lastSentRigidBodyOrientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_lastSentRigidBodyVelocity{0.0};
    glm::dquat m_lastSentRigidBodyAngularVelocityQuat{1.0, 0.0, 0.0, 0.0}; // Quaternion for one physics step
    uint64_t m_nextUpdatePhysicsTimeStep{0}; // When the next update should happen
    mutable uint64_t m_lastCheckedPhysicsTimeStep{0}; // Last time step we checked in shouldUpdateGPU
    
    // Update thresholds
    static constexpr double POSITION_THRESHOLD = 0.002; // 5mm
    static constexpr double ORIENTATION_THRESHOLD_BASE = 0.002; // Base threshold for radius = 1.0
    static constexpr uint64_t TIME_THRESHOLD = 256; // Update at least every 256 physics steps (~8 seconds)
    
    // Face visibility and mesh management methods
    void recalculateMassAndInertia();
    void recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& cellCoords, bool isRemoval = false);
    void updateRigidBodyInverses();
    void updateCellGraphics(const glm::ivec3& coord);
    bool isFaceVisible(const glm::ivec3& coord, int faceIndex) const;
    void queueNeighborsForUpdate(const glm::ivec3& coord);
    bool shouldUpdateGPU() const;
    double getApproximateRadius() const;

    // Static face mesh data
    static std::vector<AssetMeshData> s_faceMeshData;
    static bool s_faceMeshDataLoaded;
    static void loadFaceMeshData();
    
    // Face transform lookup table
    struct FaceTransform {
        glm::dvec3 axis;
        double angle;
    };
    static const FaceTransform s_faceTransforms[6];
    static glm::dmat4 getFaceTransform(int faceIndex, const glm::ivec3& coord);
};