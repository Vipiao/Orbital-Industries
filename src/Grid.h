// Grid.h
#pragma once

#include "PhysicsEngine.h"
#include "GraphicsEngine.h"
#include "GridCollider.h"
#include "HashFunctions.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>
#include <queue>

// Cell type enum
enum class CellType { ARMOR };

// Cell structure to hold face triangle IDs
struct GridCell {
    CellType type;
    std::vector<uint32_t> faceTriangleIds[6]; // Triangle IDs for each face direction
};

// Using glm::ivec3 for coordinates
using CellMap = std::unordered_map<glm::ivec3, GridCell, IVec3Hash>;

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
    CellMap m_cells;
    std::queue<glm::ivec3> m_graphicsUpdateQueue;

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
    
    // Face visibility and mesh management methods
    void recalculateMassAndInertia();
    void recalculateMassAndInertiaIncremental(const std::vector<glm::ivec3>& newCellCoords);
    void updateRigidBodyMassProperties(double totalMass, double totalMoment);
    void updateCellGraphics(const glm::ivec3& coord);
    bool isFaceVisible(const glm::ivec3& coord, int faceIndex) const;
    void queueNeighborsForUpdate(const glm::ivec3& coord);
    
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