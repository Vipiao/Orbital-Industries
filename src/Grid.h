// Grid.h - Updated
#pragma once

#include "PhysicsEngine.h"
#include "GraphicsEngine.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <unordered_map>
#include <queue>

// New - Custom hash function for glm::ivec3
struct IVec3Hash {
    size_t operator()(const glm::ivec3& coord) const {
        // XOR coordinates then apply basic hash
        size_t hash = static_cast<size_t>(coord.x) ^ 
                      static_cast<size_t>(coord.y) ^ 
                      static_cast<size_t>(coord.z);
        hash = hash * 73856093;
        return hash;
    }
};

// New - Cell type enum
enum class CellType { ARMOR };

// New - Cell structure to hold face triangle IDs
struct GridCell {
    CellType type;
    std::vector<uint32_t> faceTriangleIds[6]; // Triangle IDs for each face direction
};

// New - Using glm::ivec3 for coordinates
using CellMap = std::unordered_map<glm::ivec3, GridCell, IVec3Hash>;

class Grid {
public:
    // Updated - Constructor now takes physics and graphics pointers
    Grid(PhysicsEngine* physics, GraphicsEngine* graphics, 
         const glm::dvec3& position, 
         const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0));
    ~Grid();
    
    // New - Cell management methods
    void addCell(const glm::ivec3& coord, CellType type = CellType::ARMOR);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    
    // Updated - Split graphics update method
    void updateGraphics();
    void processGraphicsQueue();
    
    // New - Getters for GameBase
    int getRigidBodyId() const { return m_rigidBodyId; }
    bool hasGraphicsUpdates() const { return !m_graphicsUpdateQueue.empty(); }
    
    // New - Physics and rendering properties
    glm::dvec3 m_centerOfMass{0.0, 0.0, 0.0};
    
private:
    // New - Core data for block grid
    CellMap m_cells;
    std::queue<glm::ivec3> m_graphicsUpdateQueue;
    
    // Updated - External system references
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    int m_rigidBodyId{-1};
    int m_meshId{-1};
    int m_colorTextureUnit{-1};
    int m_normalTextureUnit{-1};
    
    // New - Face visibility and mesh management methods
    void recalculateCenterOfMass();
    void updateCellGraphics(const glm::ivec3& coord);
    bool isFaceVisible(const glm::ivec3& coord, int faceIndex) const;
    void queueNeighborsForUpdate(const glm::ivec3& coord);
    
    // New - Static face mesh data
    static std::vector<AssetMeshData> s_faceMeshData;
    static bool s_faceMeshDataLoaded;
    static void loadFaceMeshData();
    
    // New - Face transform lookup table
    struct FaceTransform {
        glm::dvec3 axis;
        double angle;
    };
    static const FaceTransform s_faceTransforms[6];
    static glm::dmat4 getFaceTransform(int faceIndex, const glm::ivec3& coord);
};