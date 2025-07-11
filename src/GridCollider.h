// GridCollider.h
#pragma once

#include "Collider.h"
#include "CubeCollider.h"
#include "HashFunctions.h"
#include <glm/glm.hpp>
#include <unordered_map>
#include <memory>

class GridCollider : public Collider {
public:
    GridCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 ColliderReference* reference = nullptr);
    
    virtual ~GridCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateTransformAndAABB() override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Grid-specific methods
    void addCell(const glm::ivec3& coord, double width = 1.0);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    
    // Get sub-collider for a specific cell
    CubeCollider* getCell(const glm::ivec3& coord);

    // Allow access to cells for collision detection utils
    const std::unordered_map<glm::ivec3, std::unique_ptr<CubeCollider>, IVec3Hash>& getCells() const { return m_cells; }
    
    glm::dvec3 gridToWorld(const glm::dvec3& gridCoord) const;
    glm::dvec3 worldToGrid(const glm::dvec3& worldCoord) const;

    // Type identification  
    static constexpr int TYPE_ID = hashColliderName("GridCollider");

private:
    // Map of grid coordinates to cube colliders
    std::unordered_map<glm::ivec3, std::unique_ptr<CubeCollider>, IVec3Hash> m_cells;
    
    // Helper methods
    void updateSubColliderTransformsAndAABB();

    // Filter normal management
    void updateFilterNormalsForCell(const glm::ivec3& coord);
    void updateFilterNormalsAfterRemoval(const glm::ivec3& removedCoord);
};