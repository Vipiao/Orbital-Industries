// GridCollider.h
#pragma once

#include "Collider.h"
#include "BallCollider.h"
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
    virtual CollisionResult collideWith(Collider* other) override;
    virtual void updateAABB() override;
    virtual CollisionResult collideWithBall(BallCollider* ball) override;
    virtual CollisionResult collideWithGrid(GridCollider* grid) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Grid-specific methods
    void addCell(const glm::ivec3& coord, double radius = 0.5);
    void removeCell(const glm::ivec3& coord);
    bool hasCell(const glm::ivec3& coord) const;
    
    // Get sub-collider for a specific cell
    BallCollider* getCell(const glm::ivec3& coord);
    
private:
    // Map of grid coordinates to ball colliders
    std::unordered_map<glm::ivec3, std::unique_ptr<BallCollider>, IVec3Hash> m_cells;
    
    // Helper methods
    void updateSubColliderTransformsAndAABB();
    glm::dvec3 gridToWorld(const glm::dvec3& gridCoord) const;
};