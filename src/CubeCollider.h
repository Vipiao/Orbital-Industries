// CubeCollider.h
#pragma once

#include "Collider.h"

// Forward declarations
class BallCollider;
class GridCollider;

class CubeCollider : public Collider {
public:
    CubeCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double width = 1.0,
                 ColliderReference* reference = nullptr);
    
    virtual ~CubeCollider() = default;
    
    // Override base class methods
    virtual CollisionResult collideWith(Collider* other) override;
    virtual void updateAABB() override;
    virtual CollisionResult collideWithBall(BallCollider* ball) override;
    virtual CollisionResult collideWithCube(CubeCollider* cube) override;
    virtual CollisionResult collideWithGrid(GridCollider* grid) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Public member variable
    double m_width;
    
private:
    // Helper method to get cube vertices
    std::vector<glm::dvec3> getVertices() const;
};