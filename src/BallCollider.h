// BallCollider.h
#pragma once

#include "Collider.h"

// Forward declaration
class GridCollider;

class BallCollider : public Collider {
public:
    BallCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double radius = 1.0,
                 ColliderReference* reference = nullptr);
    
    virtual ~BallCollider() = default;
    
    // Override base class methods
    virtual CollisionResult collideWith(Collider* other) override;
    virtual void updateAABB() override;
    virtual CollisionResult collideWithBall(BallCollider* ball) override;
    virtual CollisionResult collideWithGrid(GridCollider* grid) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Public member variable
    double m_radius;
};