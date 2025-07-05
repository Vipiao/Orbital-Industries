// BallCollider.h
#pragma once

#include "Collider.h"
#include "HashFunctions.h"

class BallCollider : public Collider {
public:
    BallCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double radius = 1.0,
                 ColliderReference* reference = nullptr);
    
    virtual ~BallCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateTransformAndAABB() override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Public member variable
    double m_radius;

    // Type identification
    static constexpr int TYPE_ID = hashColliderName("BallCollider");
};