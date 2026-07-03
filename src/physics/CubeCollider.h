// CubeCollider.h
#pragma once

#include "PolyhedronCollider.h"
#include "utils/HashFunctions.h"

class CubeCollider : public PolyhedronCollider {
public:
    CubeCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double width = 1.0);
    
    virtual ~CubeCollider() = default;
    
    // Override type identification
    virtual int getTypeId() const override { return TYPE_ID; }
    
    // Type identification
    static constexpr int TYPE_ID = Hash::hashColliderName("CubeCollider");

private:

protected:
    virtual void updateCachedCollisionAxes() const override;
};