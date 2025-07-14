// CubeCollider.h
#pragma once

#include "Collider.h"
#include "SATCacheable.h"
#include "../utils/HashFunctions.h"

class CubeCollider : public Collider, public SATCacheable {
public:
    CubeCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double width = 1.0,
                 ColliderReference* reference = nullptr);
    
    virtual ~CubeCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateTransformAndAABB() override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Public member variable
    double m_width;
    
     // Get cached vertices (public so collision detection can access)
    std::vector<glm::dvec3> getVertices() const { return m_cachedVertices; }

    // Get collision axes (face axes, edge axes) for SAT collision detection
    std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> getCollisionAxes() const;
    
    // Filter normal management
    void addFilterNormal(const glm::dvec3& normal);
    void removeFilterNormal(const glm::dvec3& normal);
    void clearFilterNormals();
    const std::vector<glm::dvec3>& getFilterNormals() const;

    // Type identification
    static constexpr int TYPE_ID = hashColliderName("CubeCollider");

private:
    
    // Helper method to update cached vertices
    void updateCachedVertices();
    
    std::vector<glm::dvec3> m_cachedVertices;

    // Filter normals in local space for collision filtering
    std::vector<glm::dvec3> m_filterNormals;
};