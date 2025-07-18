// CubeCollider.h
#pragma once

#include "Collider.h"
#include "../utils/PairCache.h"
#include "../utils/HashFunctions.h"

class CubeCollider : public Collider {
public:
    CubeCollider(const glm::dvec3& position = glm::dvec3(0.0),
                 const glm::dquat& orientation = glm::dquat(1.0, 0.0, 0.0, 0.0),
                 double width = 1.0,
                 ColliderReference* reference = nullptr);
    
    virtual ~CubeCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateSimpleAABB() override;
    virtual void updateAdvancedAABB() override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Public member variable
    double m_width;
    
    // Get cached vertices (public so collision detection can access)
    std::vector<glm::dvec3> getVertices() const;

    // Get collision axes (face axes, edge axes) for SAT collision detection
    std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> getCollisionAxes() const;
    
    // Filter normal management
    void addFilterNormal(const glm::dvec3& normal);
    void removeFilterNormal(const glm::dvec3& normal);
    void clearFilterNormals();
    const std::vector<glm::dvec3>& getFilterNormals() const;

    // SAT axis caching convenience methods
    bool getCachedAxis(const CubeCollider* other, glm::dvec3& axis) const {
        return PairCache<glm::dvec3>::getCachedData(this, other, axis);
    }
    void setCachedAxis(const CubeCollider* other, const glm::dvec3& axis) {
        PairCache<glm::dvec3>::setCachedData(this, other, axis);
    }

    // Type identification
    static constexpr int TYPE_ID = hashColliderName("CubeCollider");

private:
    
    // Cached vertices with dirty flag for lazy calculation
    mutable std::vector<glm::dvec3> m_cachedVertices;
    mutable bool m_verticesDirty = true;
    mutable bool m_advancedAABBDirty = true;
    mutable std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> m_cachedCollisionAxes;
    mutable bool m_collisionAxesDirty = true;
    
    void updateCachedVertices() const;
    void updateCachedCollisionAxes() const;

    // Filter normals in local space for collision filtering
    std::vector<glm::dvec3> m_filterNormals;
};