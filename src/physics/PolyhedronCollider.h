// PolyhedronCollider.h
#pragma once

#include "Collider.h"
#include "../utils/PairCache.h"
#include "../utils/HashFunctions.h"

class PolyhedronCollider : public Collider {
public:
    PolyhedronCollider(const glm::dvec3& position,
                       const glm::dquat& orientation,
                       const std::vector<glm::dvec3>& localVertices,
                       const std::vector<glm::dvec3>& localFaceAxes,
                       const std::vector<glm::dvec3>& localEdgeAxes,
                       ColliderReference* reference = nullptr);
    
    virtual ~PolyhedronCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateSimpleAABB() override;
    virtual void updateAdvancedAABB() override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    
    // Polyhedron interface (public so collision detection can access)
    std::vector<glm::dvec3> getVertices() const;
    std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> getCollisionAxes() const;
    
    // Filter normal management
    void addFilterNormal(const glm::dvec3& normal);
    void removeFilterNormal(const glm::dvec3& normal);
    void clearFilterNormals();
    const std::vector<glm::dvec3>& getFilterNormals() const;

    // Type identification
    static constexpr int TYPE_ID = hashColliderName("PolyhedronCollider");

protected:
    std::vector<glm::dvec3> m_localVertices;
    std::vector<glm::dvec3> m_localFaceAxes;
    std::vector<glm::dvec3> m_localEdgeAxes;
    double m_halfMaxWidth;

    // Protected access for derived class optimizations
    mutable std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> m_cachedCollisionAxes;
    mutable bool m_collisionAxesDirty = true;
    std::vector<glm::dvec3> m_filterNormals;
    virtual void updateCachedCollisionAxes() const;

private:
    // Cached vertices with dirty flag for lazy calculation
    mutable std::vector<glm::dvec3> m_cachedVertices;
    mutable bool m_verticesDirty = true;
    mutable bool m_advancedAABBDirty = true;
    
    void updateCachedVertices() const;
    void calculateHalfMaxWidth();
};