// PolyhedronCollider.h
#pragma once

#include "Collider.h"
#include "utils/PairCache.h"
#include "utils/HashFunctions.h"

class PolyhedronCollider : public Collider {
public:
    PolyhedronCollider(const glm::dvec3& position,
                       const glm::dquat& orientation,
                       const std::vector<glm::dvec3>& localVertices,
                       const std::vector<glm::dvec3>& localFaceAxes,
                       const std::vector<glm::dvec3>& localEdgeAxes,
                       const std::vector<std::array<int, 2>>& localEdges);
    
    virtual ~PolyhedronCollider() = default;
    
    // Override base class methods
    virtual int getTypeId() const override { return TYPE_ID; }
    virtual void updateSimpleAABB(uint64_t currentTimestep) override;
    virtual void updateAdvancedAABB(uint64_t currentTimestep) override;
    virtual bool checkAABBCollision(const Collider* other) const override;
    virtual RayIntersectionResult intersectRay(const glm::dvec3& rayStart, const glm::dvec3& rayEnd) const override;
    
    // Polyhedron interface (public so collision detection can access)
    std::vector<glm::dvec3> getVertices(uint64_t currentTimestep = 0) const;
    std::vector<glm::dvec3> getLocalVertices() const;
    /**
     * @brief Generate triangle indices for the polyhedron's surface faces
     * @note This method caches results and only recalculates when geometry changes
     * @return Vector of triangle indices based on local vertices and face projections
     */
    std::vector<std::array<int, 3>> generateTriangleIndices() const;
    std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> getCollisionAxes(uint64_t currentTimestep = 0) const;
    
    // Edge connectivity access
    const std::vector<std::array<int, 2>>& getEdgeIndices() const { return m_localEdges; }
    int getEdgeCount() const { return static_cast<int>(m_localEdges.size()); }

    // Get half of the maximum dimension
    double getHalfMaxWidth() const { return m_halfMaxWidth; }

    // Filter normal management
    void addFilterNormal(const glm::dvec3& normal);
    void removeFilterNormal(const glm::dvec3& normal);
    void clearFilterNormals();
    const std::vector<glm::dvec3>& getFilterNormals() const;

    // Type identification
    static constexpr int TYPE_ID = Hash::hashName("PolyhedronCollider");

protected:
    std::vector<glm::dvec3> m_localVertices;
    std::vector<glm::dvec3> m_localFaceAxes;
    std::vector<glm::dvec3> m_localEdgeAxes;
    std::vector<std::array<int, 2>> m_localEdges;
    double m_halfMaxWidth;

    // Protected access for derived class optimizations
    mutable std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> m_cachedCollisionAxes;
    mutable uint64_t m_collisionAxesValidUntilTime = 0;
    std::vector<glm::dvec3> m_filterNormals;
    virtual void updateCachedCollisionAxes() const;

    // Cached triangle indices with dirty flag
    mutable std::vector<std::array<int, 3>> m_cachedTriangleIndices;
    mutable bool m_triangleIndicesDirty = true;

private:
    // Cached vertices with dirty flag for lazy calculation
    mutable std::vector<glm::dvec3> m_cachedVertices;
    mutable uint64_t m_verticesValidUntilTime = 0;
    
    void updateCachedVertices() const;
    void calculateHalfMaxWidth();
};