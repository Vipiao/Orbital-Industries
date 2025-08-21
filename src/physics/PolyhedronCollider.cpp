// PolyhedronCollider.cpp
#include "PolyhedronCollider.h"
#include <glm/gtx/transform.hpp>
#include <algorithm>
#include <stdexcept>
#include "../utils/GeometryUtils.h"

PolyhedronCollider::PolyhedronCollider(const glm::dvec3& position,
                                       const glm::dquat& orientation,
                                       const std::vector<glm::dvec3>& localVertices,
                                       const std::vector<glm::dvec3>& localFaceAxes,
                                       const std::vector<glm::dvec3>& localEdgeAxes,
                                       ColliderReference* reference)
    : Collider(position, orientation, reference)
    , m_localVertices(localVertices)
    , m_localFaceAxes(localFaceAxes)
    , m_localEdgeAxes(localEdgeAxes)
{
    calculateHalfMaxWidth();
}

void PolyhedronCollider::calculateHalfMaxWidth() {
    if (m_localVertices.empty()) {
        m_halfMaxWidth = 0.5; // Default fallback
        return;
    }
    
    // Find bounding box of local vertices
    glm::dvec3 minVertex = m_localVertices[0];
    glm::dvec3 maxVertex = m_localVertices[0];
    
    for (const glm::dvec3& vertex : m_localVertices) {
        minVertex = glm::min(minVertex, vertex);
        maxVertex = glm::max(maxVertex, vertex);
    }
    
    // Calculate dimensions
    glm::dvec3 dimensions = maxVertex - minVertex;
    double maxDimension = glm::max(glm::max(dimensions.x, dimensions.y), dimensions.z);
    
    m_halfMaxWidth = maxDimension * 0.5;
}

void PolyhedronCollider::updateSimpleAABB(uint64_t currentTimestep) {
    // Check if simple AABB is still valid
    if (currentTimestep <= m_simpleAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    // Simple AABB using half max width (conservative sphere)
    double halfDiagonal = m_halfMaxWidth * std::sqrt(3.0);
    m_AABBMin = m_position - glm::dvec3(halfDiagonal);
    m_AABBMax = m_position + glm::dvec3(halfDiagonal);

    // Mark simple AABB as valid for this timestep
    m_simpleAABBValidUntilTime = currentTimestep;

    // Invalidate cached data since position/orientation may have changed
    m_verticesValidUntilTime = 0;
    m_collisionAxesValidUntilTime = 0;
}

void PolyhedronCollider::updateAdvancedAABB(uint64_t currentTimestep) {
    // Check if advanced AABB is still valid
    if (currentTimestep <= m_advancedAABBValidUntilTime) {
        return; // Still valid, no need to recalculate
    }

    std::vector<glm::dvec3> vertices = getVertices(currentTimestep);
    
    if (vertices.empty()) {
        throw std::runtime_error("PolyhedronCollider::updateAdvancedAABB: No vertices available for AABB calculation");
    }
    
    // Initialize with first vertex
    m_AABBMin = vertices[0];
    m_AABBMax = vertices[0];
    
    // Expand to include all vertices
    for (size_t i = 1; i < vertices.size(); ++i) {
        m_AABBMin = glm::min(m_AABBMin, vertices[i]);
        m_AABBMax = glm::max(m_AABBMax, vertices[i]);
    }

    // Mark advanced AABB as valid for this timestep
    m_advancedAABBValidUntilTime = currentTimestep;
}

bool PolyhedronCollider::checkAABBCollision(const Collider* other) const {
    // Simple AABB overlap test
    return (m_AABBMin.x <= other->m_AABBMax.x && 
            m_AABBMax.x >= other->m_AABBMin.x &&
            m_AABBMin.y <= other->m_AABBMax.y && 
            m_AABBMax.y >= other->m_AABBMin.y &&
            m_AABBMin.z <= other->m_AABBMax.z && 
            m_AABBMax.z >= other->m_AABBMin.z);
}

std::vector<glm::dvec3> PolyhedronCollider::getVertices(uint64_t currentTimestep) const {
    // Lazy calculation - only recalculate if invalid
    if (currentTimestep > m_verticesValidUntilTime) {
        updateCachedVertices();
        m_verticesValidUntilTime = currentTimestep;
    }
    return m_cachedVertices;
}

std::vector<glm::dvec3> PolyhedronCollider::getLocalVertices() const {
    return m_localVertices;
}

std::vector<std::array<int, 3>> PolyhedronCollider::generateTriangleIndices() const {
    std::vector<std::array<int, 3>> triangles;
    
    if (m_localVertices.size() < 4) {
        return triangles; // Need at least 4 vertices for a polyhedron
    }
    
    const double margin = 1e-6;
    
    // For each face axis direction
    for (const auto& axis : m_localFaceAxes) {
        // Project all vertices onto this axis
        std::vector<double> projections;
        projections.reserve(m_localVertices.size());
        
        for (const auto& vertex : m_localVertices) {
            projections.push_back(glm::dot(vertex, axis));
        }
        
        // Find min and max projections
        double minProj = *std::min_element(projections.begin(), projections.end());
        double maxProj = *std::max_element(projections.begin(), projections.end());
        
        double extremeProjs[2] = {minProj, maxProj};

        // Check both extremes (min and max)
        for (int extreme = 0; extreme < 2; ++extreme) {
            double targetProj = extremeProjs[extreme];
            
            // Find vertices at this extreme
            std::vector<int> extremeVertexIndices;
            std::vector<glm::dvec3> extremeVertices;
            
            for (size_t i = 0; i < m_localVertices.size(); ++i) {
                if (std::abs(projections[i] - targetProj) <= margin) {
                    extremeVertexIndices.push_back(static_cast<int>(i));
                    extremeVertices.push_back(m_localVertices[i]);
                }
            }
            
            // Need at least 3 vertices to form triangles
            if (extremeVertices.size() < 3) {
                continue;
            }
            
            // Create plane transform for this face
            glm::dvec3 planeNormal = axis;
            glm::dvec3 planePoint = extremeVertices[0]; // Use first vertex as reference point
            auto transform = GeometryUtils::createPlaneTransform(planeNormal);
            
            // Project vertices to 2D
            std::vector<glm::dvec2> vertices2D = GeometryUtils::projectToPlane(extremeVertices, transform);
            
            // Wind the 2D points
            std::vector<glm::dvec2> windedVertices2D = GeometryUtils::windPoints(vertices2D);
            
            // Triangulate the winded polygon (fan triangulation from first vertex)
            for (size_t i = 1; i < windedVertices2D.size() - 1; ++i) {
                // Find original vertex indices for the winded vertices
                int idx0 = extremeVertexIndices[0]; // First vertex
                int idx1 = -1, idx2 = -1;
                
                // Find indices for vertices i and i+1 in winded order
                for (size_t j = 0; j < vertices2D.size(); ++j) {
                    if (glm::length(vertices2D[j] - windedVertices2D[i]) < margin) {
                        idx1 = extremeVertexIndices[j];
                    }
                    if (glm::length(vertices2D[j] - windedVertices2D[i + 1]) < margin) {
                        idx2 = extremeVertexIndices[j];
                    }
                }
                
                if (idx1 >= 0 && idx2 >= 0) {
                    triangles.push_back({idx0, idx1, idx2});
                }
            }
        }
    }
    
    return triangles;
}

void PolyhedronCollider::updateCachedVertices() const {
    // Pre-allocate vertices array
    if (m_cachedVertices.size() != m_localVertices.size()) {
        m_cachedVertices.resize(m_localVertices.size());
    }
    
    for (size_t i = 0; i < m_localVertices.size(); ++i) {
        m_cachedVertices[i] = m_position + m_orientation * m_localVertices[i];
    }
}

std::tuple<std::vector<glm::dvec3>, std::vector<glm::dvec3>, std::vector<glm::dvec3>> PolyhedronCollider::getCollisionAxes(uint64_t currentTimestep) const {
    // Lazy calculation - only recalculate if invalid
    if (currentTimestep > m_collisionAxesValidUntilTime) {
        updateCachedCollisionAxes();
        m_collisionAxesValidUntilTime = currentTimestep;
    }
    return m_cachedCollisionAxes;
}

void PolyhedronCollider::updateCachedCollisionAxes() const {
    // Transform face axes to world space
    std::vector<glm::dvec3> worldFaceAxes;
    worldFaceAxes.reserve(m_localFaceAxes.size());
    for (const glm::dvec3& faceAxis : m_localFaceAxes) {
        worldFaceAxes.push_back(m_orientation * faceAxis);
    }
    
    // Transform edge axes to world space
    std::vector<glm::dvec3> worldEdgeAxes;
    worldEdgeAxes.reserve(m_localEdgeAxes.size());
    for (const glm::dvec3& edgeAxis : m_localEdgeAxes) {
        worldEdgeAxes.push_back(m_orientation * edgeAxis);
    }
    
    // Transform filter normals to world space
    std::vector<glm::dvec3> worldFilterNormals;
    worldFilterNormals.reserve(m_filterNormals.size());
    for (const glm::dvec3& filterNormal : m_filterNormals) {
        worldFilterNormals.push_back(m_orientation * filterNormal);
    }
    
    m_cachedCollisionAxes = {worldFaceAxes, worldEdgeAxes, worldFilterNormals};
}

void PolyhedronCollider::addFilterNormal(const glm::dvec3& normal) {
    // Check if this normal already exists (within tolerance)
    const double tolerance = 0.99; // Very close to parallel
    for (const glm::dvec3& existing : m_filterNormals) {
        if (glm::dot(existing, normal) > tolerance) {
            return; // Already exists, don't add duplicate
        }
    }
    
    // Add normalized filter normal
    glm::dvec3 normalizedNormal = glm::normalize(normal);
    m_filterNormals.push_back(normalizedNormal);

    m_collisionAxesValidUntilTime = 0; // Invalidate collision axes
}

void PolyhedronCollider::removeFilterNormal(const glm::dvec3& normal) {
    const double tolerance = 0.99; // Very close to parallel
    glm::dvec3 normalizedNormal = glm::normalize(normal);
    
    auto it = std::remove_if(m_filterNormals.begin(), m_filterNormals.end(),
        [&](const glm::dvec3& existing) {
            return glm::dot(existing, normalizedNormal) > tolerance;
        });
    
    m_filterNormals.erase(it, m_filterNormals.end());

    m_collisionAxesValidUntilTime = 0; // Invalidate collision axes
}

void PolyhedronCollider::clearFilterNormals() {
    m_filterNormals.clear();

    m_collisionAxesValidUntilTime = 0; // Invalidate collision axes
}

const std::vector<glm::dvec3>& PolyhedronCollider::getFilterNormals() const {
    return m_filterNormals;
}