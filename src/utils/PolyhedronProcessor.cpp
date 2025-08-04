#include "PolyhedronProcessor.h"
#include <glm/gtx/norm.hpp>
#include <cmath>
#include <algorithm>

bool IVec3Compare::operator()(const glm::ivec3& a, const glm::ivec3& b) const {
    if (a.x != b.x) return a.x < b.x;
    if (a.y != b.y) return a.y < b.y;
    return a.z < b.z;
}

bool Vec3Compare::operator()(const glm::dvec3& a, const glm::dvec3& b) const {
    if (std::abs(a.x - b.x) > eps) return a.x < b.x;
    if (std::abs(a.y - b.y) > eps) return a.y < b.y;
    return a.z < b.z - eps;
}

std::array<std::array<int, 3>, 2> PolyhedronProcessor::getConvexTriangulation(
    const glm::ivec3& v0, const glm::ivec3& v1, const glm::ivec3& v2, const glm::ivec3& v3,
    int idx0, int idx1, int idx2, int idx3) {
    
    // Calculate normal of triangle (v0, v1, v2)
    glm::ivec3 edge1 = v1 - v0;
    glm::ivec3 edge2 = v2 - v0;
    glm::ivec3 normal = glm::cross(edge1, edge2);
    
    // Calculate vector from v0 to v3
    glm::ivec3 vec_to_v3 = v3 - v0;
    
    // Check if quad is convex by testing if v3 is on the same side as the normal
    int dot_product = glm::dot(normal, vec_to_v3);
    
    if (dot_product > 0) {
        // Quad is concave, use alternative triangulation
        return {{
            {idx0, idx1, idx3},
            {idx3, idx1, idx2}
        }};
    } else {
        // Quad is convex, use standard triangulation
        return {{
            {idx0, idx1, idx2},
            {idx0, idx2, idx3}
        }};
    }
}

PolyhedronProcessor::AxisResult PolyhedronProcessor::getAxis(const std::vector<glm::ivec3>& vertices, int maxSize) {
    if (vertices.size() != 8) {
        return {}; // Invalid input - need exactly 8 vertices for a cube
    }
    
    AxisResult result;
    
    // Process each face of the cube
    for (const auto& face : CUBE_FACES) {
        // Get the 4 vertices of this face (ordered counter-clockwise from outside)
        glm::ivec3 v0 = vertices[face[0]];
        glm::ivec3 v1 = vertices[face[1]];
        glm::ivec3 v2 = vertices[face[2]];
        glm::ivec3 v3 = vertices[face[3]];
        
        // Get convex triangulation
        auto triangulation = getConvexTriangulation(v0, v1, v2, v3, face[0], face[1], face[2], face[3]);
        
        // Process both triangles
        for (const auto& triangle : triangulation) {
            glm::ivec3 t_v0 = vertices[triangle[0]];
            glm::ivec3 t_v1 = vertices[triangle[1]];
            glm::ivec3 t_v2 = vertices[triangle[2]];
            
            glm::ivec3 t_edge1 = t_v1 - t_v0;
            glm::ivec3 t_edge2 = t_v2 - t_v0;
            glm::ivec3 t_normal = glm::cross(t_edge1, t_edge2);
            
            if (glm::length2(t_normal) > 0) {
                glm::dvec3 normalized = glm::normalize(glm::dvec3(t_normal));
                result.faceAxis.insert(normalized);
            }
        }
    }
    
    // Process each edge of the cube
    for (const auto& edge : CUBE_EDGES) {
        glm::ivec3 edgeVec = vertices[edge[1]] - vertices[edge[0]];
        
        if (glm::length2(edgeVec) > 0) {
            glm::dvec3 normalized = glm::normalize(glm::dvec3(edgeVec));
            result.edgeAxis.insert(normalized);
        }
    }
    
    return result;
}

std::vector<glm::dvec3> PolyhedronProcessor::getUniqueVertices(const std::vector<glm::ivec3>& vertices, int maxSize) {
    std::set<glm::ivec3, IVec3Compare> uniqueSet;
    
    // First find unique ivec3 vertices
    for (const auto& v : vertices) {
        uniqueSet.insert(v);
    }
    
    // Then convert to normalized dvec3
    std::vector<glm::dvec3> result;
    result.reserve(uniqueSet.size());
    
    for (const auto& v : uniqueSet) {
        glm::dvec3 normalized = glm::dvec3(v) / double(maxSize);
        result.push_back(normalized);
    }
    
    return result;
}

std::vector<std::array<glm::dvec3, 3>> PolyhedronProcessor::getTriangles(const std::vector<glm::ivec3>& vertices, int maxSize) {
    if (vertices.size() != 8) {
        return {}; // Invalid input
    }
    
    std::vector<std::array<glm::dvec3, 3>> triangles;
    
    // Process each face and create triangles
    for (const auto& face : CUBE_FACES) {
        // Get the 4 vertices of this face
        glm::ivec3 v0 = vertices[face[0]];
        glm::ivec3 v1 = vertices[face[1]];
        glm::ivec3 v2 = vertices[face[2]];
        glm::ivec3 v3 = vertices[face[3]];
        
        // Get convex triangulation
        auto triangulation = getConvexTriangulation(v0, v1, v2, v3, face[0], face[1], face[2], face[3]);
        
        // Process both triangles
        for (const auto& triangle : triangulation) {
            glm::ivec3 t_v0 = vertices[triangle[0]];
            glm::ivec3 t_v1 = vertices[triangle[1]];
            glm::ivec3 t_v2 = vertices[triangle[2]];
            
            glm::ivec3 t_cross = glm::cross(t_v1 - t_v0, t_v2 - t_v0);
            double area = 0.5 * glm::length(glm::dvec3(t_cross));
            
            if (area > Vec3Compare::eps) {
                glm::dvec3 dv0 = glm::dvec3(t_v0) / double(maxSize);
                glm::dvec3 dv1 = glm::dvec3(t_v1) / double(maxSize);
                glm::dvec3 dv2 = glm::dvec3(t_v2) / double(maxSize);
                triangles.push_back({dv0, dv1, dv2});
            }
        }
    }
    
    return triangles;
}

glm::dvec3 PolyhedronProcessor::getTriangleCenter(const std::array<glm::dvec3, 3>& triangle) {
    return (triangle[0] + triangle[1] + triangle[2]) / 3.0;
}

glm::dvec3 PolyhedronProcessor::getTriangleNormal(const std::array<glm::dvec3, 3>& triangle) {
    glm::dvec3 edge1 = triangle[1] - triangle[0];
    glm::dvec3 edge2 = triangle[2] - triangle[0];
    glm::dvec3 normal = glm::cross(edge1, edge2);
    
    if (glm::length(normal) > Vec3Compare::eps) {
        return glm::normalize(normal);
    }
    return glm::dvec3(0.0, 0.0, 0.0); // Degenerate triangle
}

int PolyhedronProcessor::countSharedVertices(const std::array<glm::dvec3, 3>& t1, const std::array<glm::dvec3, 3>& t2, double tolerance) {
    int count = 0;
    
    for (const auto& v1 : t1) {
        for (const auto& v2 : t2) {
            if (glm::length(v1 - v2) < tolerance) {
                count++;
                break; // Found a match for this vertex, move to next
            }
        }
    }
    
    return count;
}

bool PolyhedronProcessor::validatePolyhedron(const std::vector<glm::ivec3>& vertices, int maxSize, 
                                             double normalThreshold, double convexityMargin) {
    // Get all triangles
    auto triangles = getTriangles(vertices, maxSize);
    
    // Check minimum triangle count (need at least 4 for a tetrahedron)
    if (triangles.size() < 4) {
        return false;
    }
    
    // Check all pairs of triangles
    for (size_t i = 0; i < triangles.size(); ++i) {
        for (size_t j = i + 1; j < triangles.size(); ++j) {
            const auto& triangleA = triangles[i];
            const auto& triangleB = triangles[j];
            
            // Count shared vertices
            int sharedVertices = countSharedVertices(triangleA, triangleB);
            
            // Only check triangles that share 1 or 2 vertices
            if (sharedVertices == 1 || sharedVertices == 2) {
                // Calculate normals
                glm::dvec3 normalA = getTriangleNormal(triangleA);
                glm::dvec3 normalB = getTriangleNormal(triangleB);
                
                // Check if normals are degenerate
                if (glm::length(normalA) < Vec3Compare::eps || glm::length(normalB) < Vec3Compare::eps) {
                    return false; // Degenerate triangle
                }
                
                // Check for nearly opposite normals
                double normalDot = glm::dot(normalA, normalB);
                if (normalDot < normalThreshold) {
                    return false; // Nearly opposite normals
                }
            }
            
            // Convexity check: vector from center of A to center of B should point outward from A
            glm::dvec3 centerA = getTriangleCenter(triangleA);
            glm::dvec3 centerB = getTriangleCenter(triangleB);
            glm::dvec3 normalA = getTriangleNormal(triangleA);
            
            // Check if normal is degenerate
            if (glm::length(normalA) < Vec3Compare::eps) {
                return false; // Degenerate triangle
            }
            
            glm::dvec3 centerToCenter = glm::normalize(centerB - centerA);
            double convexityDot = glm::dot(normalA, centerToCenter);
            
            // If dot product is significantly negative, triangle A is facing inward relative to B (concave)
            if (convexityDot < convexityMargin) {
                return false; // Concave with margin
            }
        }
    }
    
    return true; // All checks passed
}