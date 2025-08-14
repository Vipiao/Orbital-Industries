#include "PolyhedronProcessor.h"
#include <glm/gtx/norm.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include "../math/IntegerVectorMath.h"

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
    glm::ivec3 normal = IntegerVectorMath::cross(edge1, edge2);
    
    // Calculate vector from v0 to v3
    glm::ivec3 vec_to_v3 = v3 - v0;
    
    // Check if quad is convex by testing if v3 is on the same side as the normal
    int dot_product = IntegerVectorMath::dot(normal, vec_to_v3);
    
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

    // Collect all face normals as integer vectors (reserve for max 12 triangles from 6 faces)
    std::vector<glm::ivec3> faceAxisInts;
    faceAxisInts.reserve(12);
    
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
            glm::ivec3 t_normal = IntegerVectorMath::cross(t_edge1, t_edge2);
            
            if (IntegerVectorMath::length2(t_normal) > 0) {
                faceAxisInts.push_back(t_normal);
            }
        }
    }
    
    // Remove duplicates from face axes using cross product comparison
    std::vector<glm::ivec3> uniqueFaceAxisInts;
    uniqueFaceAxisInts.reserve(faceAxisInts.size());
    for (const auto& axis : faceAxisInts) {
        bool isDuplicate = false;
        for (const auto& existingAxis : uniqueFaceAxisInts) {
            glm::ivec3 crossProduct = IntegerVectorMath::cross(axis, existingAxis);
            if (crossProduct == glm::ivec3(0, 0, 0)) {
                isDuplicate = true;
                break;
            }
        }
        if (!isDuplicate) {
            uniqueFaceAxisInts.push_back(axis);
        }
    }
    
    // Normalize face axes and add to result
    result.faceAxis.reserve(uniqueFaceAxisInts.size());
    for (const auto& axis : uniqueFaceAxisInts) {
        result.faceAxis.push_back(glm::normalize(glm::dvec3(axis)));
    }
    
    // Collect all edge directions as integer vectors (reserve for 12 edges max)
    std::vector<glm::ivec3> edgeAxisInts;
    edgeAxisInts.reserve(12);

    for (const auto& edge : CUBE_EDGES) {
        glm::ivec3 edgeVec = vertices[edge[1]] - vertices[edge[0]];
        
        if (IntegerVectorMath::length2(edgeVec) > 0) {
            edgeAxisInts.push_back(edgeVec);
        }
    }
    
    // Remove duplicates from edge axes using cross product comparison
    std::vector<glm::ivec3> uniqueEdgeAxisInts;
    uniqueEdgeAxisInts.reserve(edgeAxisInts.size());
    for (const auto& axis : edgeAxisInts) {
        bool isDuplicate = false;
        for (const auto& existingAxis : uniqueEdgeAxisInts) {
            glm::ivec3 crossProduct = IntegerVectorMath::cross(axis, existingAxis);
            if (crossProduct == glm::ivec3(0, 0, 0)) {
                isDuplicate = true;
                break;
            }
        }
        if (!isDuplicate) {
            uniqueEdgeAxisInts.push_back(axis);
        }
    }
    
    // Normalize edge axes and add to result
    result.edgeAxis.reserve(uniqueEdgeAxisInts.size());
    for (const auto& axis : uniqueEdgeAxisInts) {
        result.edgeAxis.push_back(glm::normalize(glm::dvec3(axis)));
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
            
            glm::ivec3 t_cross = IntegerVectorMath::cross(t_v1 - t_v0, t_v2 - t_v0);
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
            if (sharedVertices == 0) {
                continue;
            }

            // Calculate normals
            glm::dvec3 normalA = getTriangleNormal(triangleA);
            glm::dvec3 normalB = getTriangleNormal(triangleB);
            
            // Check if normals are degenerate
            if (glm::length2(normalA) < Vec3Compare::eps || glm::length2(normalB) < Vec3Compare::eps) {
                return false; // Degenerate triangle
            }
            
            // Check for nearly opposite normals
            double normalDot = glm::dot(normalA, normalB);
            if (normalDot < normalThreshold) {
                return false; // Nearly opposite normals
            }
        
            // Convexity check: vector from center of A to center of B should point outward from A
            glm::dvec3 centerA = getTriangleCenter(triangleA);
            glm::dvec3 centerB = getTriangleCenter(triangleB);
            
            glm::dvec3 centerToCenter = glm::normalize(centerB - centerA);
            double convexityDot = glm::dot(-normalA, centerToCenter);
            
            // If dot product is significantly negative, triangle A is facing inward relative to B (concave)
            if (convexityDot < convexityMargin) {
                return false; // Concave with margin
            }
        }
    }
    
    return true; // All checks passed
}

PolyhedronProcessor::MeshData PolyhedronProcessor::generateMeshData(const std::vector<std::array<glm::dvec3, 3>>& triangles) {
    MeshData meshData;
    
    if (triangles.empty()) {
        return meshData; // Return empty mesh data
    }
    
    // Reserve space for triangle data (3 vertices per triangle)
    size_t numVertices = triangles.size() * 3;
    meshData.positions.reserve(numVertices);
    meshData.normals.reserve(numVertices);
    meshData.tangents.reserve(numVertices);
    meshData.uvs.reserve(numVertices);
    
    // First pass: Generate positions, normals, and determine projection for each triangle
    struct TriangleInfo {
        glm::dvec3 normal;
        int projectionAxis; // 0=YZ, 1=XZ, 2=XY
        std::array<glm::dvec2, 3> projectedCoords;
    };
    
    std::vector<TriangleInfo> triangleInfos;
    triangleInfos.reserve(triangles.size());
    
    // Collect all 2D projected coordinates for global bounding box calculation
    std::vector<glm::dvec2> allProjectedCoords;
    allProjectedCoords.reserve(triangles.size() * 3); // Reserve for known size
    
    for (const auto& triangle : triangles) {
        TriangleInfo info;
        
        // Calculate triangle normal
        glm::dvec3 edge1 = triangle[1] - triangle[0];
        glm::dvec3 edge2 = triangle[2] - triangle[0];
        info.normal = glm::normalize(glm::cross(edge1, edge2));
        
        // Determine best projection axis based on normal
        glm::dvec3 absNormal = glm::abs(info.normal);
        if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z) {
            info.projectionAxis = 0; // Project to YZ plane
        } else if (absNormal.y >= absNormal.z) {
            info.projectionAxis = 1; // Project to XZ plane
        } else {
            info.projectionAxis = 2; // Project to XY plane
        }
        
        // Project triangle vertices to 2D
        for (int i = 0; i < 3; ++i) {
            glm::dvec2 projected;
            switch (info.projectionAxis) {
                case 0: // YZ plane
                    projected = glm::dvec2(triangle[i].y, triangle[i].z);
                    break;
                case 1: // XZ plane
                    projected = glm::dvec2(triangle[i].x, triangle[i].z);
                    break;
                case 2: // XY plane
                    projected = glm::dvec2(triangle[i].x, triangle[i].y);
                    break;
            }
            info.projectedCoords[i] = projected;
            allProjectedCoords.push_back(projected);
        }
        
        triangleInfos.push_back(info);
    }
    
    // Calculate global bounding box for UV normalization
    glm::dvec2 minCoords = allProjectedCoords[0];
    glm::dvec2 maxCoords = allProjectedCoords[0];
    
    for (const auto& coord : allProjectedCoords) {
        minCoords = glm::min(minCoords, coord);
        maxCoords = glm::max(maxCoords, coord);
    }
    
    glm::dvec2 uvRange = maxCoords - minCoords;
    // Prevent division by zero
    if (uvRange.x < 1e-9) uvRange.x = 1.0;
    if (uvRange.y < 1e-9) uvRange.y = 1.0;
    
    // Second pass: Generate all vertex attributes
    for (size_t triIdx = 0; triIdx < triangles.size(); ++triIdx) {
        const auto& triangle = triangles[triIdx];
        const auto& info = triangleInfos[triIdx];
        
        // Generate UVs for this triangle
        std::array<glm::dvec2, 3> uvs;
        for (int i = 0; i < 3; ++i) {
            uvs[i] = (info.projectedCoords[i] - minCoords) / uvRange;
        }
        
        // Calculate tangent vector from UV coordinates
        glm::dvec3 edge1 = triangle[1] - triangle[0];
        glm::dvec3 edge2 = triangle[2] - triangle[0];
        glm::dvec2 deltaUV1 = uvs[1] - uvs[0];
        glm::dvec2 deltaUV2 = uvs[2] - uvs[0];
        
        glm::dvec3 tangent;
        double denominator = deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y;
        
        if (glm::abs(denominator) > 1e-9) {
            double invDenominator = 1.0 / denominator;
            tangent = (deltaUV2.y * edge1 - deltaUV1.y * edge2) * invDenominator;
        } else {
            // Degenerate UV case - generate tangent perpendicular to normal
            if (glm::abs(info.normal.x) < 0.9) {
                tangent = glm::cross(info.normal, glm::dvec3(1.0, 0.0, 0.0));
            } else {
                tangent = glm::cross(info.normal, glm::dvec3(0.0, 1.0, 0.0));
            }
        }
        
        // Orthogonalize tangent against normal and normalize
        tangent = glm::normalize(tangent - glm::dot(tangent, info.normal) * info.normal);
        
        // Add vertices for this triangle
        for (int i = 0; i < 3; ++i) {
            meshData.positions.push_back(triangle[i]);
            meshData.normals.push_back(info.normal);
            meshData.tangents.push_back(tangent);
            meshData.uvs.push_back(uvs[i]);
        }
    }
    
    return meshData;
}

glm::dvec3 PolyhedronProcessor::getGeometricCenter(const std::vector<glm::ivec3>& vertices) {
    if (vertices.empty()) {
        return glm::dvec3(0.0);
    }
    
    glm::dvec3 sum(0.0);
    for (const glm::ivec3& vertex : vertices) {
        sum += glm::dvec3(vertex);
    }
    
    return sum / static_cast<double>(vertices.size());
}

double PolyhedronProcessor::calculateTetrahedronVolume(const glm::dvec3& apex, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& v3) {
    // Volume = |det(v1-apex, v2-apex, v3-apex)| / 6
    glm::dvec3 edge1 = v1 - apex;
    glm::dvec3 edge2 = v2 - apex;
    glm::dvec3 edge3 = v3 - apex;
    
    double det = glm::dot(edge1, glm::cross(edge2, edge3));
    return std::abs(det) / 6.0;
}

glm::dvec3 PolyhedronProcessor::calculateTetrahedronCentroid(const glm::dvec3& apex, const glm::dvec3& v1, const glm::dvec3& v2, const glm::dvec3& v3) {
    // Centroid = (apex + v1 + v2 + v3) / 4
    return (apex + v1 + v2 + v3) * 0.25;
}