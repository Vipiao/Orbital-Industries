#pragma once

#include <glm/glm.hpp>
#include <vector>
#include <set>
#include <array>

// Custom comparator for ivec3 for set operations
struct IVec3Compare {
    bool operator()(const glm::ivec3& a, const glm::ivec3& b) const;
};

// Custom comparator for dvec3 with tolerance for set operations
struct Vec3Compare {
    static constexpr double eps = 1e-9;
    
    bool operator()(const glm::dvec3& a, const glm::dvec3& b) const;
};

class PolyhedronProcessor {
public:
    struct AxisResult {
        std::set<glm::dvec3, Vec3Compare> faceAxis;
        std::set<glm::dvec3, Vec3Compare> edgeAxis;
    };

    /**
     * Extract face normals and edge directions from modified cube vertices
     * @param vertices Vector of 8 cube vertices (possibly modified)
     * @param maxSize Maximum coordinate value (halfWidth = maxSize/2)
     * @return AxisResult containing unique face normals and edge directions
     */
    static AxisResult getAxis(const std::vector<glm::ivec3>& vertices, int maxSize);

    /**
     * Extract unique vertices and normalize them
     * @param vertices Input vertices
     * @param maxSize Maximum coordinate value for normalization
     * @return Vector of unique normalized vertices
     */
    static std::vector<glm::dvec3> getUniqueVertices(const std::vector<glm::ivec3>& vertices, int maxSize);

    /**
     * Extract triangles with non-zero area from the cube faces
     * @param vertices Vector of 8 cube vertices
     * @param maxSize Maximum coordinate value for normalization
     * @return Vector of triangles (each triangle is 3 vertices)
     */
    static std::vector<std::array<glm::dvec3, 3>> getTriangles(const std::vector<glm::ivec3>& vertices, int maxSize);

    /**
     * Validate if the polyhedron formed by the vertices is valid
     * @param vertices Vector of 8 cube vertices
     * @param maxSize Maximum coordinate value for normalization
     * @param normalThreshold Threshold for nearly opposite normals (default -0.9)
     * @param convexityMargin Margin for convexity check (default -0.1)
     * @return true if the polyhedron is valid, false otherwise
     */
    static bool validatePolyhedron(const std::vector<glm::ivec3>& vertices, int maxSize, 
                                   double normalThreshold = -0.9, double convexityMargin = -0.1);

private:
    // Face definitions for cube (indices into vertex array)  
    // Coordinate system: +X right, +Y forward, +Z up (right-handed)
    // Vertices ordered counter-clockwise when viewed from outside
    static constexpr std::array<std::array<int, 4>, 6> CUBE_FACES = {{
        {0, 3, 2, 1}, // Bottom face (Z = -halfWidth)
        {4, 5, 6, 7}, // Top face (Z = +halfWidth)
        {0, 1, 5, 4}, // Back face (Y = -halfWidth)
        {2, 3, 7, 6}, // Front face (Y = +halfWidth)
        {0, 4, 7, 3}, // Left face (X = -halfWidth)
        {1, 2, 6, 5}  // Right face (X = +halfWidth)
    }};

    // Edge definitions for cube (12 edges total)
    static constexpr std::array<std::array<int, 2>, 12> CUBE_EDGES = {{
        // Bottom face edges (Z = -halfWidth)
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        // Top face edges (Z = +halfWidth)
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        // Vertical edges connecting bottom to top
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    }};

    /**
     * Check if a quad is convex and return appropriate triangulation
     * @param v0, v1, v2, v3 The quad vertices in counter-clockwise order
     * @return Array of 2 triangles: either [(v0,v1,v2),(v0,v2,v3)] or [(v0,v1,v3),(v3,v1,v2)]
     */
    static std::array<std::array<int, 3>, 2> getConvexTriangulation(
        const glm::ivec3& v0, const glm::ivec3& v1, const glm::ivec3& v2, const glm::ivec3& v3,
        int idx0, int idx1, int idx2, int idx3);

    /**
     * Calculate the center (centroid) of a triangle
     * @param triangle Triangle vertices
     * @return Center point of the triangle
     */
    static glm::dvec3 getTriangleCenter(const std::array<glm::dvec3, 3>& triangle);

    /**
     * Calculate the normal of a triangle
     * @param triangle Triangle vertices
     * @return Normalized normal vector
     */
    static glm::dvec3 getTriangleNormal(const std::array<glm::dvec3, 3>& triangle);

    /**
     * Count shared vertices between two triangles
     * @param t1, t2 Triangle vertices to compare
     * @param tolerance Tolerance for vertex comparison
     * @return Number of shared vertices (0, 1, 2, or 3)
     */
    static int countSharedVertices(const std::array<glm::dvec3, 3>& t1, const std::array<glm::dvec3, 3>& t2, double tolerance = Vec3Compare::eps);
};