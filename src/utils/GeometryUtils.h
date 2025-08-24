// GeometryUtils.h
#pragma once

#include <glm/glm.hpp>
#include <vector>

/**
 * @brief Static utility class for computational geometry operations
 * Includes polygon clipping, separating axis theorem, and geometric transformations
 */
class GeometryUtils {
public:
    // Helper structures for geometric operations
    struct SeparatingAxisResult {
        bool isSeparating;
        double penetration;
        glm::dvec3 axis;
    };
    
    struct ProjectionResult {
        double min;
        double max;
    };

    // Separating Axis Theorem operations
    static SeparatingAxisResult testSeparatingAxis(
        const glm::dvec3& axis,
        const std::vector<glm::dvec3>& verticesA,
        const std::vector<glm::dvec3>& verticesB);
    
    static ProjectionResult projectVertices(
        const std::vector<glm::dvec3>& vertices, 
        const glm::dvec3& axis);

    // Plane transformation operations
    static glm::dmat3 createPlaneTransform(const glm::dvec3& normal);
    
    static std::vector<glm::dvec2> projectToPlane(
        const std::vector<glm::dvec3>& points3D,
        const glm::dmat3& transformMatrix,
        double& averageZ);
    
    static std::vector<glm::dvec2> projectToPlane(
        const std::vector<glm::dvec3>& points3D,
        const glm::dmat3& transformMatrix);
    
    static std::vector<glm::dvec3> projectToWorld(
        const std::vector<glm::dvec2>& points2D,
        const glm::dmat3& inverseMatrix,
        double averageZ = 0.0);

    // Polygon operations
    static std::vector<glm::dvec2> windPoints(const std::vector<glm::dvec2>& points);
    
    // Polygon clipping operations
    static std::vector<glm::dvec2> windPointsAroundOrigin(const std::vector<glm::dvec2>& points);
    
    // Polygon clipping operations
    static std::vector<glm::dvec2> sutherlandHodgmanClip(
        const std::vector<glm::dvec2>& subjectPoly,
        const std::vector<glm::dvec2>& clipPoly);
    
    static std::vector<glm::dvec2> clipSegmentAgainstPolygon(
        const std::vector<glm::dvec2>& segment, 
        const std::vector<glm::dvec2>& polygon);
    
    static bool clipSegmentAgainstEdge(
        glm::dvec2& p1, glm::dvec2& p2, 
        const glm::dvec2& edgeStart, const glm::dvec2& edgeEnd);
    
    // Line segment operations
    static bool segmentIntersection(
        const glm::dvec2& p1, const glm::dvec2& q1,
        const glm::dvec2& p2, const glm::dvec2& q2,
        glm::dvec2& intersection);

    /**
     * @brief Calculate the minimum distance from a point to a triangle
     * @param point The query point
     * @param v0, v1, v2 Triangle vertices
     * @return Minimum distance from point to triangle (always >= 0)
     */
    static double pointToTriangleDistance(
        const glm::dvec3& point,
        const glm::dvec3& v0, const glm::dvec3& v1, const glm::dvec3& v2);

    /**
     * @brief Calculate the closest point on a line segment to a given point
     * @param point The query point
     * @param segmentStart Start of line segment
     * @param segmentEnd End of line segment
     * @return Closest point on segment to the query point
     */
    static glm::dvec3 closestPointOnSegment(
        const glm::dvec3& point,
        const glm::dvec3& segmentStart,
        const glm::dvec3& segmentEnd);

private:
    // Private constructor to prevent instantiation
    GeometryUtils() = delete;
};