// GeometryUtils.cpp
#include "GeometryUtils.h"
#include <algorithm>
#include <cmath>
#include <limits>

GeometryUtils::SeparatingAxisResult GeometryUtils::testSeparatingAxis(
    const glm::dvec3& axis,
    const std::vector<glm::dvec3>& verticesA,
    const std::vector<glm::dvec3>& verticesB) {
    
    ProjectionResult projA = projectVertices(verticesA, axis);
    ProjectionResult projB = projectVertices(verticesB, axis);
    
    double overlap = std::min(projA.max, projB.max) - std::max(projA.min, projB.min);
    
    SeparatingAxisResult result;
    result.isSeparating = overlap <= 0;
    result.penetration = overlap;
    result.axis = axis;
    
    return result;
}

GeometryUtils::ProjectionResult GeometryUtils::projectVertices(
    const std::vector<glm::dvec3>& vertices, 
    const glm::dvec3& axis) {
    
    if (vertices.empty()) {
        return {0.0, 0.0};
    }
    
    double minProj = glm::dot(vertices[0], axis);
    double maxProj = minProj;
    
    for (size_t i = 1; i < vertices.size(); ++i) {
        double proj = glm::dot(vertices[i], axis);
        minProj = std::min(minProj, proj);
        maxProj = std::max(maxProj, proj);
    }
    
    return {minProj, maxProj};
}

glm::dmat3 GeometryUtils::createPlaneTransform(const glm::dvec3& normal) {
    // Create orthonormal basis with normal as Z-axis
    glm::dvec3 zAxis = glm::normalize(normal);
    
    // Find a vector not parallel to normal for cross product
    glm::dvec3 tempVector;
    if (std::abs(zAxis.x) < 0.9) {
        tempVector = glm::dvec3(1.0, 0.0, 0.0);
    } else {
        tempVector = glm::dvec3(0.0, 1.0, 0.0);
    }
    
    // Create orthogonal axes
    glm::dvec3 xAxis = glm::normalize(glm::cross(tempVector, zAxis));
    glm::dvec3 yAxis = glm::cross(zAxis, xAxis);
    
    // Build transformation matrix (rows should be the basis vectors for change of basis)
    // GLM constructor puts vectors as columns, so we need to transpose
    glm::dmat3 columnMatrix = glm::dmat3(xAxis, yAxis, zAxis);
    return glm::transpose(columnMatrix);
}

std::vector<glm::dvec2> GeometryUtils::projectToPlane(
    const std::vector<glm::dvec3>& points3D,
    const glm::dmat3& transformMatrix,
    double& averageZ) {

    std::vector<glm::dvec2> points2D;
    points2D.reserve(points3D.size());

    double sumZ = 0.0;

    for (const glm::dvec3& point : points3D) {
        // Transform point to plane coordinate system
        glm::dvec3 transformedPoint = transformMatrix * point;
        // Take X and Y coordinates (Z is the normal direction)
        points2D.push_back(glm::dvec2(transformedPoint.x, transformedPoint.y));
        // Accumulate Z coordinate for average calculation
        sumZ += transformedPoint.z;
    }

    // Calculate average Z displacement
    averageZ = points3D.empty() ? 0.0 : sumZ / static_cast<double>(points3D.size());

    return points2D;
}

std::vector<glm::dvec3> GeometryUtils::projectToWorld(
    const std::vector<glm::dvec2>& points2D,
    const glm::dmat3& inverseMatrix,
    double averageZ) {
    
    std::vector<glm::dvec3> points3D;
    points3D.reserve(points2D.size());
    
    for (const glm::dvec2& point2D : points2D) {
        // Reconstruct 3D point using average Z displacement
        glm::dvec3 planePoint = glm::dvec3(point2D.x, point2D.y, averageZ);
        // Transform back to world coordinates
        glm::dvec3 point3D = inverseMatrix * planePoint;
        points3D.push_back(point3D);
    }
    
    return points3D;
}

std::vector<glm::dvec2> GeometryUtils::windPoints(const std::vector<glm::dvec2>& points) {
    if (points.size() < 3) {
        return points;
    }
    
    // Calculate centroid
    glm::dvec2 center(0.0);
    for (const glm::dvec2& point : points) {
        center += point;
    }
    center /= static_cast<double>(points.size());
    
    // Sort points by angle from center
    std::vector<std::pair<double, size_t>> anglePoints;
    anglePoints.reserve(points.size());
    
    for (size_t i = 0; i < points.size(); ++i) {
        const glm::dvec2& point = points[i];
        glm::dvec2 dir = point - center;
        double angle = std::atan2(dir.y, dir.x);
        anglePoints.push_back({angle, i});
    }
    
    std::sort(anglePoints.begin(), anglePoints.end());
    
    std::vector<glm::dvec2> sortedPoints;
    sortedPoints.reserve(points.size());
    
    for (const auto& pair : anglePoints) {
        sortedPoints.push_back(points[pair.second]);
    }
    
    return sortedPoints;
}

std::vector<glm::dvec2> GeometryUtils::sutherlandHodgmanClip(
    const std::vector<glm::dvec2>& subjectPoly,
    const std::vector<glm::dvec2>& clipPoly) {
    
    if (subjectPoly.empty() || clipPoly.empty()) {
        return {};
    }
    
    std::vector<glm::dvec2> outputList = subjectPoly;
    
    // For each edge of the clipping polygon
    for (size_t i = 0; i < clipPoly.size(); ++i) {
        if (outputList.empty()) break;
        
        size_t j = (i + 1) % clipPoly.size();
        glm::dvec2 clipVertex1 = clipPoly[i];
        glm::dvec2 clipVertex2 = clipPoly[j];
        
        std::vector<glm::dvec2> inputList = outputList;
        outputList.clear();
        
        if (inputList.empty()) continue;
        
        glm::dvec2 s = inputList.back();
        
        for (const glm::dvec2& e : inputList) {
            // Check if point e is inside the clipping edge
            glm::dvec2 clipEdge = clipVertex2 - clipVertex1;
            glm::dvec2 toE = e - clipVertex1;
            double crossE = clipEdge.x * toE.y - clipEdge.y * toE.x;
            
            // Check if point s is inside the clipping edge
            glm::dvec2 toS = s - clipVertex1;
            double crossS = clipEdge.x * toS.y - clipEdge.y * toS.x;
            
            if (crossE >= 0) { // e is inside
                if (crossS < 0) { // s is outside
                    // Find intersection
                    glm::dvec2 se = e - s;
                    double denominator = clipEdge.x * se.y - clipEdge.y * se.x;
                    if (std::abs(denominator) < 1e-9) {
                        // Segments are parallel, skip intersection
                        continue;
                    }
                    double t = -(clipEdge.x * toS.y - clipEdge.y * toS.x) / denominator;
                    glm::dvec2 intersection = s + t * se;
                    outputList.push_back(intersection);
                }
                outputList.push_back(e);
            } else if (crossS >= 0) { // s is inside, e is outside
                // Find intersection
                glm::dvec2 se = e - s;
                double denominator = clipEdge.x * se.y - clipEdge.y * se.x;
                if (std::abs(denominator) < 1e-9) {
                    // Segments are parallel, skip intersection
                    continue;
                }
                double t = -(clipEdge.x * toS.y - clipEdge.y * toS.x) / denominator;
                glm::dvec2 intersection = s + t * se;
                outputList.push_back(intersection);
            }
            s = e;
        }
    }
    
    return outputList;
}

std::vector<glm::dvec2> GeometryUtils::clipSegmentAgainstPolygon(
    const std::vector<glm::dvec2>& segment, const std::vector<glm::dvec2>& polygon) {
    
    if (segment.size() != 2 || polygon.size() < 3) {
        return {};
    }
    
    glm::dvec2 p1 = segment[0];
    glm::dvec2 p2 = segment[1];
    
    // Clip segment against each edge of the polygon
    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t j = (i + 1) % polygon.size();
        glm::dvec2 clipVertex1 = polygon[i];
        glm::dvec2 clipVertex2 = polygon[j];
        
        // Clip segment against this edge
        if (!clipSegmentAgainstEdge(p1, p2, clipVertex1, clipVertex2)) {
            return {}; // Segment is completely outside
        }
    }
    
    return {p1, p2};
}

bool GeometryUtils::clipSegmentAgainstEdge(
    glm::dvec2& p1, glm::dvec2& p2, 
    const glm::dvec2& edgeStart, const glm::dvec2& edgeEnd) {
    
    glm::dvec2 edgeDir = edgeEnd - edgeStart;
    
    // Check which side of the edge each point is on
    double cross1 = edgeDir.x * (p1.y - edgeStart.y) - edgeDir.y * (p1.x - edgeStart.x);
    double cross2 = edgeDir.x * (p2.y - edgeStart.y) - edgeDir.y * (p2.x - edgeStart.x);
    
    if (cross1 >= 0 && cross2 >= 0) {
        return true; // Both points inside
    } else if (cross1 < 0 && cross2 < 0) {
        return false; // Both points outside
    } else {
        // One point inside, one outside - find intersection
        glm::dvec2 segmentDir = p2 - p1;
        double t = cross1 / (cross1 - cross2);
        glm::dvec2 intersection = p1 + t * segmentDir;
        
        if (cross1 < 0) p1 = intersection;
        else p2 = intersection;
        
        return true;
    }
}

bool GeometryUtils::segmentIntersection(
    const glm::dvec2& p1, const glm::dvec2& q1,
    const glm::dvec2& p2, const glm::dvec2& q2,
    glm::dvec2& intersection) {
    
    glm::dvec2 r = q1 - p1;
    glm::dvec2 s = q2 - p2;
    glm::dvec2 qp = p2 - p1;
    
    double rs = r.x * s.y - r.y * s.x;
    double qpr = qp.x * r.y - qp.y * r.x;
    
    if (std::abs(rs) < 1e-9) {
        return false; // Segments are parallel or collinear
    }
    
    double t = (qp.x * s.y - qp.y * s.x) / rs;
    double u = qpr / rs;
    
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        intersection = p1 + t * r;
        return true;
    }
    
    return false; // No intersection within segments
}