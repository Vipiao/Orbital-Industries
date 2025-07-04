// CollisionDetectionUtils.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "CollisionResult.h"

// Forward declarations
class BallCollider;
class CubeCollider;
class GridCollider;

class CollisionDetectionUtils {
public:
    // Ball-Ball collision
    static CollisionResult detectBallBall(
        const glm::dvec3& posA, double radiusA,
        const glm::dvec3& posB, double radiusB,
        Collider* colliderA = nullptr, Collider* colliderB = nullptr);
    
    // Ball-Cube collision
    static CollisionResult detectBallCube(
        const glm::dvec3& ballPos, double ballRadius,
        const glm::dvec3& cubePos, const glm::dquat& cubeOri, double cubeWidth,
        Collider* ballCollider = nullptr, Collider* cubeCollider = nullptr);
    
    // Cube-Cube collision
    static CollisionResult detectCubeCube(
        CubeCollider* cubeA, CubeCollider* cubeB);
    
    // Ball-Grid collision (delegate to individual ball-ball tests)
    static CollisionResult detectBallGrid(
        const glm::dvec3& ballPos, double ballRadius,
        const GridCollider* grid,
        Collider* ballCollider = nullptr, Collider* gridCollider = nullptr);
    
    // Cube-Grid collision (delegate to individual cube-ball tests)
    static CollisionResult detectCubeGrid(
        const glm::dvec3& cubePos, const glm::dquat& cubeOri, double cubeWidth,
        const GridCollider* grid,
        Collider* cubeCollider = nullptr, Collider* gridCollider = nullptr);
    
    // Grid-Grid collision (delegate to individual ball-ball tests)
    static CollisionResult detectGridGrid(
        const GridCollider* gridA, const GridCollider* gridB,
        Collider* colliderA = nullptr, Collider* colliderB = nullptr);

private:
    // Helper structures for cube collision detection
    struct SeparatingAxisResult {
        bool isSeparating;
        double penetration;
        glm::dvec3 axis;
    };
    
    struct ProjectionResult {
        double min;
        double max;
    };
    
    struct ContactInfo {
        std::vector<glm::dvec3> contactPoints;
        glm::dvec3 normal;
        double penetration;
    };

    // Cube collision detection helpers
    static SeparatingAxisResult testSeparatingAxis(
        const glm::dvec3& axis,
        const std::vector<glm::dvec3>& verticesA,
        const std::vector<glm::dvec3>& verticesB);
    
    static ProjectionResult projectVertices(
        const std::vector<glm::dvec3>& vertices, 
        const glm::dvec3& axis);
    
    static std::vector<glm::dvec3> getCubeVertices(
        const glm::dvec3& position, 
        const glm::dquat& orientation, 
        double width);
    
    static ContactInfo generateContactPoints(
        const std::vector<glm::dvec3>& verticesA,
        const std::vector<glm::dvec3>& verticesB,
        const glm::dvec3& normal,
        double penetration);
    
    // Polygon clipping helpers
    static std::vector<glm::dvec2> projectToPlane(
        const std::vector<glm::dvec3>& points3D,
        const glm::dmat3& transformMatrix,
        double& averageZ);
    
    static std::vector<glm::dvec2> windPoints(const std::vector<glm::dvec2>& points);
    
    static std::vector<glm::dvec2> sutherlandHodgmanClip(
        const std::vector<glm::dvec2>& subjectPoly,
        const std::vector<glm::dvec2>& clipPoly);
    
    static std::vector<glm::dvec3> projectToWorld(
        const std::vector<glm::dvec2>& points2D,
        const glm::dmat3& inverseMatrix,
        double averageZ = 0.0);

    static glm::dmat3 createPlaneTransform(const glm::dvec3& normal);

    // Helper functions for contact generation
    static std::vector<glm::dvec2> clipSegmentAgainstPolygon(
        const std::vector<glm::dvec2>& segment, const std::vector<glm::dvec2>& polygon);
    static bool clipSegmentAgainstEdge(glm::dvec2& p1, glm::dvec2& p2, 
        const glm::dvec2& edgeStart, const glm::dvec2& edgeEnd);
    
    // General geometry utilities
    static bool segmentIntersection(
        const glm::dvec2& p1, const glm::dvec2& q1,
        const glm::dvec2& p2, const glm::dvec2& q2,
        glm::dvec2& intersection);
};