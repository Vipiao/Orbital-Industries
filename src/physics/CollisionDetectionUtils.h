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

// Contact point reduction thresholds
static constexpr size_t CONTACT_REDUCTION_THRESHOLD = 12;
static constexpr int MAX_CONTACT_POINTS = 12;

class CollisionDetectionUtils {
public:
    // Main collision entry point - replaces double dispatch
    static CollisionResult collideWith(Collider* colliderA, Collider* colliderB);

    // Ball-Ball collision
    static CollisionResult detectBallBall(
        BallCollider* ballA, BallCollider* ballB);
    
    // Ball-Cube collision
    static CollisionResult detectBallCube(
        BallCollider* ball, CubeCollider* cube);
    
    // Cube-Cube collision
    static CollisionResult detectCubeCube(
        CubeCollider* cubeA, CubeCollider* cubeB);
    
    // Ball-Grid collision
    static CollisionResult detectBallGrid(
        BallCollider* ball, GridCollider* grid);
    
    // Cube-Grid collision
    static CollisionResult detectCubeGrid(
        CubeCollider* cube, GridCollider* grid);
    
    // Grid-Grid collision
    static CollisionResult detectGridGrid(
        GridCollider* gridA, GridCollider* gridB);

    // Reverse-order functions with normal flipping
    static CollisionResult detectCubeBall(
        CubeCollider* cube, BallCollider* ball);
    static CollisionResult detectGridBall(
        GridCollider* grid, BallCollider* ball);
    static CollisionResult detectGridCube(
        GridCollider* grid, CubeCollider* cube);

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

    // Contact point reduction for performance optimization
    static void reduceContactPoints(CollisionResult& collision, int maxPoints = MAX_CONTACT_POINTS);

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