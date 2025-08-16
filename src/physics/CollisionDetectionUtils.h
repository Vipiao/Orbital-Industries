// CollisionDetectionUtils.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "CollisionResult.h"

// Collision cache data structure for spatial coherence optimization
struct CollisionCacheData {
    // Previous spatial relationship (B relative to A)
    glm::dvec3 prevBCenterInA;
    glm::dquat prevBOrientationInA;
    glm::dquat prevAOrientationWorld; // Store A's world orientation for normal rotation
    
    // Cached collision result
    std::vector<glm::dvec3> contactPoints;
    std::vector<ContactData> contactData;
    std::vector<glm::dvec3> contactPointsLocalA;
    std::vector<glm::dvec3> contactPointsLocalB;
    int collisionPairCount = 0;
    
    // Cache validity tracking
    uint64_t gridAShapeTimestamp = static_cast<uint64_t>(-1); // -1 means uninitialized
    uint64_t gridBShapeTimestamp = static_cast<uint64_t>(-1);
    uint64_t cacheTimestamp = 0;
};

// Forward declarations
class BallCollider;
class CubeCollider;
class GridCollider;
class PolyhedronCollider;

// Contact point reduction thresholds
static constexpr size_t CONTACT_REDUCTION_THRESHOLD = 11;

class CollisionDetectionUtils {
public:
    // Main collision entry point - replaces double dispatch
    static CollisionResult collideWith(Collider* colliderA, Collider* colliderB, uint64_t currentTimestep);

    // Ball-Ball collision
    static CollisionResult detectBallBall(
        BallCollider* ballA, BallCollider* ballB,
        uint64_t currentTimestep);
    
    // Ball-Cube collision
    static CollisionResult detectBallCube(
        BallCollider* ball, CubeCollider* cube,
        uint64_t currentTimestep);
    
    // Polyhedron-Polyhedron collision
    static CollisionResult detectPolyhedronPolyhedron(
        PolyhedronCollider* polyA, PolyhedronCollider* polyB,
        uint64_t currentTimestep,
        bool useSimplifiedContactGeneration = false);
    
    // Ball-Grid collision
    static CollisionResult detectBallGrid(
        BallCollider* ball, GridCollider* grid,
        uint64_t currentTimestep);
    
    // Polyhedron-Grid collision
    static CollisionResult detectPolyhedronGrid(
        PolyhedronCollider* polyhedron, GridCollider* grid,
        uint64_t currentTimestep);
    
    // Grid-Grid collision
    static CollisionResult detectGridGrid(
        GridCollider* gridA, GridCollider* gridB,
        uint64_t currentTimestep);

    // Reverse-order functions with normal flipping
    static CollisionResult detectPolyhedronBall(
        PolyhedronCollider* polyhedron, BallCollider* ball,
        uint64_t currentTimestep);
    static CollisionResult detectGridBall(
        GridCollider* grid, BallCollider* ball,
        uint64_t currentTimestep);
    static CollisionResult detectGridPolyhedron(
        GridCollider* grid, PolyhedronCollider* polyhedron, 
        uint64_t currentTimestep);

private:
    // Helper function to eliminate vector copying between cube-grid and grid-grid detection
    static void processPolyhedronGridCollision(
        PolyhedronCollider* polyhedron, 
        GridCollider* grid,
        const glm::dvec3& gridSpacePolyCenterPos,
        std::vector<ContactData>& allContactData,
        std::vector<glm::dvec3>& allContactPoints,
        int& collisionPairCount,
        bool useSimplifiedContactGeneration = false,
        uint64_t currentTimestep = 0,
        bool normalFlip = false);

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
        double penetration,
        bool useSimplifiedContactGeneration = false);
    
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
    static void mergeCloseContactPoints(CollisionResult& collision, double mergeDistance = 0.3);
    
    // Complex contact point reduction for grid-grid only
    static void reduceContactPoints(CollisionResult& collision, int maxPoints = CONTACT_REDUCTION_THRESHOLD);

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