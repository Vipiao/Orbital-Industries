// CollisionDetectionUtils.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include "../utils/GeometryUtils.h"
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
static constexpr size_t CONTACT_REDUCTION_THRESHOLD = 12 + 8;

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

    struct ContactInfo {
        std::vector<glm::dvec3> contactPoints;
        glm::dvec3 normal;
        double penetration;
    };

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
    
    // Contact point reduction for performance optimization
    static void mergeCloseContactPoints(CollisionResult& collision, double mergeDistance = 0.3);
    
    // Complex contact point reduction for grid-grid only
    static void reduceContactPoints(CollisionResult& collision, int maxPoints = CONTACT_REDUCTION_THRESHOLD);
};