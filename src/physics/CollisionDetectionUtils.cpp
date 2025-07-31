// CollisionDetectionUtils.cpp
#include "CollisionDetectionUtils.h"
#include "BallCollider.h"
#include "CubeCollider.h"
#include "GridCollider.h"
#include "PolyhedronCollider.h"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "../debug/DebugGlobals.h"
#include "../debug/DebugRenderer.h"
#include "../utils/PairCache.h"
#include <cmath>

CollisionResult CollisionDetectionUtils::collideWith(Collider* colliderA, Collider* colliderB, uint64_t currentTimestep) {
    int typeA = colliderA->getTypeId();
    int typeB = colliderB->getTypeId();
    
    // Switch table for collision detection
    switch (typeA) {
        case BallCollider::TYPE_ID:
            switch (typeB) {
                case BallCollider::TYPE_ID:
                    return detectBallBall(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<BallCollider*>(colliderB),
                        currentTimestep);
                case CubeCollider::TYPE_ID:
                    return detectBallCube(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<CubeCollider*>(colliderB),
                        currentTimestep);
                case PolyhedronCollider::TYPE_ID:
                    return detectBallCube(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<CubeCollider*>(colliderB),
                        currentTimestep);
                case GridCollider::TYPE_ID:
                    return detectBallGrid(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<GridCollider*>(colliderB),
                        currentTimestep);
            }
            break;
        case CubeCollider::TYPE_ID:
            switch (typeB) {
                case BallCollider::TYPE_ID:
                    return detectPolyhedronBall(
                        static_cast<CubeCollider*>(colliderA),
                        static_cast<BallCollider*>(colliderB), currentTimestep);
                case PolyhedronCollider::TYPE_ID:
                    return detectPolyhedronPolyhedron(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB), currentTimestep);
                case CubeCollider::TYPE_ID:
                    return detectPolyhedronPolyhedron(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB), currentTimestep);
                case GridCollider::TYPE_ID:
                    return detectPolyhedronGrid(
                        static_cast<CubeCollider*>(colliderA),
                        static_cast<GridCollider*>(colliderB), currentTimestep);
            }
            break;
        case PolyhedronCollider::TYPE_ID:
            switch (typeB) {
                case BallCollider::TYPE_ID:
                    return detectPolyhedronBall(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<BallCollider*>(colliderB), currentTimestep);
                case CubeCollider::TYPE_ID:
                    return detectPolyhedronPolyhedron(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB), currentTimestep);
                case PolyhedronCollider::TYPE_ID:
                    return detectPolyhedronPolyhedron(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB), currentTimestep);
                case GridCollider::TYPE_ID:
                    return detectPolyhedronGrid(
                        static_cast<PolyhedronCollider*>(colliderA),
                        static_cast<GridCollider*>(colliderB), currentTimestep);
            }
            break;
        case GridCollider::TYPE_ID:
            switch (typeB) {
                case BallCollider::TYPE_ID:
                    return detectGridBall(
                        static_cast<GridCollider*>(colliderA),
                        static_cast<BallCollider*>(colliderB),
                        currentTimestep);
                case CubeCollider::TYPE_ID:
                    return detectGridPolyhedron(
                        static_cast<GridCollider*>(colliderA),
                        static_cast<CubeCollider*>(colliderB), currentTimestep);
                case PolyhedronCollider::TYPE_ID:
                    return detectGridPolyhedron(
                        static_cast<GridCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB), currentTimestep);
                case GridCollider::TYPE_ID:
                    return detectGridGrid(
                        static_cast<GridCollider*>(colliderA),
                        static_cast<GridCollider*>(colliderB),
                        currentTimestep);
            }
            break;
    }
    
    // Default case - no collision
    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectBallBall(
    BallCollider* ballA, BallCollider* ballB,
    uint64_t currentTimestep) {

    const glm::dvec3& posA = ballA->m_position;
    const glm::dvec3& posB = ballB->m_position;
    double radiusA = ballA->m_radius;
    double radiusB = ballB->m_radius;
    
    // Calculate distance between centers
    glm::dvec3 direction = posB - posA;
    double distanceSquared = glm::length2(direction);
    
    // Calculate sum of radii
    double radiusSum = radiusA + radiusB;
    double radiusSumSquared = radiusSum * radiusSum;
    
    // Check for collision using squared distance test
    if (distanceSquared <= radiusSumSquared) {
        // Collision detected
        double distance = glm::sqrt(distanceSquared);
        
        // Calculate collision normal (from A towards B)
        glm::dvec3 normal;
        if (distance > 1e-9) { // Avoid division by zero
            normal = direction / distance;
        } else {
            // Balls are at exactly the same position, use arbitrary normal
            normal = glm::dvec3(1.0, 0.0, 0.0);
        }
        
        // Calculate contact point (on the surface of ball A)
        glm::dvec3 contactPoint = posA + normal * radiusA;
        
        // Calculate penetration depth
        double penetrationDepth = radiusSum - distance;
        
        return CollisionResult(true, normal, contactPoint, penetrationDepth, ballA, ballB);
    }
    
    // No collision
    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectBallCube(
    BallCollider* ball, CubeCollider* cube,
    uint64_t currentTimestep) {

    const glm::dvec3& ballPos = ball->m_position;
    double ballRadius = ball->m_radius;
    const glm::dvec3& cubePos = cube->m_position;
    const glm::dquat& cubeOri = cube->m_orientation;
    double cubeWidth = cube->m_width;
    
    // Transform ball position to cube's local space
    glm::dvec3 localBallPos = glm::conjugate(cubeOri) * (ballPos - cubePos);
    
    // Half width of cube
    double halfWidth = cubeWidth * 0.5;
    
    // Find closest point on cube to ball center
    glm::dvec3 closestPoint = glm::clamp(localBallPos, 
                                       glm::dvec3(-halfWidth), 
                                       glm::dvec3(halfWidth));
    
    // Calculate distance vector from ball center to closest point (in local space)
    glm::dvec3 localDistanceVec = localBallPos - closestPoint;
    double distanceSquared = glm::length2(localDistanceVec);
    double radiusSquared = ballRadius * ballRadius;
    
    if (distanceSquared <= radiusSquared) {
        // Collision detected
        double distance = glm::sqrt(distanceSquared);
        
        // Calculate normal in local space
        glm::dvec3 localNormal;
        if (distance > 1e-9) {
            localNormal = localDistanceVec / distance;
        } else {
            // Ball center is inside cube, find closest face
            glm::dvec3 distToFaces = glm::abs(localBallPos) - glm::dvec3(halfWidth);
            if (distToFaces.x >= distToFaces.y && distToFaces.x >= distToFaces.z) {
                localNormal = glm::dvec3(glm::sign(localBallPos.x), 0.0, 0.0);
            } else if (distToFaces.y >= distToFaces.z) {
                localNormal = glm::dvec3(0.0, glm::sign(localBallPos.y), 0.0);
            } else {
                localNormal = glm::dvec3(0.0, 0.0, glm::sign(localBallPos.z));
            }
        }
        
        // Transform normal back to world space
        glm::dvec3 worldNormal = cubeOri * localNormal;
        
        // Contact point is on ball surface in direction of normal
        glm::dvec3 contactPoint = ballPos - worldNormal * ballRadius;
        
        // Penetration depth
        double penetration = ballRadius - distance;
        
        return CollisionResult(true, worldNormal, contactPoint, penetration, ball, cube);
    }
    
    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectPolyhedronPolyhedron(
    PolyhedronCollider* polyA, PolyhedronCollider* polyB,
    uint64_t currentTimestep,
    bool useSimplifiedContactGeneration) {

    // Update accurate AABBs for precise collision detection
    polyA->updateAdvancedAABB(currentTimestep);
    polyB->updateAdvancedAABB(currentTimestep);
    
    if (!polyA->checkAABBCollision(polyB)) {
        return CollisionResult(); // No collision after precise AABB check
    }
    
    // Get vertices for both cubes - will use cached vertices
    std::vector<glm::dvec3> verticesA = polyA->getVertices(currentTimestep);
    std::vector<glm::dvec3> verticesB = polyB->getVertices(currentTimestep);

    //if (DebugGlobals::getDebugRenderer()) {
    //    std::string vertA = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(verticesA);
    //    std::cout << vertA << std::endl;
    //    std::string vertB = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(verticesB);
    //    std::cout << vertB << std::endl;
    //}

    //std::string geogebraCommands = m_deb->generateGeogebraCommands(verticesA);
    //std::cout << geogebraCommands << std::endl;
    
    // Get axes for both cubes
    auto [faceAxesA, edgeAxesA, filterNormalsA] = polyA->getCollisionAxes(currentTimestep);
    auto [faceAxesB, edgeAxesB, filterNormalsB] = polyB->getCollisionAxes(currentTimestep);
    
    double minPenetration = std::numeric_limits<double>::max();
    glm::dvec3 separatingAxis;

    // Try cached axis first
    const glm::dvec3* cachedAxis = PairCache<glm::dvec3>::getCachedData(polyA, polyB);
    if (cachedAxis) {
        SeparatingAxisResult result = testSeparatingAxis(*cachedAxis, verticesA, verticesB);
        if (result.isSeparating) {
            // Cache this separating axis again (it worked!)
            PairCache<glm::dvec3>::setCachedData(polyA, polyB, std::move(*cachedAxis));
            return CollisionResult();
        }
    }
    
    // Test face normals of cube A
    for (const glm::dvec3& axis : faceAxesA) {
        SeparatingAxisResult result = testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            // Cache this separating axis
            PairCache<glm::dvec3>::setCachedData(polyA, polyB, glm::normalize(axis));
            return CollisionResult();
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
    }
    
    // Test face normals of cube B
    for (const glm::dvec3& axis : faceAxesB) {
        SeparatingAxisResult result = testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            // Cache this separating axis
            PairCache<glm::dvec3>::setCachedData(polyA, polyB, glm::normalize(axis));
            return CollisionResult();
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
    }
    
    // Test edge-edge cross products
    for (const glm::dvec3& axisA : edgeAxesA) {
        for (const glm::dvec3& axisB : edgeAxesB) {
            glm::dvec3 crossProduct = glm::cross(axisA, axisB);
            
            // Skip nearly parallel edges
            double ll{ glm::length2(crossProduct) };
            if (ll < 0.01) { // Angle ~0.1 radians
                continue;
            }
            
            crossProduct = crossProduct / ll; // Normalize.
            SeparatingAxisResult result = testSeparatingAxis(crossProduct, verticesA, verticesB);
            if (result.isSeparating) {
                // Cache this separating axis
                PairCache<glm::dvec3>::setCachedData(polyA, polyB, crossProduct);
                return CollisionResult();
            }
            if (result.penetration < minPenetration) {
                minPenetration = result.penetration;
                separatingAxis = result.axis;
            }
        }
    }
    
    // If we reach here, there's a collision
    // Ensure normal points from A toward B
    glm::dvec3 centerToCenter = polyB->m_position - polyA->m_position;
    if (glm::dot(separatingAxis, centerToCenter) < 0.0) {
        separatingAxis = -separatingAxis;
    }
    
    // Now separatingAxis is our collision normal pointing from A toward B
    glm::dvec3 collisionNormal = glm::normalize(separatingAxis);
    
    // Check if this collision should be filtered based on filter normals
    const double filterTolerance = 0.7; // arccos(0.7) ~ 45.57 deg tolerance
    
    // Check filter normals from both cubes
    for (const glm::dvec3& filterNormal : filterNormalsA) {
        if (glm::dot(collisionNormal, filterNormal) > filterTolerance) {
            return CollisionResult(polyA, polyB, true); // True means is filtered.
        }
    }
    
    for (const glm::dvec3& filterNormal : filterNormalsB) {
        if (glm::dot(-collisionNormal, filterNormal) > filterTolerance) {
            return CollisionResult(polyA, polyB, true); // True means is filtered.
        }
    }

    // Generate contact points (full complexity)
    ContactInfo contactInfo = generateContactPoints(verticesA, verticesB, collisionNormal, minPenetration, useSimplifiedContactGeneration);
    
    // Create collision result
    CollisionResult result(true, contactInfo.contactPoints.size() > 0 ? 
                          std::vector<glm::dvec3>(contactInfo.contactPoints.size(), contactInfo.normal) :
                          std::vector<glm::dvec3>{contactInfo.normal},
                          contactInfo.contactPoints.size() > 0 ? contactInfo.contactPoints : std::vector<glm::dvec3>{polyA->m_position},
                          contactInfo.contactPoints.size() > 0 ? 
                          std::vector<double>(contactInfo.contactPoints.size(), contactInfo.penetration) :
                          std::vector<double>{contactInfo.penetration},
                          polyA, polyB);
    
    // Reduce contact points if there are too close
    if (result.m_contactPoints.size() > 1) {
        mergeCloseContactPoints(result);
    }

    return result;
}


CollisionResult CollisionDetectionUtils::detectBallGrid(
    BallCollider* ball, GridCollider* grid,
    uint64_t currentTimestep) {
    
    const glm::dvec3& ballPos = ball->m_position;
    double ballRadius = ball->m_radius;

    // Early exit if grid is empty
    if (grid->getCells().empty()) {
        return CollisionResult();
    }
    
    // Find colliders within search area using ball radius
    std::vector<Collider*> nearbyColliders = grid->findCellsInRadius(ballPos, ballRadius);
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Test collision with each found collider
    for (Collider* gridCell : nearbyColliders) {
        // Quick AABB check first
        if (!gridCell->checkAABBCollision(ball)) {
            continue;
        }
        
        // Perform detailed collision detection
        CollisionResult result = detectBallCube(ball, static_cast<CubeCollider*>(gridCell), currentTimestep);
        
        if (result.m_hasCollision) {
            // Add all collision data to our result
            allNormals.insert(allNormals.end(), result.m_normals.begin(), result.m_normals.end());
            allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
            allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        CollisionResult result(true, std::move(allNormals), std::move(allContactPoints), 
                               std::move(allPenetrationDepths), ball, grid);
        
        return result;
    }
    
    return CollisionResult();
}

void CollisionDetectionUtils::processPolyhedronGridCollision(
    PolyhedronCollider* polyhedron,
    GridCollider* grid,
    const glm::dvec3& gridSpacePolyCenterPos,
    std::vector<glm::dvec3>& allNormals,
    std::vector<glm::dvec3>& allContactPoints,
    std::vector<double>& allPenetrationDepths,
    int& collisionPairCount,
    bool useSimplifiedContactGeneration,
    uint64_t currentTimestep,
    bool normalFlip) {
    
    // Early exit if grid is empty
    if (grid->getCells().empty()) {
        return;
    }

    // Get polyhedron classification if it's part of a grid
    CellMetadata::CellClassification polyhedronClass = CellMetadata::CellClassification::CORNER; // Default to corner
    bool hasPolyhedronClass = false;
    CellMetadata* polyhedronMetadata = polyhedron->get_pointer<CellMetadata>();
    if (polyhedronMetadata) {
        polyhedronClass = polyhedronMetadata->classification;
        hasPolyhedronClass = true;
    }

    const double searchRadius = 0.5 * std::sqrt(3.0);

    // Early AABB test optimization - check if cube can possibly intersect grid
    glm::dvec3 expandedMin = glm::dvec3(grid->getLocalAABBMin()) - glm::dvec3(searchRadius);
    glm::dvec3 expandedMax = glm::dvec3(grid->getLocalAABBMax()) + glm::dvec3(1.0) + glm::dvec3(searchRadius);
    
    if (gridSpacePolyCenterPos.x < expandedMin.x || gridSpacePolyCenterPos.x > expandedMax.x ||
        gridSpacePolyCenterPos.y < expandedMin.y || gridSpacePolyCenterPos.y > expandedMax.y ||
        gridSpacePolyCenterPos.z < expandedMin.z || gridSpacePolyCenterPos.z > expandedMax.z) {
        return; // Polyhedron is too far from grid's AABB
    }
    
    // Optimization: Use precomputed neighborhoods if available
    glm::ivec3 gridCenterCoord = glm::ivec3(glm::floor(gridSpacePolyCenterPos));
    auto neighborhoodIt = grid->m_neighborhoods.find(gridCenterCoord);
    
    if (neighborhoodIt != grid->m_neighborhoods.end()) {
        const auto& neighborhood = neighborhoodIt->second;

        // Update polyhedron position
        polyhedron->updatePosition(currentTimestep);
        polyhedron->updateAdvancedAABB(currentTimestep);
        
        // Process all cells in neighborhood (center cell first, then neighbors)
        for (Collider* gridCollider : neighborhood.m_neighbors) {
            // Apply classification-based filtering if polyhedron has classification
            if (hasPolyhedronClass) {
                CellMetadata* gridMetadata = gridCollider->get_pointer<CellMetadata>();
                if (gridMetadata) {
                    CellMetadata::CellClassification gridClass = gridMetadata->classification;
                    
                    // Skip if both are not corners AND not both are edges
                    bool shouldSkip = (polyhedronClass != CellMetadata::CellClassification::CORNER && 
                                     gridClass != CellMetadata::CellClassification::CORNER) &&
                                    !(polyhedronClass == CellMetadata::CellClassification::EDGE && 
                                      gridClass == CellMetadata::CellClassification::EDGE);
                    
                    if (shouldSkip) {
                        continue;
                    }
                }
            }

            // Update this neighbor's transform
            gridCollider->updatePosition(currentTimestep);
            gridCollider->updateAdvancedAABB(currentTimestep);

            // Quick AABB check first
            if (polyhedron->checkAABBCollision(gridCollider)) {
                // Perform detailed collision detection
                CollisionResult result = detectPolyhedronPolyhedron(polyhedron, static_cast<PolyhedronCollider*>(gridCollider), currentTimestep, useSimplifiedContactGeneration);
                
                if (result.m_hasCollision) {
                    collisionPairCount++; // Increment by 1 per collision pair, not per contact point
                    
                    // Directly append collision data to result vectors
                    if (normalFlip) {
                        for (const auto& normal : result.m_normals) {
                            allNormals.push_back(-normal);
                        }
                    } else {
                        for (const auto& normal : result.m_normals) {
                            allNormals.push_back(normal);
                        }
                    }
                    allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
                    allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
                }
            }
        }
    }
}

CollisionResult CollisionDetectionUtils::detectPolyhedronGrid(
    PolyhedronCollider* polyhedron, GridCollider* grid,
    uint64_t currentTimestep) {
    
    // Early exit if grid is empty
    if (grid->getCells().empty()) {
        return CollisionResult();
    }
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    int collisionPairCount = 0;
    
    // Use helper function to process collision
    glm::dvec3 gridSpacePolyCenterPos = grid->worldToGrid(polyhedron->m_position);
    processPolyhedronGridCollision(polyhedron, grid, gridSpacePolyCenterPos, allNormals, allContactPoints, allPenetrationDepths,
                           collisionPairCount, false, currentTimestep, false);
    
    // Return combined result
    if (!allNormals.empty()) {
        CollisionResult result(true, std::move(allNormals), std::move(allContactPoints), 
                               std::move(allPenetrationDepths), polyhedron, grid);
        
        // Simple contact point reduction - remove points within 10cm of each other
        if (result.m_contactPoints.size() > 1) {
            mergeCloseContactPoints(result);
        }
        
        return result;
    }
    
    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectGridGrid(
    GridCollider* gridA, GridCollider* gridB,
    uint64_t currentTimestep) {
    
    // Early exit if either grid is empty
    if (gridA->getCells().empty() || gridB->getCells().empty()) {
        return CollisionResult();
    }

    // Check cached collision data from previous iteration
    const CollisionCacheData* cacheData = PairCache<CollisionCacheData>::getCachedData(gridA, gridB);
    
    // Get AABB centers instead of positions (more representative of actual object centers)
    glm::dvec3 gridBCenterWorld = (gridB->m_AABBMax + gridB->m_AABBMin) * 0.5;
    
    // Calculate B's center in A's coordinate space using worldToGrid
    glm::dvec3 currentBCenterInA = gridA->worldToGrid(gridBCenterWorld);
    glm::dquat currentBOrientationInA = glm::conjugate(gridA->m_orientation) * gridB->m_orientation;
    
    // Check if we can reuse cached collision data
    bool canUseCachedContacts = false;
    if (cacheData) {
        // Check if grid shapes have changed since cache was created (use -1 as uninitialized marker)
        bool shapesUnchanged = (cacheData->gridAShapeTimestamp == gridA->getShapeChangeTimestamp() &&
                               cacheData->gridBShapeTimestamp == gridB->getShapeChangeTimestamp());
        
        if (shapesUnchanged) {
            // Calculate movement threshold using approximate radius (like GridGraphics)
            double approxRadiusB = gridB->getApproximateRadius();
            
            //// Position delta
            //double positionDelta = glm::length(currentBCenterInA - cacheData->prevBCenterInA);

            //// Orientation delta (convert to angle difference)
            //double orientationDot = glm::abs(glm::dot(currentBOrientationInA, cacheData->prevBOrientationInA));
            //orientationDot = glm::clamp(orientationDot, 0.0, 1.0);
            //double angleDiff = 2.0 * glm::acos(orientationDot);
            //double orientationDelta = angleDiff * approxRadiusB;
            //
            //// Movement threshold (similar to GridGraphics adaptive thresholds)
            //const double POSITION_THRESHOLD = 0.08;
            //double maxMovement = positionDelta + orientationDelta;
            //canUseCachedContacts = maxMovement < POSITION_THRESHOLD;

            // The above canUseCachedContacts test is simplified
            // to the below according to the following math.
            // positionDelta + orientationDelta < POSITION_THRESHOLD
            // orientationDelta < POSITION_THRESHOLD - positionDelta
            // approxRadiusB * 2.0 * glm::acos(orientationDot) <
            //     POSITION_THRESHOLD - positionDelta
            // glm::acos(orientationDot) <
            //     (POSITION_THRESHOLD - positionDelta) /
            //     (approxRadiusB * 2.0)
            // glm::cos(glm::acos(orientationDot)) >
            //     glm::cos((POSITION_THRESHOLD - positionDelta) /
            //     (approxRadiusB * 2.0))
            // orientationDot >
            //     glm::cos((POSITION_THRESHOLD - positionDelta) /
            //     (approxRadiusB * 2.0))
            // For small x, cos(x) ~ 1-1/x^2
            // nn = 2. * glm::sqrt(2)
            // remainder = (POSITION_THRESHOLD - positionDelta) /
            //     (nn * approxRadiusB);
            // orientationDot > 1.0 - remainder * remainder
            // Also notice the larger than sign switch due to
            // cos(x) is strictly decreasing close to 0. But what if x is large?
            // No worries as my 1-1/x^2 approximation is always strictly decreasing.
            
            canUseCachedContacts = true;
            // Position delta
            double positionDelta = glm::length(currentBCenterInA - cacheData->prevBCenterInA);
            const double POSITION_THRESHOLD = 0.08;
            if (positionDelta > POSITION_THRESHOLD)
            {
                canUseCachedContacts = false;
            } else {
                // Orientation delta (convert to angle difference)
                double orientationDot = glm::abs(glm::dot(currentBOrientationInA, cacheData->prevBOrientationInA));
                orientationDot = glm::clamp(orientationDot, 0.0, 1.0);

                double constexpr nn = 2. * glm::sqrt(2);
                double remainder = (POSITION_THRESHOLD - positionDelta) / (nn * approxRadiusB);
                if (orientationDot < 1.0 - remainder * remainder)
                {
                    canUseCachedContacts = false;
                }
            }
        }
    }
    
    if (canUseCachedContacts) {
        //std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        // Reuse cached collision data
        if (!cacheData->contactPoints.empty()) {
            // Precalculate local-to-world transform matrices
            glm::dmat4 gridAToWorld = glm::translate(glm::dmat4(1.0), gridA->m_position) * 
                                     glm::mat4_cast(gridA->m_orientation);
            glm::dmat4 gridBToWorld = glm::translate(glm::dmat4(1.0), gridB->m_position) * 
                                     glm::mat4_cast(gridB->m_orientation);
            
            // Calculate how much grid A has rotated since cache was created
            glm::dquat rotationDifference = gridA->m_orientation * glm::conjugate(cacheData->prevAOrientationWorld);
            
            // Rotate the cached normals by the rotation difference
            std::vector<glm::dvec3> updatedNormals;
            updatedNormals.reserve(cacheData->normals.size());
            for (const glm::dvec3& cachedNormal : cacheData->normals) {
                glm::dvec3 rotatedNormal = rotationDifference * cachedNormal;
                updatedNormals.push_back(rotatedNormal);
            }

            // Transform cached local coordinates to current world coordinates
            std::vector<glm::dvec3> updatedContactPoints;
            updatedContactPoints.reserve(cacheData->contactPointsLocalA.size());
            
            for (size_t i = 0; i < cacheData->contactPointsLocalA.size(); ++i) {
                glm::dvec3 worldFromA = glm::dvec3(gridAToWorld * glm::dvec4(cacheData->contactPointsLocalA[i], 1.0));
                glm::dvec3 worldFromB = glm::dvec3(gridBToWorld * glm::dvec4(cacheData->contactPointsLocalB[i], 1.0));
                glm::dvec3 averagedWorld = (worldFromA + worldFromB) * 0.5;
                updatedContactPoints.push_back(averagedWorld);
            }

            CollisionResult result(true, std::move(updatedNormals), std::move(updatedContactPoints),
                                   std::vector<double>(cacheData->penetrationDepths), gridA, gridB,
                                   std::vector<glm::dvec3>(cacheData->contactPointsLocalA), 
                                   std::vector<glm::dvec3>(cacheData->contactPointsLocalB));
            return result;
        } else {
            return CollisionResult(); // No collision cached
        }
    }
    //hasCachedData = false;

    // Use simplified contact generation if previous iteration had too many collision pairs
    const int CONTACT_COMPLEXITY_THRESHOLD = 6;
    bool useSimplifiedContactGeneration = cacheData && cacheData->collisionPairCount > CONTACT_COMPLEXITY_THRESHOLD;
    
    // Calculate the 4 options to find optimal iteration strategy
    size_t totalCorners = gridA->m_cornerCells.size() + gridB->m_cornerCells.size();
    size_t optionA = gridA->getCells().size(); // All cells in A
    size_t optionB = gridB->getCells().size(); // All cells in B
    size_t optionC = gridA->m_edgeCells.size() + totalCorners; // Edges A + All Corners
    size_t optionD = gridB->m_edgeCells.size() + totalCorners; // Edges B + All Corners
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    int collisionPairCount = 0;

    // Reserve some capacity to avoid reallocations
    allNormals.reserve(8);
    allContactPoints.reserve(8);
    allPenetrationDepths.reserve(8);

    // Helper function to process a group of cells
    auto processGroup = [&](const auto& cellMap, const glm::dmat4& transform, GridCollider* targetGrid, bool normalFlip, auto extractCollider) {
        // Collect items into vector for deterministic iteration
        std::vector<std::pair<glm::ivec3, PolyhedronCollider*>> sortedItems;
        sortedItems.reserve(cellMap.size());
        
        for (const auto& [queryCoord, mapValue] : cellMap) {
            PolyhedronCollider* queryCollider = static_cast<PolyhedronCollider*>(extractCollider(mapValue));
            sortedItems.emplace_back(queryCoord, queryCollider);
        }
        
        // Sort by coordinates (lexicographic order: x, then y, then z)
        std::sort(sortedItems.begin(), sortedItems.end(), 
            [](const auto& a, const auto& b) {
                return std::tie(a.first.x, a.first.y, a.first.z) < std::tie(b.first.x, b.first.y, b.first.z);
            });
        
        for (const auto& [queryCoord, queryCollider] : sortedItems) {
        
            // Transform cell coordinate to target grid space
            glm::dvec4 homogeneousCoord = glm::dvec4(queryCoord, 1.0);
            glm::dvec4 transformedCoord = transform * homogeneousCoord;
            glm::dvec3 gridSpacePolyCenterPos = glm::dvec3(transformedCoord);

            processPolyhedronGridCollision(queryCollider, targetGrid, gridSpacePolyCenterPos, allNormals, allContactPoints,
                                   allPenetrationDepths, collisionPairCount, 
                                   useSimplifiedContactGeneration, currentTimestep, normalFlip);
        }
    };
    
    // Choose optimal strategy and execute
    if (optionA <= optionB && optionA <= optionC && optionA <= optionD) {
        // Option A: Iterate over all cells in A, check against all cells in B
        glm::dmat4 worldToGridB = glm::translate(glm::mat4_cast(glm::conjugate(gridB->m_orientation)), -gridB->m_position);
        glm::dmat4 gridACenterToWorldToGridB = glm::translate(worldToGridB, gridA->m_position) * glm::mat4_cast(gridA->m_orientation);
        glm::dmat4 gridACornerToWorldToGridB = glm::translate(gridACenterToWorldToGridB, glm::dvec3(0.5, 0.5, 0.5));
        
        processGroup(gridA->getCells(), gridACornerToWorldToGridB, gridB, false, [](const auto& uniquePtr) { return uniquePtr.get(); });
    }
    else if (optionB <= optionC && optionB <= optionD) {
        // Option B: Iterate over all cells in B, check against all cells in A
        glm::dmat4 worldToGridA = glm::translate(glm::mat4_cast(glm::conjugate(gridA->m_orientation)), -gridA->m_position);
        glm::dmat4 gridBCenterToWorldToGridA = glm::translate(worldToGridA, gridB->m_position) * glm::mat4_cast(gridB->m_orientation);
        glm::dmat4 gridBCornerToWorldToGridA = glm::translate(gridBCenterToWorldToGridA, glm::dvec3(0.5, 0.5, 0.5));
        
        processGroup(gridB->getCells(), gridBCornerToWorldToGridA, gridA, true, [](const auto& uniquePtr) { return uniquePtr.get(); });
    }
    else if (optionC <= optionD) {
        // Option C: Edges A + All Corners
        glm::dmat4 worldToGridB = glm::translate(glm::mat4_cast(glm::conjugate(gridB->m_orientation)), -gridB->m_position);
        glm::dmat4 gridACenterToWorldToGridB = glm::translate(worldToGridB, gridA->m_position) * glm::mat4_cast(gridA->m_orientation);
        glm::dmat4 gridACornerToWorldToGridB = glm::translate(gridACenterToWorldToGridB, glm::dvec3(0.5, 0.5, 0.5));
        
        glm::dmat4 worldToGridA = glm::translate(glm::mat4_cast(glm::conjugate(gridA->m_orientation)), -gridA->m_position);
        glm::dmat4 gridBCenterToWorldToGridA = glm::translate(worldToGridA, gridB->m_position) * glm::mat4_cast(gridB->m_orientation);
        glm::dmat4 gridBCornerToWorldToGridA = glm::translate(gridBCenterToWorldToGridA, glm::dvec3(0.5, 0.5, 0.5));
        
        processGroup(gridA->m_edgeCells, gridACornerToWorldToGridB, gridB, false, [](Collider* ptr) { return ptr; });
        processGroup(gridA->m_cornerCells, gridACornerToWorldToGridB, gridB, false, [](Collider* ptr) { return ptr; });
        processGroup(gridB->m_cornerCells, gridBCornerToWorldToGridA, gridA, true, [](Collider* ptr) { return ptr; });
    }
    else {
        // Option D: Edges B + All Corners
        glm::dmat4 worldToGridA = glm::translate(glm::mat4_cast(glm::conjugate(gridA->m_orientation)), -gridA->m_position);
        glm::dmat4 gridBCenterToWorldToGridA = glm::translate(worldToGridA, gridB->m_position) * glm::mat4_cast(gridB->m_orientation);
        glm::dmat4 gridBCornerToWorldToGridA = glm::translate(gridBCenterToWorldToGridA, glm::dvec3(0.5, 0.5, 0.5));
        
        glm::dmat4 worldToGridB = glm::translate(glm::mat4_cast(glm::conjugate(gridB->m_orientation)), -gridB->m_position);
        glm::dmat4 gridACenterToWorldToGridB = glm::translate(worldToGridB, gridA->m_position) * glm::mat4_cast(gridA->m_orientation);
        glm::dmat4 gridACornerToWorldToGridB = glm::translate(gridACenterToWorldToGridB, glm::dvec3(0.5, 0.5, 0.5));

        processGroup(gridB->m_edgeCells, gridBCornerToWorldToGridA, gridA, true, [](Collider* ptr) { return ptr; });
        processGroup(gridA->m_cornerCells, gridACornerToWorldToGridB, gridB, false, [](Collider* ptr) { return ptr; });
        processGroup(gridB->m_cornerCells, gridBCornerToWorldToGridA, gridA, true, [](Collider* ptr) { return ptr; });
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        CollisionResult result(true, std::move(allNormals), std::move(allContactPoints), 
                               std::move(allPenetrationDepths), 
                               gridA, gridB);
        
        // Reduce contact points if there are too many
        if (result.m_contactPoints.size() > CONTACT_REDUCTION_THRESHOLD) {
            reduceContactPoints(result);
        }

        // Cache the collision data for next iteration
        CollisionCacheData newCacheData;
        newCacheData.prevBCenterInA = currentBCenterInA;
        newCacheData.prevBOrientationInA = currentBOrientationInA;
        newCacheData.prevAOrientationWorld = gridA->m_orientation;
        newCacheData.contactPoints = result.m_contactPoints;
        newCacheData.contactPointsLocalA = result.m_contactPointsLocalA;
        newCacheData.contactPointsLocalB = result.m_contactPointsLocalB;
        newCacheData.normals = result.m_normals;
        newCacheData.penetrationDepths = result.m_penetrationDepths;
        newCacheData.collisionPairCount = collisionPairCount;
        newCacheData.gridAShapeTimestamp = gridA->getShapeChangeTimestamp();
        newCacheData.gridBShapeTimestamp = gridB->getShapeChangeTimestamp();
        newCacheData.cacheTimestamp = currentTimestep;
        PairCache<CollisionCacheData>::setCachedData(gridA, gridB, std::move(newCacheData));

        // Debug visualization of contact points
        //if (DebugGlobals::getDebugRenderer()) {
        //    DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("contact_point_");
        //    for (size_t i = 0; i < result.m_contactPoints.size(); ++i) {
        //        std::string sphereName = "contact_point_" + std::to_string(i);
        //        DebugGlobals::getDebugRenderer()->createSphere(sphereName, result.m_contactPoints[i], 0.1);
        //    }
        //}
        
        return result;
    }
    
    // No collision detected - cache empty collision data
    CollisionCacheData newCacheData;
    newCacheData.prevBCenterInA = currentBCenterInA;
    newCacheData.prevBOrientationInA = currentBOrientationInA;
    newCacheData.prevAOrientationWorld = gridA->m_orientation;
    newCacheData.collisionPairCount = 0;
    newCacheData.gridAShapeTimestamp = gridA->getShapeChangeTimestamp();
    newCacheData.gridBShapeTimestamp = gridB->getShapeChangeTimestamp();
    newCacheData.cacheTimestamp = currentTimestep;
    PairCache<CollisionCacheData>::setCachedData(gridA, gridB, std::move(newCacheData));

    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectPolyhedronBall(
    PolyhedronCollider* polyhedron, BallCollider* ball,
    uint64_t currentTimestep) {
    // Use ball-cube detection and flip normals
    CollisionResult result = detectBallCube(ball, static_cast<CubeCollider*>(polyhedron), currentTimestep);
    
    // Flip normal direction since we called ball-cube instead of cube-ball
    for (glm::dvec3& normal : result.m_normals) {
        normal = -normal;
    }
    
    return result;
}

CollisionResult CollisionDetectionUtils::detectGridBall(
    GridCollider* grid, BallCollider* ball,
    uint64_t currentTimestep) {
    // Use ball-grid detection and flip normals
    CollisionResult result = detectBallGrid(ball, grid, currentTimestep);
    
    // Flip normal direction since we called ball-grid instead of grid-ball
    for (glm::dvec3& normal : result.m_normals) {
        normal = -normal;
    }
    
    return result;
}

CollisionResult CollisionDetectionUtils::detectGridPolyhedron(
    GridCollider* grid, PolyhedronCollider* polyhedron,
    uint64_t currentTimestep) {
    // Use cube-grid detection and flip normals
    CollisionResult result = detectPolyhedronGrid(polyhedron, grid, currentTimestep);
    
    // Flip normal direction since we called cube-grid instead of grid-cube
    for (glm::dvec3& normal : result.m_normals) {
        normal = -normal;
    }
    
    return result;
}

// Helper function implementations

CollisionDetectionUtils::SeparatingAxisResult CollisionDetectionUtils::testSeparatingAxis(
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

CollisionDetectionUtils::ProjectionResult CollisionDetectionUtils::projectVertices(
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

CollisionDetectionUtils::ContactInfo CollisionDetectionUtils::generateContactPoints(
    const std::vector<glm::dvec3>& verticesA,
    const std::vector<glm::dvec3>& verticesB,
    const glm::dvec3& normal,
    double penetration,
    bool useSimplifiedContactGeneration) {
    
    ContactInfo info;
    info.normal = normal;
    info.penetration = penetration;
    
    // Find which vertices are on the positive and negative sides of the separation plane
    std::vector<glm::dvec3> positiveVertices, negativeVertices;
    
    // Project all vertices onto the normal axis
    ProjectionResult projA = projectVertices(verticesA, normal);
    ProjectionResult projB = projectVertices(verticesB, normal);
    
    // Determine which cube is on which side
    if (projA.min > projB.max || projB.min > projA.max) {
        // No overlap - shouldn't happen if we got here
        return info;
    }
    
    // Find overlapping region
    double overlapStart = std::max(projA.min, projB.min);
    double overlapEnd = std::min(projA.max, projB.max);
    double tolerance = 1e-6;
    
    // Collect vertices that are in the overlapping region
    for (const glm::dvec3& vertex : verticesA) {
        double proj = glm::dot(vertex, normal);
        if (proj >= overlapStart - tolerance && proj <= overlapEnd + tolerance) {
            positiveVertices.push_back(vertex);
        }
    }
    
    for (const glm::dvec3& vertex : verticesB) {
        double proj = glm::dot(vertex, normal);
        if (proj >= overlapStart - tolerance && proj <= overlapEnd + tolerance) {
            negativeVertices.push_back(vertex);
        }
    }
    
    if (positiveVertices.empty() || negativeVertices.empty()) {
        // Fallback to simple contact point
        glm::dvec3 centerA(0.0);
        glm::dvec3 centerB(0.0);
        for (const glm::dvec3& v : verticesA) centerA += v;
        for (const glm::dvec3& v : verticesB) centerB += v;
        centerA /= static_cast<double>(verticesA.size());
        centerB /= static_cast<double>(verticesB.size());
        
        info.contactPoints.push_back((centerA + centerB) * 0.5);
        return info;
    }

    // Early exit optimization: if either set has only 1 vertex, use it directly
    if (positiveVertices.size() == 1) {
        info.contactPoints.push_back(positiveVertices[0]);
        return info;
    }
    
    if (negativeVertices.size() == 1) {
        info.contactPoints.push_back(negativeVertices[0]);
        return info;
    }

    // Handle simplified contact generation
    if (useSimplifiedContactGeneration) {
        // Calculate average position of collision vertices
        glm::dvec3 avgPositive(0.0);
        glm::dvec3 avgNegative(0.0);
        
        for (const glm::dvec3& vertex : positiveVertices) {
            avgPositive += vertex;
        }
        avgPositive /= static_cast<double>(positiveVertices.size());
        
        for (const glm::dvec3& vertex : negativeVertices) {
            avgNegative += vertex;
        }
        avgNegative /= static_cast<double>(negativeVertices.size());
        
        glm::dvec3 center = (avgPositive + avgNegative) * 0.5;
        const double radius = 0.2;
        
        // Choose plane perpendicular to collision normal's largest component
        if (std::abs(normal.x) > std::abs(normal.y) && std::abs(normal.x) > std::abs(normal.z)) {
            // Use YZ plane
            info.contactPoints = {
                center + glm::dvec3(0.0, radius, 0.0),
                center + glm::dvec3(0.0, -radius * 0.5, radius * 0.866),
                center + glm::dvec3(0.0, -radius * 0.5, -radius * 0.866)
            };
        } else if (std::abs(normal.y) > std::abs(normal.z)) {
            // Use XZ plane
            info.contactPoints = {
                center + glm::dvec3(radius, 0.0, 0.0),
                center + glm::dvec3(-radius * 0.5, 0.0, radius * 0.866),
                center + glm::dvec3(-radius * 0.5, 0.0, -radius * 0.866)
            };
        } else {
            // Use XY plane
            info.contactPoints = {
                center + glm::dvec3(radius, 0.0, 0.0),
                center + glm::dvec3(-radius * 0.5, radius * 0.866, 0.0),
                center + glm::dvec3(-radius * 0.5, -radius * 0.866, 0.0)
            };
        }
        
        return info;
    }
    
    // Create transformation matrix that aligns normal with Z-axis
    glm::dmat3 transformMatrix = createPlaneTransform(normal);
    glm::dmat3 inverseMatrix = glm::transpose(transformMatrix); // Orthogonal matrix, so transpose = inverse
    
    // Project vertices to 2D plane
    double averageZA, averageZB;
    std::vector<glm::dvec2> points2DA = projectToPlane(positiveVertices, transformMatrix, averageZA);
    std::vector<glm::dvec2> points2DB = projectToPlane(negativeVertices, transformMatrix, averageZB);
    double averageZ = (averageZA + averageZB) * 0.5; // Average of both sets

    // Wind the points to create proper polygons
    points2DA = windPoints(points2DA);
    points2DB = windPoints(points2DB);

    //if (DebugGlobals::getDebugRenderer()) {
    //    std::string aa = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(points2DA);
    //    std::cout << aa << std::endl;
    //    std::string bb = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(points2DB);
    //    std::cout << bb << std::endl;
    //}
    
    // Handle different cases based on vertex count
    size_t sizeA = points2DA.size();
    size_t sizeB = points2DB.size();
    
    std::vector<glm::dvec2> clippedPoints;
    
    // We know we have at least 2 vertices in each set due to early exits above
    if (sizeA > 2) {
        if (sizeB > 2) {
            // Polygon vs Polygon
            clippedPoints = sutherlandHodgmanClip(points2DA, points2DB);
        } else {
            // Polygon vs Edge (sizeB == 2)
            clippedPoints = clipSegmentAgainstPolygon(points2DB, points2DA);
        }
    } else {
        // sizeA == 2 (Edge)
        if (sizeB > 2) {
            // Edge vs Polygon
            clippedPoints = clipSegmentAgainstPolygon(points2DA, points2DB);
        } else {
            // Edge vs Edge (sizeA == 2 and sizeB == 2)
            glm::dvec2 intersection;
            if (segmentIntersection(points2DA[0], points2DA[1], points2DB[0], points2DB[1], intersection)) {
                clippedPoints = {intersection};
            }
        }
    }

    // Convert back to 3D
    std::vector<glm::dvec3> contactPoints3D = projectToWorld(clippedPoints, inverseMatrix, averageZ);

    // Merge close contact points
    const double mergeThreshold = 0.01;
    info.contactPoints.clear();
    
    for (const glm::dvec3& point : contactPoints3D) {
        bool merged = false;
        for (glm::dvec3& existing : info.contactPoints) {
            if (glm::length(point - existing) < mergeThreshold) {
                existing = (existing + point) * 0.5;
                merged = true;
                break;
            }
        }
        if (!merged) {
            info.contactPoints.push_back(point);
        }
    }
    
    //// Limit to maximum 4 contact points
    //if (info.contactPoints.size() > 4) {
    //    info.contactPoints.resize(4);
    //}
    
    return info;
}

std::vector<glm::dvec2> CollisionDetectionUtils::clipSegmentAgainstPolygon(
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

bool CollisionDetectionUtils::clipSegmentAgainstEdge(
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

bool CollisionDetectionUtils::segmentIntersection(
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

glm::dmat3 CollisionDetectionUtils::createPlaneTransform(const glm::dvec3& normal) {
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

std::vector<glm::dvec2> CollisionDetectionUtils::projectToPlane(
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

std::vector<glm::dvec2> CollisionDetectionUtils::windPoints(const std::vector<glm::dvec2>& points) {
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

std::vector<glm::dvec2> CollisionDetectionUtils::sutherlandHodgmanClip(
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
                    double t = -(clipEdge.x * toS.y - clipEdge.y * toS.x) / 
                              (clipEdge.x * se.y - clipEdge.y * se.x);
                    glm::dvec2 intersection = s + t * se;
                    outputList.push_back(intersection);
                }
                outputList.push_back(e);
            } else if (crossS >= 0) { // s is inside, e is outside
                // Find intersection
                glm::dvec2 se = e - s;
                double t = -(clipEdge.x * toS.y - clipEdge.y * toS.x) / 
                          (clipEdge.x * se.y - clipEdge.y * se.x);
                glm::dvec2 intersection = s + t * se;
                outputList.push_back(intersection);
            }
            s = e;
        }
    }
    
    return outputList;
}

std::vector<glm::dvec3> CollisionDetectionUtils::projectToWorld(
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

void CollisionDetectionUtils::mergeCloseContactPoints(CollisionResult& collision, double mergeDistance) {
    if (collision.m_contactPoints.size() <= 1) {
        return;
    }
    
    // Remove contact points that are too close to earlier ones
    for (size_t i = collision.m_contactPoints.size() - 1; i > 0; --i) {
        for (size_t j = 0; j < i; ++j) {
            if (glm::length(collision.m_contactPoints[i] - collision.m_contactPoints[j]) < mergeDistance) {
                collision.m_contactPoints.erase(collision.m_contactPoints.begin() + i);
                collision.m_normals.erase(collision.m_normals.begin() + i);
                collision.m_penetrationDepths.erase(collision.m_penetrationDepths.begin() + i);
                collision.m_contactPointsLocalA.erase(collision.m_contactPointsLocalA.begin() + i);
                collision.m_contactPointsLocalB.erase(collision.m_contactPointsLocalB.begin() + i);
                break;
            }
        }
    }
}

void CollisionDetectionUtils::reduceContactPoints(CollisionResult& collision, int maxPoints) {
    if (collision.m_contactPoints.size() <= static_cast<size_t>(maxPoints)) {
        return; // No reduction needed
    }
    
    size_t numPoints = collision.m_contactPoints.size();
    
    // Step 1: Generate 6D coordinates
    std::vector<std::array<double, 6>> coords6D(numPoints);
    
    // Find min/max for x, y, z coordinates
    double minX = collision.m_contactPoints[0].x, maxX = collision.m_contactPoints[0].x;
    double minY = collision.m_contactPoints[0].y, maxY = collision.m_contactPoints[0].y;
    double minZ = collision.m_contactPoints[0].z, maxZ = collision.m_contactPoints[0].z;
    
    for (const auto& point : collision.m_contactPoints) {
        minX = std::min(minX, point.x);
        maxX = std::max(maxX, point.x);
        minY = std::min(minY, point.y);
        maxY = std::max(maxY, point.y);
        minZ = std::min(minZ, point.z);
        maxZ = std::max(maxZ, point.z);
    }
    
    // Generate normalized 6D coordinates
    for (size_t i = 0; i < numPoints; ++i) {
        const auto& point = collision.m_contactPoints[i];
        const auto& normal = collision.m_normals[i];
        
        // Normalize position coordinates (handle edge case where max == min)
        double normX = (maxX > minX) ? 2.0 * (point.x - minX) / (maxX - minX) - 1.0 : 0.0;
        double normY = (maxY > minY) ? 2.0 * (point.y - minY) / (maxY - minY) - 1.0 : 0.0;
        double normZ = (maxZ > minZ) ? 2.0 * (point.z - minZ) / (maxZ - minZ) - 1.0 : 0.0;
        
        coords6D[i] = {normX, normY, normZ, normal.x, normal.y, normal.z};
    }
    
    // Step 2: Support point selection algorithm
    std::vector<double> costs(numPoints, 0.0);
    std::vector<bool> selected(numPoints, false);
    std::vector<size_t> selectedIndices;
    
    // Find point with largest x-coordinate as first support point
    size_t firstIndex = 0;
    for (size_t i = 1; i < numPoints; ++i) {
        if (collision.m_contactPoints[i].x > collision.m_contactPoints[firstIndex].x) {
            firstIndex = i;
        }
    }
    
    selectedIndices.push_back(firstIndex);
    selected[firstIndex] = true;
    
    // Select remaining points
    while (selectedIndices.size() < static_cast<size_t>(maxPoints) && selectedIndices.size() < numPoints) {
        size_t currentIndex = selectedIndices.back();
        
        // Update costs from current selected point
        for (size_t i = 0; i < numPoints; ++i) {
            if (selected[i]) continue;
            
            // Calculate 6D distance squared
            double distSq = 0.0;
            for (int dim = 0; dim < 6; ++dim) {
                double diff = coords6D[i][dim] - coords6D[currentIndex][dim];
                distSq += diff * diff;
            }
            
            // Add 1/(distance²) to cost (with small epsilon to avoid division by zero)
            costs[i] += 1.0 / (distSq + 1e-12);
        }
        
        // Find point with minimum cost
        size_t nextIndex = 0;
        double minCost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < numPoints; ++i) {
            if (!selected[i] && costs[i] < minCost) {
                minCost = costs[i];
                nextIndex = i;
            }
        }
        
        selectedIndices.push_back(nextIndex);
        selected[nextIndex] = true;
    }
    
    // Step 3: Reconstruct collision result with selected points only
    std::vector<glm::dvec3> newNormals;
    std::vector<glm::dvec3> newContactPoints;
    std::vector<double> newPenetrationDepths;
    std::vector<glm::dvec3> newContactPointsLocalA;
    std::vector<glm::dvec3> newContactPointsLocalB;
    
    for (size_t index : selectedIndices) {
        newNormals.push_back(collision.m_normals[index]);
        newContactPoints.push_back(collision.m_contactPoints[index]);
        newPenetrationDepths.push_back(collision.m_penetrationDepths[index]);
        newContactPointsLocalA.push_back(collision.m_contactPointsLocalA[index]);
        newContactPointsLocalB.push_back(collision.m_contactPointsLocalB[index]);
    }
    
    collision.m_normals = std::move(newNormals);
    collision.m_contactPoints = std::move(newContactPoints);
    collision.m_penetrationDepths = std::move(newPenetrationDepths);
    collision.m_contactPointsLocalA = std::move(newContactPointsLocalA);
    collision.m_contactPointsLocalB = std::move(newContactPointsLocalB);
    
    // Reset collision masses calculation flag since we changed the contacts
    collision.m_collisionMassesCalculated = false;
    collision.m_collisionMasses.clear();
}