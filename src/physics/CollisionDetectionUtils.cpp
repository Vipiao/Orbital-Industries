// CollisionDetectionUtils.cpp
#include "CollisionDetectionUtils.h"
#include "BallCollider.h"
#include "CubeCollider.h"
#include "GridCollider.h"
#include "PolyhedronCollider.h"
#include "SensorCollider.h"
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
                    return detectBallPolyhedron(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<PolyhedronCollider*>(colliderB),
                        currentTimestep);
                case GridCollider::TYPE_ID:
                    return detectBallGrid(
                        static_cast<BallCollider*>(colliderA),
                        static_cast<GridCollider*>(colliderB),
                        currentTimestep);
                case SensorCollider::TYPE_ID:
                    return CollisionResult(); // Sensors don't generate collisions
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
                case SensorCollider::TYPE_ID:
                    return CollisionResult(); // Sensors don't generate collisions
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
                case SensorCollider::TYPE_ID:
                    return CollisionResult(); // Sensors don't generate collisions
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
                case SensorCollider::TYPE_ID:
                    return CollisionResult(); // Sensors don't generate collisions
            }
            break;
        case SensorCollider::TYPE_ID:
            // Sensors never generate collision results
            return CollisionResult();
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
        
        return CollisionResult(true, std::vector<ContactData>{ContactData(normal, penetrationDepth)}, 
                              std::vector<glm::dvec3>{contactPoint}, ballA, ballB);
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
        
        // Calculate normal in local space (from ball center toward closest point on cube)
        glm::dvec3 localNormal;  // Points from ball toward cube
        if (distance > 1e-9) {
            // localDistanceVec points from closest point toward ball center (cube→ball)
            // We need ball→cube, so negate it
            localNormal = -localDistanceVec / distance;
        } else {
            // Ball center is inside cube, find closest face
            glm::dvec3 distToFaces = glm::abs(localBallPos) - glm::dvec3(halfWidth);
            if (distToFaces.x >= distToFaces.y && distToFaces.x >= distToFaces.z) {
                // Normal points from ball toward nearest face
                localNormal = glm::dvec3(-glm::sign(localBallPos.x), 0.0, 0.0);
            } else if (distToFaces.y >= distToFaces.z) {
                localNormal = glm::dvec3(0.0, -glm::sign(localBallPos.y), 0.0);
            } else {
                localNormal = glm::dvec3(0.0, 0.0, -glm::sign(localBallPos.z));
            }
        }
        
        // Transform normal back to world space
        glm::dvec3 worldNormal = cubeOri * localNormal;
        
        // Contact point is on ball surface toward cube
        glm::dvec3 contactPoint = ballPos + worldNormal * ballRadius;
        
        // Penetration depth
        double penetration = ballRadius - distance;
        
        return CollisionResult(true, std::vector<ContactData>{ContactData(worldNormal, penetration)}, 
                              std::vector<glm::dvec3>{contactPoint}, ball, cube);
    }
    
    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectBallPolyhedron(
    BallCollider* ball, PolyhedronCollider* polyhedron,
    uint64_t currentTimestep) {

    const glm::dvec3& ballPos = ball->m_position;
    double ballRadius = ball->m_radius;
    const glm::dvec3& polyPos = polyhedron->m_position;
    const glm::dquat& polyOri = polyhedron->m_orientation;
    
    // Transform ball position to polyhedron's local space
    glm::dvec3 localBallPos = glm::conjugate(polyOri) * (ballPos - polyPos);
    
    // Update polyhedron's advanced AABB for precise collision detection
    polyhedron->updateAdvancedAABB(currentTimestep);
    
    // Get polyhedron data
    std::vector<glm::dvec3> vertices = polyhedron->getVertices(currentTimestep);
    auto [faceAxes, edgeAxes, filterNormals] = polyhedron->getCollisionAxes(currentTimestep);
    
    double minPenetration = std::numeric_limits<double>::max();
    glm::dvec3 separatingAxis(0.0);
    bool foundCollision = false;
    
    // Test face normals
    for (const glm::dvec3& axis : faceAxes) {
        glm::dvec3 normalizedAxis = glm::normalize(axis);
        
        // Project polyhedron vertices onto axis
        GeometryUtils::ProjectionResult projPoly = GeometryUtils::projectVertices(vertices, normalizedAxis);
        
        // Project sphere onto axis: sphere center ± radius
        double sphereCenter = glm::dot(localBallPos, normalizedAxis);
        double projSphereMin = sphereCenter - ballRadius;
        double projSphereMax = sphereCenter + ballRadius;
        
        // Check for separation
        if (projSphereMax < projPoly.min || projSphereMin > projPoly.max) {
            return CollisionResult(); // No collision
        }
        
        // Calculate penetration
        double penetration = glm::min(projSphereMax - projPoly.min, projPoly.max - projSphereMin);
        
        if (penetration < minPenetration) {
            minPenetration = penetration;
            separatingAxis = normalizedAxis;
            foundCollision = true;
        }
    }
    
    // Test sphere-center-to-vertex directions
    for (const glm::dvec3& vertex : vertices) {
        glm::dvec3 axis = localBallPos - vertex;
        double axisLengthSq = glm::length2(axis);
        
        // Skip if vertex is very close to sphere center
        if (axisLengthSq < 1e-10) {
            continue;
        }
        
        glm::dvec3 normalizedAxis = axis / glm::sqrt(axisLengthSq);
        
        // Project polyhedron vertices onto axis
        GeometryUtils::ProjectionResult projPoly = GeometryUtils::projectVertices(vertices, normalizedAxis);
        
        // Project sphere onto axis
        double sphereCenter = glm::dot(localBallPos, normalizedAxis);
        double projSphereMin = sphereCenter - ballRadius;
        double projSphereMax = sphereCenter + ballRadius;
        
        // Check for separation
        if (projSphereMax < projPoly.min || projSphereMin > projPoly.max) {
            return CollisionResult(); // No collision
        }
        
        // Calculate penetration
        double penetration = glm::min(projSphereMax - projPoly.min, projPoly.max - projSphereMin);
        
        if (penetration < minPenetration) {
            minPenetration = penetration;
            separatingAxis = normalizedAxis;
            foundCollision = true;
        }
    }
    
    if (!foundCollision) {
        return CollisionResult(); // No collision found
    }
    
    // Ensure normal points from ball toward polyhedron
    // Calculate polyhedron geometric center
    glm::dvec3 polyCenter(0.0);
    for (const glm::dvec3& vertex : vertices) {
        polyCenter += vertex;
    }
    polyCenter /= static_cast<double>(vertices.size());
    
    // Check if axis points from ball toward polyhedron
    glm::dvec3 ballToPolyDir = polyCenter - localBallPos;
    if (glm::dot(separatingAxis, ballToPolyDir) < 0.0) {
        separatingAxis = -separatingAxis;
    }
    
    // Transform normal back to world space
    glm::dvec3 worldNormal = polyOri * separatingAxis;
    
    // Contact point is on ball surface toward polyhedron
    glm::dvec3 contactPoint = ballPos + worldNormal * ballRadius;
    
    return CollisionResult(true, std::vector<ContactData>{ContactData(worldNormal, minPenetration)}, 
                          std::vector<glm::dvec3>{contactPoint}, ball, polyhedron);
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
    
    // Get axes for both cubes
    auto [faceAxesA, edgeAxesA, filterNormalsA] = polyA->getCollisionAxes(currentTimestep);
    auto [faceAxesB, edgeAxesB, filterNormalsB] = polyB->getCollisionAxes(currentTimestep);
    
    // Array to store all tested axes and their penetrations
    struct AxisData {
        glm::dvec3 axis;
        double penetration;
    };
    std::array<AxisData, 300> allTestedAxes;
    size_t axisCount = 0;

    double minPenetration = std::numeric_limits<double>::max();
    glm::dvec3 separatingAxis;

    // Try cached axis first
    constexpr uint64_t SEPARATING_AXIS_CACHE_DURATION = 10; // 10 timesteps
    const glm::dvec3* cachedAxis = PairCache<glm::dvec3>::getCachedData(polyA->m_debugId, polyB->m_debugId, currentTimestep);
    if (cachedAxis) {
        GeometryUtils::SeparatingAxisResult result = GeometryUtils::testSeparatingAxis(*cachedAxis, verticesA, verticesB);
        if (result.isSeparating) {
            // Refresh this separating axis cache (it worked!)
            PairCache<glm::dvec3>::refreshCachedData(polyA->m_debugId, polyB->m_debugId, currentTimestep, SEPARATING_AXIS_CACHE_DURATION);
            return CollisionResult();
        }
    }
    
    // Test face normals of cube A
    for (const glm::dvec3& axis : faceAxesA) {
        if (axisCount < 300) {
            allTestedAxes[axisCount].axis = glm::normalize(axis);
        }
        GeometryUtils::SeparatingAxisResult result = GeometryUtils::testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            // Cache this separating axis
            PairCache<glm::dvec3>::setCachedData(polyA->m_debugId, polyB->m_debugId, glm::normalize(axis), currentTimestep, SEPARATING_AXIS_CACHE_DURATION);
            return CollisionResult();
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
        
        if (axisCount < 300) {
            allTestedAxes[axisCount].penetration = result.penetration;
            axisCount++;
        }
    }
    
    // Test face normals of cube B
    for (const glm::dvec3& axis : faceAxesB) {
        if (axisCount < 300) {
            allTestedAxes[axisCount].axis = glm::normalize(axis);
        }

        GeometryUtils::SeparatingAxisResult result = GeometryUtils::testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            // Cache this separating axis
            PairCache<glm::dvec3>::setCachedData(polyA->m_debugId, polyB->m_debugId, glm::normalize(axis), currentTimestep, SEPARATING_AXIS_CACHE_DURATION);
            return CollisionResult();
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
        
        if (axisCount < 300) {
            allTestedAxes[axisCount].penetration = result.penetration;
            axisCount++;
        }
    }
    
    // Test edge-edge cross products
    for (const glm::dvec3& axisA : edgeAxesA) {
        for (const glm::dvec3& axisB : edgeAxesB) {
            glm::dvec3 crossProduct = glm::cross(axisA, axisB);

            if (axisCount < 300) {
                allTestedAxes[axisCount].axis = glm::normalize(crossProduct);
            }
            
            // Skip nearly parallel edges
            double ll{ glm::length2(crossProduct) };
            if (ll < 0.01) { // Angle ~0.1 radians
                continue;
            }
            
            crossProduct = crossProduct / glm::sqrt(ll); // Normalize.
            GeometryUtils::SeparatingAxisResult result = GeometryUtils::testSeparatingAxis(crossProduct, verticesA, verticesB);
            if (result.isSeparating) {
                // Cache this separating axis
                PairCache<glm::dvec3>::setCachedData(polyA->m_debugId, polyB->m_debugId, crossProduct, currentTimestep, SEPARATING_AXIS_CACHE_DURATION);
                return CollisionResult();
            }
            if (result.penetration < minPenetration) {
                minPenetration = result.penetration;
                separatingAxis = result.axis;
            }
            
            if (axisCount < 300) {
                allTestedAxes[axisCount].penetration = result.penetration;
                axisCount++;
            }
        }
    }
    
    // If we reach here, there's a collision
    // Ensure normal points from A toward B
    // Calculate geometric centers by averaging vertex positions
    glm::dvec3 centerA(0.0);
    glm::dvec3 centerB(0.0);
    for (const glm::dvec3& vertex : verticesA) {
        centerA += vertex;
    }
    for (const glm::dvec3& vertex : verticesB) {
        centerB += vertex;
    }
    centerA /= static_cast<double>(verticesA.size());
    centerB /= static_cast<double>(verticesB.size());
    glm::dvec3 centerToCenter = centerB - centerA;
    if (glm::dot(separatingAxis, centerToCenter) < 0.0) {
        separatingAxis = -separatingAxis;
    }
    
    // Now separatingAxis is our collision normal pointing from A toward B
    glm::dvec3 collisionNormal = glm::normalize(separatingAxis);

    // Ensure all stored axes are oriented consistently with the collision normal direction
    // (flip them the same way we flipped the collision normal)
    for (size_t i = 0; i < axisCount; ++i) {
        glm::dvec3& storedAxis = allTestedAxes[i].axis;
        // Apply the same center-to-center orientation check
        if (glm::dot(storedAxis, centerToCenter) < 0.0) {
            storedAxis = -storedAxis;
        }
    }
    
    // Find compliant normal: must be at least 40 degrees different from collision normal
    // cos(40°) ≈ 0.766
    constexpr double compliantAngleMin = glm::cos(glm::radians(40.0));   // minimum angle threshold
    constexpr double compliantAngleMax = glm::cos(glm::radians(140.0));  // maximum angle threshold
    
    glm::dvec3 compliantNormal(0.0);
    double compliantPenetration = -1.0;
    double bestCompliantPenetration = std::numeric_limits<double>::max();
    bool foundCompliant = false;
    
    for (size_t i = 0; i < axisCount; ++i) {
        
        glm::dvec3 testAxis = allTestedAxes[i].axis;
        double testPenetration = allTestedAxes[i].penetration;
        
        // Skip if this is the collision normal (or very close to it)
        if (glm::abs(glm::dot(collisionNormal, testAxis)) > 0.999) {
            continue;
        }
        
        // Check if between min and max angle different
        double dotProduct = glm::dot(collisionNormal, testAxis);
        if (dotProduct < compliantAngleMin && dotProduct > compliantAngleMax) {
            // This axis is sufficiently different, check if it's the best so far
            if (testPenetration < bestCompliantPenetration) {
                bestCompliantPenetration = testPenetration;
                compliantNormal = testAxis;
                compliantPenetration = testPenetration;
                foundCompliant = true;
            }
        }
    }

    // Generate contact points (full complexity)
    ContactInfo contactInfo = generateContactPoints(verticesA, verticesB, collisionNormal, minPenetration, useSimplifiedContactGeneration);
    
    if (contactInfo.contactPoints.size() == 0) {
        return CollisionResult();
    }

    // Calculate corrected penetration depth for each contact point
    std::vector<double> penetrations; // First projections, then penetrations
    penetrations.reserve(contactInfo.contactPoints.size());
    
    // First pass: calculate projections and find maximum
    double maxProjection = std::numeric_limits<double>::lowest();
    for (const glm::dvec3& contactPoint : contactInfo.contactPoints) {
        double projection = glm::dot(contactPoint, collisionNormal);
        penetrations.push_back(projection);
        maxProjection = std::max(maxProjection, projection);
    }
    
    // Second pass: convert projections to corrected penetration depths
    for (double& projection : penetrations) {
        projection = minPenetration - (maxProjection - projection);
    }

    // Create ContactData with compliant information
    size_t numContactPoints = contactInfo.contactPoints.size();
    std::vector<ContactData> contactData;
    contactData.reserve(numContactPoints);
    
    for (size_t i = 0; i < numContactPoints; ++i) {
        if (foundCompliant) {
            contactData.emplace_back(contactInfo.normal, penetrations[i], compliantNormal, compliantPenetration);
        } else {
            contactData.emplace_back(contactInfo.normal, penetrations[i]);
        }
    }

    // Create collision result
    CollisionResult result(true, std::move(contactData),
                          contactInfo.contactPoints.size() > 0 ? contactInfo.contactPoints : std::vector<glm::dvec3>{polyA->m_position},
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
    
    std::vector<ContactData> allContactData;
    std::vector<glm::dvec3> allContactPoints;
    
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
            allContactData.insert(allContactData.end(), result.m_contactData.begin(), result.m_contactData.end());
            allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
        }
    }
    
    // Return combined result
    if (!allContactData.empty()) {
        CollisionResult result(true, std::move(allContactData), std::move(allContactPoints), ball, grid);
        
        return result;
    }
    
    return CollisionResult();
}

void CollisionDetectionUtils::processPolyhedronGridCollision(
    PolyhedronCollider* polyhedron,
    GridCollider* grid,
    const glm::dvec3& gridSpacePolyCenterPos,
    std::vector<ContactData>& allContactData,
    std::vector<glm::dvec3>& allContactPoints,
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
                    for (const auto& contact : result.m_contactData) {
                        ContactData flippedContact = normalFlip ? 
                            ContactData(-contact.normal, contact.penetration, -contact.compliantNormal, contact.compliantPenetration) :
                            contact;
                        allContactData.push_back(flippedContact);
                    }
                    allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
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
    
    std::vector<ContactData> allContactData;
    std::vector<glm::dvec3> allContactPoints;
    int collisionPairCount = 0;
    
    // Use helper function to process collision
    glm::dvec3 gridSpacePolyCenterPos = grid->worldToGrid(polyhedron->m_position);
    processPolyhedronGridCollision(polyhedron, grid, gridSpacePolyCenterPos, allContactData, allContactPoints,
                           collisionPairCount, false, currentTimestep, false);
    
    // Return combined result
    if (!allContactData.empty()) {
        CollisionResult result(true, std::move(allContactData), std::move(allContactPoints), polyhedron, grid);
        
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
    constexpr uint64_t COLLISION_CACHE_DURATION = 5; // 5 timesteps  
    const CollisionCacheData* cacheData = PairCache<CollisionCacheData>::getCachedData(gridA->m_debugId, gridB->m_debugId, currentTimestep);
    
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
            // For small x, cos(x) ~ 1-x^2/2
            // nn = 2. * glm::sqrt(2)
            // remainder = (POSITION_THRESHOLD - positionDelta) /
            //     (nn * approxRadiusB);
            // orientationDot > 1.0 - remainder * remainder
            // Also notice the larger than sign switch due to
            // cos(x) is strictly decreasing close to 0. But what if x is large?
            // No worries as my 1-1/x^2 approximation is always strictly decreasing.
            
            canUseCachedContacts = true;
            // Position delta
            double positionDeltaSqr = glm::length2(currentBCenterInA - cacheData->prevBCenterInA);
            const double POSITION_THRESHOLD = 0.08*0.5;
            if (positionDeltaSqr > POSITION_THRESHOLD * POSITION_THRESHOLD)
            {
                canUseCachedContacts = false;
            } else {
                // Orientation delta (convert to angle difference)
                double orientationDot = glm::abs(glm::dot(currentBOrientationInA, cacheData->prevBOrientationInA));
                orientationDot = glm::clamp(orientationDot, 0.0, 1.0);

                double constexpr nn = 2.;
                double remainder = (POSITION_THRESHOLD - glm::sqrt(positionDeltaSqr)) / (nn * approxRadiusB);
                if (orientationDot < 1.0 - remainder * remainder / 2.)
                {
                    canUseCachedContacts = false;
                }
            }
        }
    }
    
    if (canUseCachedContacts) {
        //std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        // Reuse cached collision data
        // Refresh cache expiry since we're successfully using it
        PairCache<CollisionCacheData>::refreshCachedData(gridA->m_debugId, gridB->m_debugId, currentTimestep, COLLISION_CACHE_DURATION);
        if (!cacheData->contactPoints.empty()) {
            // Get cached contact data and rotate both normal and compliant normal
            std::vector<ContactData> cachedContactData = cacheData->contactData;

            // Precalculate local-to-world transform matrices
            glm::dmat4 gridAToWorld = glm::translate(glm::dmat4(1.0), gridA->m_position) * 
                                     glm::mat4_cast(gridA->m_orientation);
            glm::dmat4 gridBToWorld = glm::translate(glm::dmat4(1.0), gridB->m_position) * 
                                     glm::mat4_cast(gridB->m_orientation);
            
            // Calculate how much grid A has rotated since cache was created
            glm::dquat rotationDifference = gridA->m_orientation * glm::conjugate(cacheData->prevAOrientationWorld);
            
            // Update both normals and compliant normals with rotation
            for (auto& contact : cachedContactData) {
                contact.normal = rotationDifference * contact.normal;
                // Rotate compliant normal if it's valid (non-zero and positive penetration)
                if (contact.compliantPenetration > 0.0) {
                    contact.compliantNormal = rotationDifference * contact.compliantNormal;
                }
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

            CollisionResult result(true, std::move(cachedContactData), std::move(updatedContactPoints), gridA, gridB,
                                   std::vector<glm::dvec3>(cacheData->contactPointsLocalA), 
                                   std::vector<glm::dvec3>(cacheData->contactPointsLocalB));

            //// Debug visualization of contact points
            //if (DebugGlobals::getDebugRenderer()) {
            //    DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("contact_point_");
            //    for (size_t i = 0; i < result.m_contactPoints.size(); ++i) {
            //        std::string sphereName = "contact_point_" + std::to_string(i);
            //        double radius = result.m_contactData[i].penetration;
            //        DebugGlobals::getDebugRenderer()->createSphere(
            //            sphereName, result.m_contactPoints[i], radius);
            //    }
            //}
            return result;
        } else {
            return CollisionResult(); // No collision cached
        }
    } else {
        //std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }

    // Use simplified contact generation if previous iteration had too many collision pairs
    const int CONTACT_COMPLEXITY_THRESHOLD = 6;
    bool useSimplifiedContactGeneration = cacheData && cacheData->collisionPairCount > CONTACT_COMPLEXITY_THRESHOLD;
    //if (useSimplifiedContactGeneration) std::cout << "SIMPLIFIED" << std::endl;
    // Calculate the 4 options to find optimal iteration strategy
    size_t totalCorners = gridA->m_cornerCells.size() + gridB->m_cornerCells.size();
    size_t optionA = gridA->getCells().size(); // All cells in A
    size_t optionB = gridB->getCells().size(); // All cells in B
    size_t optionC = gridA->m_edgeCells.size() + totalCorners; // Edges A + All Corners
    size_t optionD = gridB->m_edgeCells.size() + totalCorners; // Edges B + All Corners
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<ContactData> allContactData;
    int collisionPairCount = 0;

    // Reserve some capacity to avoid reallocations
    allContactData.reserve(8);
    allContactPoints.reserve(8);

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

            processPolyhedronGridCollision(queryCollider, targetGrid, gridSpacePolyCenterPos, allContactData, allContactPoints,
                                   collisionPairCount, 
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
    if (!allContactData.empty()) {
        CollisionResult result(true, std::move(allContactData), std::move(allContactPoints),
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
        newCacheData.contactData = result.m_contactData;
        newCacheData.collisionPairCount = collisionPairCount;
        newCacheData.gridAShapeTimestamp = gridA->getShapeChangeTimestamp();
        newCacheData.gridBShapeTimestamp = gridB->getShapeChangeTimestamp();
        newCacheData.cacheTimestamp = currentTimestep;
        PairCache<CollisionCacheData>::setCachedData(gridA->m_debugId, gridB->m_debugId, std::move(newCacheData), currentTimestep, COLLISION_CACHE_DURATION);

        // Debug visualization of contact points
        //if (DebugGlobals::getDebugRenderer()) {
        //    DebugGlobals::getDebugRenderer()->removeMeshesByPrefix("contact_point_");
        //    for (size_t i = 0; i < result.m_contactPoints.size(); ++i) {
        //        std::string sphereName = "contact_point_" + std::to_string(i);
        //        double radius = result.m_contactData[i].penetration;
        //        DebugGlobals::getDebugRenderer()->createSphere(
        //            sphereName, result.m_contactPoints[i], radius);
        //    }
        //}
        
        return result;
    }
    
    // No collision detected - cache empty collision data
    CollisionCacheData newCacheData;
    newCacheData.prevBCenterInA = currentBCenterInA;
    newCacheData.prevBOrientationInA = currentBOrientationInA;
    newCacheData.prevAOrientationWorld = gridA->m_orientation;
    newCacheData.contactData.clear(); // Empty contact data for no collision
    newCacheData.collisionPairCount = 0;
    newCacheData.gridAShapeTimestamp = gridA->getShapeChangeTimestamp();
    newCacheData.gridBShapeTimestamp = gridB->getShapeChangeTimestamp();
    newCacheData.cacheTimestamp = currentTimestep;
    PairCache<CollisionCacheData>::setCachedData(gridA->m_debugId, gridB->m_debugId, std::move(newCacheData), currentTimestep, COLLISION_CACHE_DURATION);

    return CollisionResult();
}

CollisionResult CollisionDetectionUtils::detectPolyhedronBall(
    PolyhedronCollider* polyhedron, BallCollider* ball,
    uint64_t currentTimestep) {
    // Use ball-polyhedron detection and flip normals
    CollisionResult result = detectBallPolyhedron(ball, polyhedron, currentTimestep);
    
    // Flip normal direction since we called ball-cube instead of cube-ball
    for (auto& contact : result.m_contactData) {
        contact.normal = -contact.normal;
        contact.compliantNormal = -contact.compliantNormal;
    }
    
    return result;
}

CollisionResult CollisionDetectionUtils::detectGridBall(
    GridCollider* grid, BallCollider* ball,
    uint64_t currentTimestep) {
    // Use ball-grid detection and flip normals
    CollisionResult result = detectBallGrid(ball, grid, currentTimestep);
    
    // Flip normal direction since we called ball-grid instead of grid-ball
    for (auto& contact : result.m_contactData) {
        contact.normal = -contact.normal;
        contact.compliantNormal = -contact.compliantNormal;
    }
    
    return result;
}

CollisionResult CollisionDetectionUtils::detectGridPolyhedron(
    GridCollider* grid, PolyhedronCollider* polyhedron,
    uint64_t currentTimestep) {
    // Use cube-grid detection and flip normals
    CollisionResult result = detectPolyhedronGrid(polyhedron, grid, currentTimestep);
    
    // Flip normal direction since we called cube-grid instead of grid-cube
    for (auto& contact : result.m_contactData) {
        contact.normal = -contact.normal;
        contact.compliantNormal = -contact.compliantNormal;
    }
    
    return result;
}

// Helper function implementations

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
    GeometryUtils::ProjectionResult projA = GeometryUtils::projectVertices(verticesA, normal);
    GeometryUtils::ProjectionResult projB = GeometryUtils::projectVertices(verticesB, normal);
    
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
    glm::dmat3 transformMatrix = GeometryUtils::createPlaneTransform(normal);
    glm::dmat3 inverseMatrix = glm::transpose(transformMatrix); // Orthogonal matrix, so transpose = inverse
    
    // Transform vertices to plane coordinate system (keeping full 3D)
    std::vector<glm::dvec3> transformed3DA, transformed3DB;
    transformed3DA.reserve(positiveVertices.size());
    transformed3DB.reserve(negativeVertices.size());

    for (const glm::dvec3& vertex : positiveVertices) {
        transformed3DA.push_back(transformMatrix * vertex);
    }

    for (const glm::dvec3& vertex : negativeVertices) {
        transformed3DB.push_back(transformMatrix * vertex);
    }

    // Extract 2D points for clipping (take X and Y coordinates)
    std::vector<glm::dvec2> points2DA, points2DB;
    points2DA.reserve(transformed3DA.size());
    points2DB.reserve(transformed3DB.size());

    for (const glm::dvec3& point3D : transformed3DA) {
        points2DA.push_back(glm::dvec2(point3D.x, point3D.y));
    }

    for (const glm::dvec3& point3D : transformed3DB) {
        points2DB.push_back(glm::dvec2(point3D.x, point3D.y));
    }

    // Wind the points to create proper polygons
    points2DA = GeometryUtils::windPoints(points2DA);
    points2DB = GeometryUtils::windPoints(points2DB);

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
            clippedPoints = GeometryUtils::sutherlandHodgmanClip(points2DA, points2DB);
        } else {
            // Polygon vs Edge (sizeB == 2)
            clippedPoints = GeometryUtils::clipSegmentAgainstPolygon(points2DB, points2DA);
        }
    } else {
        // sizeA == 2 (Edge)
        if (sizeB > 2) {
            // Edge vs Polygon
            clippedPoints = GeometryUtils::clipSegmentAgainstPolygon(points2DA, points2DB);
        } else {
            // Edge vs Edge (sizeA == 2 and sizeB == 2)
            glm::dvec2 intersection;
            if (GeometryUtils::segmentIntersection(points2DA[0], points2DA[1], points2DB[0], points2DB[1], intersection)) {
                clippedPoints = {intersection};
            }
        }
    }

    // Restore Z coordinates using nearest neighbor search
    std::vector<glm::dvec3> contactPoints3D;
    contactPoints3D.reserve(clippedPoints.size());

    for (const glm::dvec2& clippedPoint2D : clippedPoints) {
        double minDistanceSq = std::numeric_limits<double>::max();
        double nearestZ = 0.0;
        
        std::vector<glm::dvec3>* transformed3DArr[2] = {&transformed3DA, &transformed3DB};

        for (size_t ii = 0; ii < 2; ii++) {
            std::vector<glm::dvec3>* transformed3D = transformed3DArr[ii];
        
            // Check transformed3D
            for (const glm::dvec3& point3D : *transformed3D) {
                double dx = clippedPoint2D.x - point3D.x;
                double dy = clippedPoint2D.y - point3D.y;
                double distanceSq = dx * dx + dy * dy;
                
                if (distanceSq < minDistanceSq) {
                    minDistanceSq = distanceSq;
                    nearestZ = point3D.z;
                }
            }
        }
        
        // Reconstruct 3D point and convert back to world coordinates
        glm::dvec3 restoredPoint3D = glm::dvec3(clippedPoint2D.x, clippedPoint2D.y, nearestZ);
        contactPoints3D.push_back(inverseMatrix * restoredPoint3D);
    }

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

void CollisionDetectionUtils::mergeCloseContactPoints(CollisionResult& collision, double mergeDistance) {
    if (collision.m_contactData.size() <= 1) {
        return;
    }
    
    size_t numContacts = collision.m_contactData.size();
    std::vector<bool> markedForRemoval(numContacts, false);
    double mergeDistanceSquared = mergeDistance * mergeDistance;
    
    // Phase 1: Mark contacts for removal
    for (size_t i = 0; i < numContacts; ++i) {
        if (markedForRemoval[i]) continue; // Skip already marked contacts
        
        for (size_t j = i + 1; j < numContacts; ++j) {
            if (markedForRemoval[j]) continue; // Skip already marked contacts
            
            double distanceSquared = glm::length2(collision.m_contactPoints[i] - collision.m_contactPoints[j]);
            if (distanceSquared < mergeDistanceSquared) {
                // Mark the contact with smaller penetration depth for removal
                double penetrationI = collision.m_contactData[i].penetration;
                double penetrationJ = collision.m_contactData[j].penetration;
                
                if (penetrationI < penetrationJ) {
                    markedForRemoval[i] = true;
                    break; // No need to check more pairs with i since it's marked for removal
                } else {
                    markedForRemoval[j] = true;
                    // Continue checking j+1, j+2, etc. since i is still valid
                }
            }
        }
    }
    
    // Phase 2: Remove marked contacts in reverse order
    for (size_t i = numContacts; i > 0; --i) {
        size_t index = i - 1;
        if (markedForRemoval[index]) {
            collision.m_contactData.erase(collision.m_contactData.begin() + index);
            collision.m_contactPoints.erase(collision.m_contactPoints.begin() + index);
            collision.m_contactPointsLocalA.erase(collision.m_contactPointsLocalA.begin() + index);
            collision.m_contactPointsLocalB.erase(collision.m_contactPointsLocalB.begin() + index);
        }
    }
}

void CollisionDetectionUtils::reduceContactPoints(CollisionResult& collision, int maxPoints) {
    if (collision.m_contactData.size() <= static_cast<size_t>(maxPoints)) {
        return; // No reduction needed
    }
    
    size_t numPoints = collision.m_contactData.size();
    
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
        const auto& contact = collision.m_contactData[i];
        
        // Normalize position coordinates (handle edge case where max == min)
        double normX = (maxX > minX) ? 2.0 * (point.x - minX) / (maxX - minX) - 1.0 : 0.0;
        double normY = (maxY > minY) ? 2.0 * (point.y - minY) / (maxY - minY) - 1.0 : 0.0;
        double normZ = (maxZ > minZ) ? 2.0 * (point.z - minZ) / (maxZ - minZ) - 1.0 : 0.0;
        
        coords6D[i] = {{normX, normY, normZ, contact.normal.x, contact.normal.y, contact.normal.z}};
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
    std::vector<ContactData> newContactData;
    std::vector<glm::dvec3> newContactPoints;
    std::vector<glm::dvec3> newContactPointsLocalA;
    std::vector<glm::dvec3> newContactPointsLocalB;
    
    for (size_t index : selectedIndices) {
        newContactData.push_back(collision.m_contactData[index]);
        newContactPoints.push_back(collision.m_contactPoints[index]);
        newContactPointsLocalA.push_back(collision.m_contactPointsLocalA[index]);
        newContactPointsLocalB.push_back(collision.m_contactPointsLocalB[index]);
    }
    
    collision.m_contactData = std::move(newContactData);
    collision.m_contactPoints = std::move(newContactPoints);
    collision.m_contactPointsLocalA = std::move(newContactPointsLocalA);
    collision.m_contactPointsLocalB = std::move(newContactPointsLocalB);
    
    // Reset collision masses calculation flag since we changed the contacts
    collision.m_collisionMassesCalculated = false;
    collision.m_collisionMasses.clear();
}