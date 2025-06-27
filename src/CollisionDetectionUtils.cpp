// CollisionDetectionUtils.cpp
#include "CollisionDetectionUtils.h"
#include "GridCollider.h"
#include <glm/gtx/norm.hpp>
#include <glm/gtx/transform.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "DebugGlobals.h"
#include "DebugRenderer.h"

CollisionResult CollisionDetectionUtils::detectBallBall(
    const glm::dvec3& posA, double radiusA,
    const glm::dvec3& posB, double radiusB,
    Collider* colliderA, Collider* colliderB) {
    
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
        
        return CollisionResult(true, normal, contactPoint, penetrationDepth);
    }
    
    // No collision
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult CollisionDetectionUtils::detectBallCube(
    const glm::dvec3& ballPos, double ballRadius,
    const glm::dvec3& cubePos, const glm::dquat& cubeOri, double cubeWidth,
    Collider* ballCollider, Collider* cubeCollider) {
    
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
        
        return CollisionResult(true, worldNormal, contactPoint, penetration);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult CollisionDetectionUtils::detectCubeCube(
    const glm::dvec3& posA, const glm::dquat& oriA, double widthA,
    const glm::dvec3& posB, const glm::dquat& oriB, double widthB,
    Collider* colliderA, Collider* colliderB) {
    
    // Get vertices for both cubes
    std::vector<glm::dvec3> verticesA = getCubeVertices(posA, oriA, widthA);
    std::vector<glm::dvec3> verticesB = getCubeVertices(posB, oriB, widthB);

    //if (DebugGlobals::getDebugRenderer()) {
    //    std::string vertA = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(verticesA);
    //    std::cout << vertA << std::endl;
    //    std::string vertB = DebugGlobals::getDebugRenderer()->generateGeogebraCommands(verticesB);
    //    std::cout << vertB << std::endl;
    //}

    //std::string geogebraCommands = m_deb->generateGeogebraCommands(verticesA);
    //std::cout << geogebraCommands << std::endl;
    
    // Get axes for both cubes
    std::vector<glm::dvec3> axesA = getCubeAxes(oriA);
    std::vector<glm::dvec3> axesB = getCubeAxes(oriB);
    
    double minPenetration = std::numeric_limits<double>::max();
    glm::dvec3 separatingAxis;
    bool foundSeparatingAxis = false;
    
    // Test face normals of cube A
    for (const glm::dvec3& axis : axesA) {
        SeparatingAxisResult result = testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
    }
    
    // Test face normals of cube B
    for (const glm::dvec3& axis : axesB) {
        SeparatingAxisResult result = testSeparatingAxis(axis, verticesA, verticesB);
        if (result.isSeparating) {
            return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
        }
        if (result.penetration < minPenetration) {
            minPenetration = result.penetration;
            separatingAxis = result.axis;
        }
    }
    
    // Test edge-edge cross products
    for (const glm::dvec3& axisA : axesA) {
        for (const glm::dvec3& axisB : axesB) {
            glm::dvec3 crossProduct = glm::cross(axisA, axisB);
            
            // Skip nearly parallel edges
            if (glm::length2(crossProduct) < 1e-8) {
                continue;
            }
            
            crossProduct = glm::normalize(crossProduct);
            SeparatingAxisResult result = testSeparatingAxis(crossProduct, verticesA, verticesB);
            if (result.isSeparating) {
                return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
            }
            if (result.penetration < minPenetration) {
                minPenetration = result.penetration;
                separatingAxis = result.axis;
            }
        }
    }
    
    // If we reach here, there's a collision
    // Ensure normal points from A toward B
    glm::dvec3 centerToCenter = posB - posA;
    if (glm::dot(separatingAxis, centerToCenter) < 0.0) {
        separatingAxis = -separatingAxis;
    }
    
    // Now separatingAxis is our collision normal pointing from A toward B
    glm::dvec3 collisionNormal = glm::normalize(separatingAxis);
    
    // Generate contact points
    ContactInfo contactInfo = generateContactPoints(verticesA, verticesB, collisionNormal, minPenetration);
    
    // Update contactInfo normal to ensure consistency
    contactInfo.normal = collisionNormal;

    return CollisionResult(true, contactInfo.contactPoints.size() > 0 ? 
                          std::vector<glm::dvec3>(contactInfo.contactPoints.size(), contactInfo.normal) :
                          std::vector<glm::dvec3>{contactInfo.normal},
                          contactInfo.contactPoints.size() > 0 ? contactInfo.contactPoints : std::vector<glm::dvec3>{posA},
                          contactInfo.contactPoints.size() > 0 ? 
                          std::vector<double>(contactInfo.contactPoints.size(), contactInfo.penetration) :
                          std::vector<double>{contactInfo.penetration});
}

CollisionResult CollisionDetectionUtils::detectBallGrid(
    const glm::dvec3& ballPos, double ballRadius,
    const GridCollider* grid,
    Collider* ballCollider, Collider* gridCollider) {
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Test collision with each sub-collider in the grid
    const auto& cells = grid->getCells();
    for (const auto& pair : cells) {
        CubeCollider* subCollider = pair.second.get();
        
        // Quick AABB check first
        if (!subCollider->checkAABBCollision(ballCollider)) {
            continue;
        }
        
        // Perform detailed collision detection
        CollisionResult result = detectBallCube(
            ballPos, ballRadius,
            subCollider->m_position, subCollider->m_orientation, subCollider->m_width,
            ballCollider, subCollider);
        
        if (result.m_hasCollision) {
            // Add all collision data to our result
            allNormals.insert(allNormals.end(), result.m_normals.begin(), result.m_normals.end());
            allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
            allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        return CollisionResult(true, allNormals, allContactPoints, allPenetrationDepths, ballCollider, gridCollider);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult CollisionDetectionUtils::detectCubeGrid(
    const glm::dvec3& cubePos, const glm::dquat& cubeOri, double cubeWidth,
    const GridCollider* grid,
    Collider* cubeCollider, Collider* gridCollider) {
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Test collision with each sub-collider in the grid
    const auto& cells = grid->getCells();
    for (const auto& pair : cells) {
        CubeCollider* subCollider = pair.second.get();
        
        // Quick AABB check first would require casting - skip for now
        
        // Perform detailed collision detection (cube-cube)
        CollisionResult result = detectCubeCube(
            cubePos, cubeOri, cubeWidth,
            subCollider->m_position, subCollider->m_orientation, subCollider->m_width,
            subCollider, cubeCollider);
        
        if (result.m_hasCollision) {
            // Add all collision data to our result
            allNormals.insert(allNormals.end(), result.m_normals.begin(), result.m_normals.end());
            allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
            allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        return CollisionResult(true, allNormals, allContactPoints, allPenetrationDepths, cubeCollider, gridCollider);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
}

CollisionResult CollisionDetectionUtils::detectGridGrid(
    const GridCollider* gridA, const GridCollider* gridB,
    Collider* colliderA, Collider* colliderB) {

    const auto& cellsA = gridA->getCells();
    const auto& cellsB = gridB->getCells();
    
    // Early exit if either grid is empty
    if (cellsA.empty() || cellsB.empty()) {
        return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
    }
    
    // Choose the smaller grid as the query grid for optimization
    const GridCollider* queryGrid;
    const GridCollider* targetGrid;
    bool normalFlip = false; // Track if we need to flip normals due to order change
    
    if (cellsA.size() <= cellsB.size()) {
        queryGrid = gridA;
        targetGrid = gridB;
        normalFlip = false;
    } else {
        queryGrid = gridB;
        targetGrid = gridA;
        normalFlip = true; // We swapped order, so normals need to be flipped
    }

    const auto& queryCells = (cellsA.size() <= cellsB.size()) ? cellsA : cellsB;
    const auto& targetCells = (cellsA.size() <= cellsB.size()) ? cellsB : cellsA;
    
    // Half-diagonal of a unit cube (sqrt(3)/2), scaled by cube width
    const double searchRadius = 0.5 * std::sqrt(3.0); // Cell is a cube width = 1.0
    
    std::vector<glm::dvec3> allNormals;
    std::vector<glm::dvec3> allContactPoints;
    std::vector<double> allPenetrationDepths;
    
    // Iterate through query grid cells
    for (const auto& queryPair : queryCells) {
        const glm::ivec3& queryCoord = queryPair.first;
        CubeCollider* queryCollider = queryPair.second.get();
        
        // Get center position of query cell and transform to target grid space
        glm::dvec3 queryCellCenter = queryGrid->gridToWorld(glm::dvec3(queryCoord) + glm::dvec3(0.5));
        glm::dvec3 targetSpaceCenter = targetGrid->worldToGrid(queryCellCenter);
        
        // Calculate search bounds in target grid integer coordinates
        glm::ivec3 minCoord = glm::floor(targetSpaceCenter - glm::dvec3(searchRadius));
        glm::ivec3 maxCoord = glm::floor(targetSpaceCenter + glm::dvec3(searchRadius));
        
        // Check all cells in target grid within search bounds
        for (int x = minCoord.x; x <= maxCoord.x; ++x) {
            for (int y = minCoord.y; y <= maxCoord.y; ++y) {
                for (int z = minCoord.z; z <= maxCoord.z; ++z) {
                    glm::ivec3 targetCoord(x, y, z);
                    
                    auto targetIt = targetCells.find(targetCoord);
                    if (targetIt == targetCells.end()) {
                        continue; // No cell at this coordinate
                    }
                    
                    CubeCollider* targetCollider = targetIt->second.get();
            
                    // Quick AABB check first
                    if (!queryCollider->checkAABBCollision(targetCollider)) {
                        continue;
                    }
            
                    // Perform detailed collision detection with correct order
                    CollisionResult result = detectCubeCube(
                        queryCollider->m_position, queryCollider->m_orientation, queryCollider->m_width,
                        targetCollider->m_position, targetCollider->m_orientation, targetCollider->m_width,
                        queryCollider, targetCollider);
            
                    if (result.m_hasCollision) {
                        // Add all collision data to our result, flipping normals if needed
                        for (const auto& normal : result.m_normals) {
                            allNormals.push_back(normalFlip ? -normal : normal);
                        }
                        allContactPoints.insert(allContactPoints.end(), result.m_contactPoints.begin(), result.m_contactPoints.end());
                        allPenetrationDepths.insert(allPenetrationDepths.end(), result.m_penetrationDepths.begin(), result.m_penetrationDepths.end());
                    }
                }
            }
        }
    }
    
    // Return combined result
    if (!allNormals.empty()) {
        return CollisionResult(true, allNormals, allContactPoints, allPenetrationDepths, 
                               colliderA, colliderB);
    }
    
    return CollisionResult(false, std::vector<glm::dvec3>(), std::vector<glm::dvec3>(), std::vector<double>());
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

std::vector<glm::dvec3> CollisionDetectionUtils::getCubeVertices(
    const glm::dvec3& position, 
    const glm::dquat& orientation, 
    double width) {
    
    double halfWidth = width * 0.5;
    std::vector<glm::dvec3> localVertices = {
        {-halfWidth, -halfWidth, -halfWidth},
        { halfWidth, -halfWidth, -halfWidth},
        { halfWidth,  halfWidth, -halfWidth},
        {-halfWidth,  halfWidth, -halfWidth},
        {-halfWidth, -halfWidth,  halfWidth},
        { halfWidth, -halfWidth,  halfWidth},
        { halfWidth,  halfWidth,  halfWidth},
        {-halfWidth,  halfWidth,  halfWidth}
    };
    
    std::vector<glm::dvec3> worldVertices;
    worldVertices.reserve(8);
    
    for (const glm::dvec3& localVertex : localVertices) {
        glm::dvec3 worldVertex = position + orientation * localVertex;
        worldVertices.push_back(worldVertex);
    }
    
    return worldVertices;
}

std::vector<glm::dvec3> CollisionDetectionUtils::getCubeAxes(const glm::dquat& orientation) {
    return {
        orientation * glm::dvec3(1.0, 0.0, 0.0),  // X-axis
        orientation * glm::dvec3(0.0, 1.0, 0.0),  // Y-axis
        orientation * glm::dvec3(0.0, 0.0, 1.0)   // Z-axis
    };
}

CollisionDetectionUtils::ContactInfo CollisionDetectionUtils::generateContactPoints(
    const std::vector<glm::dvec3>& verticesA,
    const std::vector<glm::dvec3>& verticesB,
    const glm::dvec3& normal,
    double penetration) {
    
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
    
    if (sizeA > 2) {
        if (sizeB > 2) {
            clippedPoints = sutherlandHodgmanClip(points2DA, points2DB);
        } else if (sizeB > 1) {
            clippedPoints = clipSegmentAgainstPolygon(points2DB, points2DA);
        } else {
            clippedPoints = points2DB; // Just return the point
        }
    } else if (sizeA > 1) {
        if (sizeB > 2) {
            clippedPoints = clipSegmentAgainstPolygon(points2DA, points2DB);
        } else if (sizeB > 1) {
            // Segment-segment intersection
            glm::dvec2 intersection;
            if (segmentIntersection(points2DA[0], points2DA[1], points2DB[0], points2DB[1], intersection)) {
                clippedPoints = {intersection};
            }
        } else {
            clippedPoints = points2DB; // Just return the point
        }
    } else {
        clippedPoints = points2DA; // Just return the point
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