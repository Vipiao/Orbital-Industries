// GridGeometry.cpp
#include "GridGeometry.h"
#include <stdexcept>
#include "../utils/PolyhedronProcessor.h"
#include "../utils/GeometryUtils.h"

glm::dvec3 GridGeometry::worldToGrid(
    const glm::dvec3& worldPos,
    const glm::dvec3& gridPosition,
    const glm::dquat& gridOrientation,
    const glm::dvec3& gridCenter) {
    
    // Transform: 
    // 1. Translate relative to grid position
    // 2. Rotate by conjugate of grid orientation
    // 3. Add center offset
    return glm::conjugate(gridOrientation) * (worldPos - gridPosition) + gridCenter;
}

glm::dvec3 GridGeometry::gridToWorld(
    const glm::dvec3& gridPos,
    const glm::dvec3& gridPosition,
    const glm::dquat& gridOrientation,
    const glm::dvec3& gridCenter) {
    
    // Transform:
    // 1. Subtract center
    // 2. Apply grid orientation
    // 3. Add grid position
    return gridPosition + gridOrientation * (gridPos - gridCenter);
}

std::vector<glm::ivec3> GridGeometry::gridTraversal(
    const glm::dvec3& startPos, 
    const glm::dvec3& endPos) {
    
    // In case the direction is such that the end cell might be missed.
    // For example {-0.5, 0.5, 0.5}, {0.0, 0.0, 0.5}
    glm::ivec3 newOrigin = glm::floor(endPos);
    glm::dvec3 startPosRel{ startPos - static_cast<glm::dvec3>(newOrigin) };
    glm::dvec3 endPosRel{ endPos - static_cast<glm::dvec3>(newOrigin) };
    constexpr double shift{ 1.e-6 };
    
    if (endPosRel.x == 0.) {
        endPosRel.x = shift;
    }
    if (endPosRel.y == 0.) {
        endPosRel.y = shift;
    }
    if (endPosRel.z == 0.) {
        endPosRel.z = shift;
    }
    
    std::vector<glm::ivec3> cells;
    glm::dvec3 dir{ endPosRel - startPosRel };
    glm::ivec3 step{ glm::sign(dir) };
    glm::dvec3 nextBoundary{ glm::floor(startPosRel) + glm::dvec3{ step } };
    glm::dvec3 tMax{};
    glm::dvec3 tDelta{};
    
    for (int i = 0; i < 3; ++i) {
        // Avoid division by zero for axis-aligned rays
        if (dir[i] > 0.) {
            tMax[i] = (nextBoundary[i] - startPosRel[i]) / dir[i];
            tDelta[i] = glm::abs(step[i] / dir[i]);
        } else if (dir[i] < 0.) {
            tMax[i] = (nextBoundary[i] - startPosRel[i] + 1) / dir[i];
            tDelta[i] = glm::abs(step[i] / dir[i]);
        } else {
            tMax[i] = std::numeric_limits<double>::infinity();
            tDelta[i] = std::numeric_limits<double>::infinity();
        }
    }
    
    glm::ivec3 cell = glm::floor(startPosRel);
    glm::ivec3 endCell = glm::floor(endPosRel);
    
#ifndef NDEBUG // Debug mode.
    uint64_t maxIt{ static_cast<uint64_t>(glm::length(dir) + 1.) * 3 };
    uint64_t iteration{ 0 };
#endif
    
    while (true) {
#ifndef NDEBUG // Debug mode.
        if (iteration++ > maxIt) {
            throw std::runtime_error("gridTraversal too many iterations.");
        }
#endif
        cells.push_back(cell + newOrigin);
        if (cell == endCell) {
            break;
        }
        if (tMax.x < tMax.y) {
            if (tMax.x < tMax.z) {
                cell.x += step.x;
                tMax.x += tDelta.x;
            } else {
                cell.z += step.z;
                tMax.z += tDelta.z;
            }
        } else {
            if (tMax.y < tMax.z) {
                cell.y += step.y;
                tMax.y += tDelta.y;
            } else {
                cell.z += step.z;
                tMax.z += tDelta.z;
            }
        }
    }
    return cells;
}

std::array<bool, 8> GridGeometry::checkPolyhedronBorderIntersection(
    const glm::ivec3& coordA, const std::array<glm::dvec3, 8>& verticesA,
    const glm::ivec3& coordB, const std::array<glm::dvec3, 8>& verticesB) {
    
    std::array<bool, 8> result;
    result.fill(false);
    
    // Calculate the difference between coordinates
    glm::ivec3 diff = coordB - coordA;
    
    // Check if coordinates are adjacent (Manhattan distance exactly 1)
    if (std::abs(diff.x) + std::abs(diff.y) + std::abs(diff.z) != 1) {
        return result; // Not adjacent cells, return all false
    }
    
    // Calculate border axis and value
    int borderAxis = std::abs(diff.y) + std::abs(diff.z) * 2;
    double borderValue = std::max(diff.x + diff.y + diff.z, 0);
    double borderValueB = 1.0 - borderValue;
    
    const double tolerance = 1e-6;
    
    // Arrays for processing both A and B vertices
    const std::array<glm::dvec3, 8>* vertices[2] = {&verticesA, &verticesB};
    double borderValues[2] = {borderValue, borderValueB};
    glm::ivec3 coordOffsets[2] = {glm::ivec3(0), diff};
    
    std::vector<glm::dvec2> borderVerticesA;
    std::vector<int> borderVertexIndicesA;
    std::vector<glm::dvec2> borderVerticesB;
    
    // Collect border vertices for both polyhedra
    for (int polyIdx = 0; polyIdx < 2; ++polyIdx) {
        const auto& polyVertices = *vertices[polyIdx];
        double targetBorderValue = borderValues[polyIdx];
        glm::ivec3 coordOffset = coordOffsets[polyIdx];
        
        for (int i = 0; i < 8; ++i) {
            double axisValue = polyVertices[i][borderAxis];
            if (std::abs(axisValue - targetBorderValue) < tolerance) {
                // This vertex lies on the border, project to 2D
                glm::dvec2 vertex2D;
                if (borderAxis == 0) { // YZ plane
                    vertex2D = glm::dvec2(polyVertices[i].y + coordOffset.y, polyVertices[i].z + coordOffset.z);
                } else if (borderAxis == 1) { // XZ plane
                    vertex2D = glm::dvec2(polyVertices[i].x + coordOffset.x, polyVertices[i].z + coordOffset.z);
                } else { // XY plane
                    vertex2D = glm::dvec2(polyVertices[i].x + coordOffset.x, polyVertices[i].y + coordOffset.y);
                }
                
                if (polyIdx == 0) { // Processing A
                    borderVerticesA.push_back(vertex2D);
                    borderVertexIndicesA.push_back(i);
                } else { // Processing B
                    borderVerticesB.push_back(vertex2D);
                }
            }
        }
    }
    
    // If either polyhedron has no vertices on the border, no intersection possible
    if (borderVerticesA.empty() || borderVerticesB.empty()) {
        return result;
    }
    
    // Wind the B vertices to ensure proper polygon ordering
    std::vector<glm::dvec2> windedVerticesB;
    if (borderVerticesB.size() >= 3) {
        windedVerticesB = GeometryUtils::windPoints(borderVerticesB);
    } else {
        return result; // Can't form a polygon with < 3 vertices
    }
    
    // Check each border vertex of A against the polygon formed by B's border vertices
    const double margin = 0.01; // Small margin for intersection tolerance
    
    for (size_t i = 0; i < borderVerticesA.size(); ++i) {
        int vertexIndex = borderVertexIndicesA[i];
        const glm::dvec2& vertex = borderVerticesA[i];
        
        bool isInside = PolyhedronProcessor::isPointInConvexPolygon(vertex, windedVerticesB, margin);
        result[vertexIndex] = isInside;
    }
    
    return result;
}