// GridRaycast.cpp
#include "GridRaycast.h"

#include "Grid.h"
#include "utils/GridGeometry.h"

#include <type_traits>

namespace {

// Nudges the hit past the surface so the floor below lands inside the hit cell
// rather than on the boundary between it and its neighbour.
constexpr double k_surfaceEpsilon = 1e-6;

// Hit of a ray against one grid, with the grid handle left to the caller.
std::optional<GridRayHit> intersectGrid(const Grid& grid,
                                        const glm::dvec3& rayStart,
                                        const glm::dvec3& rayEnd,
                                        double timeRemainder) {
    glm::dvec3 interpolatedPos;
    glm::dquat interpolatedOri;
    grid.getInterpolatedTransform(timeRemainder, interpolatedPos, interpolatedOri);

    const glm::dvec3 localStart{
        GridGeometry::worldToGrid(rayStart, interpolatedPos, interpolatedOri)};
    const glm::dvec3 localEnd{
        GridGeometry::worldToGrid(rayEnd, interpolatedPos, interpolatedOri)};

    const RayIntersectionResult result{grid.intersectRay(localStart, localEnd)};
    if (result.t < 0.0) {
        return std::nullopt;
    }

    const glm::dvec3 intersection{
        localStart + (result.t + k_surfaceEpsilon) * (localEnd - localStart)};

    // The face that was hit is the one whose axis dominates the surface normal.
    const glm::dvec3 absNormal{glm::abs(result.surfaceNormal)};
    glm::ivec3 surfaceAxis;
    if (absNormal.x >= absNormal.y && absNormal.x >= absNormal.z) {
        surfaceAxis = {result.surfaceNormal.x > 0.0 ? 1 : -1, 0, 0};
    } else if (absNormal.y >= absNormal.z) {
        surfaceAxis = {0, result.surfaceNormal.y > 0.0 ? 1 : -1, 0};
    } else {
        surfaceAxis = {0, 0, result.surfaceNormal.z > 0.0 ? 1 : -1};
    }

    GridRayHit hit{};
    hit.m_hitCoord = glm::ivec3{glm::floor(intersection)};
    hit.m_placeCoord = hit.m_hitCoord + surfaceAxis;
    hit.m_t = result.t;
    return hit;
}

// Shared body of the two closestHit overloads; Handle is a weak or shared Grid pointer.
template <typename Handle>
std::optional<GridRayHit> closestHitOf(const std::vector<Handle>& grids,
                                       const glm::dvec3& rayStart,
                                       const glm::dvec3& rayEnd,
                                       double timeRemainder) {
    std::optional<GridRayHit> closest{};
    for (const Handle& handle : grids) {
        std::shared_ptr<Grid> grid{};
        if constexpr (std::is_same_v<Handle, std::weak_ptr<Grid>>) {
            grid = handle.lock();
        } else {
            grid = handle;
        }
        if (!grid) {
            continue;
        }
        std::optional<GridRayHit> hit{
            intersectGrid(*grid, rayStart, rayEnd, timeRemainder)};
        if (hit && (!closest || hit->m_t < closest->m_t)) {
            hit->m_grid = grid;
            closest = hit;
        }
    }
    return closest;
}

}  // namespace

std::optional<GridRayHit> GridRaycast::closestHit(
    const std::vector<std::weak_ptr<Grid>>& grids,
    const glm::dvec3& rayStart,
    const glm::dvec3& rayEnd,
    double timeRemainder) {
    return closestHitOf(grids, rayStart, rayEnd, timeRemainder);
}

std::optional<GridRayHit> GridRaycast::closestHit(
    const std::vector<std::shared_ptr<Grid>>& grids,
    const glm::dvec3& rayStart,
    const glm::dvec3& rayEnd,
    double timeRemainder) {
    return closestHitOf(grids, rayStart, rayEnd, timeRemainder);
}
