// GridRaycast.h
#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <vector>

class Grid;

// Where a world-space ray first enters the cells of a grid.
struct GridRayHit {
    std::weak_ptr<Grid> m_grid{};
    glm::ivec3 m_hitCoord{0};    // cell the ray enters
    glm::ivec3 m_placeCoord{0};  // neighbour cell across the surface that was hit
    double     m_t{0.0};         // ray parameter of the hit, in [0, 1]
};

/**
 * @brief Picking of grid cells along a world-space ray
 */
class GridRaycast {
public:
    /**
     * @brief Closest cell hit across a set of grids
     *
     * Each grid is tested against its transform interpolated by timeRemainder, so a
     * hit lands on the cell the player sees rather than the one the last physics step
     * left behind.
     * @param grids Grids to test; expired entries are skipped
     * @param rayStart Ray start in world space
     * @param rayEnd Ray end in world space
     * @param timeRemainder Interpolation parameter from the graphics engine
     * @return The closest hit, or nullopt if the ray reaches no cell
     */
    static std::optional<GridRayHit> closestHit(
        const std::vector<std::weak_ptr<Grid>>& grids,
        const glm::dvec3& rayStart,
        const glm::dvec3& rayEnd,
        double timeRemainder);

    static std::optional<GridRayHit> closestHit(
        const std::vector<std::shared_ptr<Grid>>& grids,
        const glm::dvec3& rayStart,
        const glm::dvec3& rayEnd,
        double timeRemainder);
};
