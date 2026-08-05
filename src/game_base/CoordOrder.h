// CoordOrder.h
#pragma once

#include <algorithm>
#include <glm/glm.hpp>
#include <tuple>
#include <vector>

// Coordinate-sorted keys of a coord-keyed map: the canonical iteration order for
// anything whose result depends on the order cells are visited in — a folded hash,
// a floating-point sum — since a hash map's own order is unspecified.
template <typename Map>
std::vector<glm::ivec3> sortedCoords(const Map& cells) {
    std::vector<glm::ivec3> coords{};
    coords.reserve(cells.size());
    for (const auto& [coord, value] : cells) {
        coords.push_back(coord);
    }
    std::sort(coords.begin(), coords.end(), [](const glm::ivec3& a, const glm::ivec3& b) {
        return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    });
    return coords;
}
