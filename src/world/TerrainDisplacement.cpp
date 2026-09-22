// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>
#include <iterator>

namespace {

// The same map laid down at several scales and added up: frequency multiplier
// and amplitude as a fraction of the relief, coarsest first, with the base layer
// leading. The steps between rows are what the constructor checks; frequency
// rises by what amplitude falls by, which leaves every layer the same rise over
// run as the one above it.
//
// Stepping them together is what makes the table self-similar: no scale is
// steeper than another, so the shape a layer contributes is the shape the whole
// sum has, only smaller. A frequency stepping faster than amplitude would give
// the fine end the steep ground instead, which is how real landforms sit but not
// how a multifractal is built.
//
// Amplitude is not a slope. A tile does not rise by its own range across its own
// width -- the map carries octaves of its own, and what they come to across a
// tile is a factor in its own right -- so rise over run is amplitude over tile
// times what the map does. The folded map averages 4.26 and its worst texel runs
// 10.5, which puts all three near thirteen degrees on average and twenty-nine at
// the steepest: 49.8 km at 2632 m, 12.4 km at 658 m, 3.11 km at 165 m.
//
// The frequency need not be a power of two: what has to land exactly is the cell
// count the lattice comes to, which PlanetSurface rounds and asserts is whole.
const glm::dvec2 k_levels[]{glm::dvec2{0.0, 1.0}, glm::dvec2{0.0256, 1.0},
                            glm::dvec2{0.1024, 1.0 / 4.0},
                            glm::dvec2{0.4096, 1.0 / 16.0}};

// Sized by the rows written rather than by the count, so a row added or dropped
// here without the count following is a compile error. Sized by the count, too
// few rows would leave the rest zeroed and reach the constructor's asserts as a
// broken ladder, and too many would not fit.
static_assert(std::size(k_levels) == TerrainDisplacement::k_levelCount,
              "The table's rows and the count the levels are read by must agree");

}  // namespace

TerrainDisplacement::TerrainDisplacement(double reliefMetres, double baseReliefMetres)
    : m_relief{reliefMetres}, m_baseRelief{baseReliefMetres} {
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");
    assert(m_baseRelief >= 0.0 && "Negative relief would sink the base layer likewise");

    // Both steps are exact in binary, so these hold to the bit and an edit that
    // breaks the progression is caught here rather than read off the surface.
    for (int level{k_firstLatticeLevel + 1}; level < k_levelCount; ++level) {
        assert(k_levels[level].x == 4.0 * k_levels[level - 1].x &&
               "Four to a step in frequency");
        assert(k_levels[level].y == k_levels[level - 1].y / 4.0 &&
               "Four to a step in amplitude");
    }

    // Read by direction rather than through a lattice, so it lays down no tiles,
    // and its amplitude is the whole of its own relief.
    assert(k_levels[k_baseLevel].x == 0.0 && k_levels[k_baseLevel].y == 1.0 &&
           "The base layer's row carries no tiling and its whole relief");
}

double TerrainDisplacement::levelFrequency(int level) const {
    assert(level >= 0 && level < k_levelCount && "No such level in the table");
    return k_levels[level].x;
}

double TerrainDisplacement::levelRelief(int level) const {
    assert(level >= 0 && level < k_levelCount && "No such level in the table");
    const double relief{level == k_baseLevel ? m_baseRelief : m_relief};
    return relief * k_levels[level].y;
}

TerrainDisplacement::Displacement TerrainDisplacement::displacement(
    const Levels& levels) const {
    switch (k_synthesis) {
        case Synthesis::FBM:
            return fbm(levels);
        case Synthesis::RIDGED_MULTIFRACTAL:
            return ridgedMultifractal(levels);
    }

    assert(false && "No such synthesis");
    return Displacement{};
}

TerrainDisplacement::Displacement TerrainDisplacement::fbm(const Levels& levels) const {
    Displacement displacement{};
    for (int level{0}; level < k_levelCount; ++level) {
        const double relief{levelRelief(level)};
        displacement.m_height += relief * levels.m_height[level];
        displacement.m_gradient += relief * levels.m_gradient[level];
    }

    return displacement;
}

TerrainDisplacement::Displacement TerrainDisplacement::ridgedMultifractal(
    const Levels& levels) const {
    Displacement displacement{};
    for (int level{0}; level < k_levelCount; ++level) {
        const double relief{levelRelief(level)};
        displacement.m_height -= relief * levels.m_height[level];
        displacement.m_gradient -= relief * levels.m_gradient[level];
    }

    return displacement;
}
