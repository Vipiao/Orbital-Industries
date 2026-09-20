// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>

namespace {

// The same map laid down at several scales and added up: frequency multiplier,
// amplitude as a fraction of the relief, and a shift in tiles, with the base
// layer last. The steps between rows are what the constructor checks; they come
// to each layer having four times the rise over run of the one above.
//
// The root of frequency rather than frequency itself: the reciprocal would give
// every layer the same slope, where real ground has gentle big landforms and
// steep small ones. Three steps reach thirty-one degrees, the angle loose ground
// gives way at, past which roughening buys nothing.
//
// Amplitude is not a slope. A tile does not rise by its own range across its own
// width -- the map carries eight layers of its own, averaging nearly three times
// that -- so rise over run is amplitude over tile times what the map does.
// Measured, as averages: 49.8 km at 658 m is two degrees, 3.11 km at 165 m eight
// and a half, 194 m at 41.1 m thirty-one. The map's worst texel runs three times
// its mean, putting the steepest ground near sixty-six, and that is the figure
// the quadtree's ranges answer for.
//
// Steps this close cost span at the fine end, leaving the finest feature hundreds
// of metres; a fourth layer would buy it back. The cell counts the octaves come
// to -- 64, 1024, 16384, stepping by sixteen as the frequencies do -- leave room
// for one.
//
// The frequency need not be a power of two: what has to land exactly is the cell
// count the lattice comes to, which PlanetSurface rounds and asserts is whole.
// The shift is what keeps the layers off one lattice: each repeats a whole
// number of times per coarser tile and would otherwise land on it with the same
// phase, every repeat reinforcing the last.
const glm::dvec4 k_levels[TerrainDisplacement::k_levelCount]{
    glm::dvec4{0.0256, 1.0, 0.0, 0.0}, glm::dvec4{0.4096, 1.0 / 4.0, 0.37, 0.71},
    glm::dvec4{6.5536, 1.0 / 16.0, 0.61, 0.19}, glm::dvec4{0.0, 1.0, 0.0, 0.0}};

}  // namespace

TerrainDisplacement::TerrainDisplacement(double reliefMetres, double baseReliefMetres)
    : m_relief{reliefMetres}, m_baseRelief{baseReliefMetres} {
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");
    assert(m_baseRelief >= 0.0 && "Negative relief would sink the base layer likewise");

    // Both steps are exact in binary, so these hold to the bit and an edit that
    // breaks the progression is caught here rather than read off the surface.
    for (int octave{1}; octave < k_octaveCount; ++octave) {
        assert(k_levels[octave].x == 16.0 * k_levels[octave - 1].x &&
               "Sixteen to a step in frequency");
        assert(k_levels[octave].y == k_levels[octave - 1].y / 4.0 &&
               "Four to a step in amplitude");
        assert((k_levels[octave].z != 0.0 || k_levels[octave].w != 0.0) &&
               "An octave past the coarsest needs a shift to stay off its lattice");
    }

    // Read by direction rather than through a lattice, so it lays down no tiles
    // and takes no shift, and its amplitude is the whole of its own relief.
    assert(k_levels[k_octaveCount].x == 0.0 && k_levels[k_octaveCount].z == 0.0 &&
           k_levels[k_octaveCount].w == 0.0 && k_levels[k_octaveCount].y == 1.0 &&
           "The base layer's row carries no tiling and its whole relief");
}

double TerrainDisplacement::octaveFrequency(int octave) const {
    assert(octave >= 0 && octave < k_octaveCount && "No such octave in the table");
    return k_levels[octave].x;
}

glm::dvec2 TerrainDisplacement::octaveShift(int octave) const {
    assert(octave >= 0 && octave < k_octaveCount && "No such octave in the table");
    return glm::dvec2{k_levels[octave].z, k_levels[octave].w};
}

double TerrainDisplacement::levelRelief(int level) const {
    assert(level >= 0 && level < k_levelCount && "No such level in the table");
    const double relief{level == k_octaveCount ? m_baseRelief : m_relief};
    return relief * k_levels[level].y;
}

TerrainDisplacement::Displacement TerrainDisplacement::displacement(
    const Levels& levels) const {
    Displacement displacement{};
    for (int level{0}; level < k_levelCount; ++level) {
        const double relief{levelRelief(level)};
        displacement.m_height += relief * levels.m_height[level];
        displacement.m_gradient += relief * levels.m_gradient[level];
    }

    return displacement;
}
