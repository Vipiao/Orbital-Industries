// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>

namespace {

// The snippet's k_levels: frequency multiplier, amplitude as a fraction of the
// relief, and a shift in tiles, layer by layer, with the base layer last. What
// each column is for, and how these numbers were arrived at, is written out once
// in the snippet's copy; the two tables have to read the same or the surface
// stops matching its bounds.
//
// The cell counts the octaves come to -- 64, 1024, 16384, stepping by sixteen as
// the frequencies do -- leave room for a fourth.
const glm::dvec4 k_levels[TerrainDisplacement::k_levelCount]{
    glm::dvec4{0.0256, 1.0, 0.0, 0.0}, glm::dvec4{0.4096, 1.0 / 4.0, 0.37, 0.71},
    glm::dvec4{6.5536, 1.0 / 16.0, 0.61, 0.19}, glm::dvec4{0.0, 1.0, 0.0, 0.0}};

}  // namespace

TerrainDisplacement::TerrainDisplacement(double reliefMetres, double baseReliefMetres)
    : m_relief{reliefMetres}, m_baseRelief{baseReliefMetres} {
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");
    assert(m_baseRelief >= 0.0 && "Negative relief would sink the base layer likewise");
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
