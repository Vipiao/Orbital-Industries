// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>

namespace {

// The snippet's k_octaves: frequency multiplier, amplitude as a fraction of the
// relief, and a shift in tiles, layer by layer. What each column is for, and how
// these numbers were arrived at, is written out once in the snippet's copy; the
// two tables have to read the same or the surface stops matching its bounds.
//
// The cell counts they come to -- 64, 1024, 16384, stepping by sixteen as the
// frequencies do -- leave room for a fourth layer.
const glm::dvec4 k_octaves[TerrainDisplacement::k_octaveCount]{
    glm::dvec4{0.0256, 1.0, 0.0, 0.0}, glm::dvec4{0.4096, 1.0 / 4.0, 0.37, 0.71},
    glm::dvec4{6.5536, 1.0 / 16.0, 0.61, 0.19}};

}  // namespace

TerrainDisplacement::TerrainDisplacement(double reliefMetres, double baseReliefMetres)
    : m_relief{reliefMetres}, m_baseRelief{baseReliefMetres} {
    assert(m_relief >= 0.0 && "Negative relief would sink the terrain into the sphere");
    assert(m_baseRelief >= 0.0 && "Negative relief would sink the base layer likewise");
}

double TerrainDisplacement::octaveFrequency(int octave) const {
    assert(octave >= 0 && octave < k_octaveCount && "No such octave in the table");
    return k_octaves[octave].x;
}

glm::dvec2 TerrainDisplacement::octaveShift(int octave) const {
    assert(octave >= 0 && octave < k_octaveCount && "No such octave in the table");
    return glm::dvec2{k_octaves[octave].z, k_octaves[octave].w};
}

TerrainDisplacement::Displacement TerrainDisplacement::displacement(const Levels& levels,
                                                                    int octaveCount) const {
    assert(octaveCount >= 0 && octaveCount <= k_octaveCount &&
           "The surface cannot carry octaves the table does not hold");

    Displacement displacement{};
    displacement.m_height = m_baseRelief * levels.m_baseHeight;
    displacement.m_gradient = m_baseRelief * levels.m_baseGradient;

    for (int octave{0}; octave < octaveCount; ++octave) {
        const double amplitude{m_relief * k_octaves[octave].y};
        displacement.m_height += amplitude * levels.m_height[octave];
        displacement.m_gradient += amplitude * levels.m_gradient[octave];
    }

    return displacement;
}

double TerrainDisplacement::octaveCeiling(int octaveCount) const {
    assert(octaveCount >= 0 && octaveCount <= k_octaveCount &&
           "The surface cannot carry octaves the table does not hold");

    double amplitude{0.0};
    for (int octave{0}; octave < octaveCount; ++octave) {
        amplitude += k_octaves[octave].y;
    }

    return m_relief * amplitude;
}
