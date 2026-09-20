// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>

namespace {

// The snippet's k_octaves: frequency multiplier, amplitude as a fraction of the
// relief, and a shift in tiles. Sixteen to a step in frequency, four to a step
// in amplitude, so each layer is four times the rise over run of the one above.
//
// The root of frequency rather than frequency itself: the reciprocal would give
// every layer the same slope, where real ground has gentle big landforms and
// steep small ones. Three steps reach thirty-one degrees, the angle loose ground
// gives way at, past which roughening buys nothing.
//
// Amplitude is not a slope. A tile does not rise by its own range across its own
// width -- the map carries eight layers of its own, averaging nearly three times
// that -- so rise over run is amplitude over tile times what the map does.
// Measured, as averages:
//
//    49.8 km at 658 m        two degrees
//    3.11 km at 165 m        eight and a half degrees
//     194 m at 41.1 m        thirty-one degrees
//
// The map's worst texel runs three times its mean, putting the steepest ground
// near sixty-six. Steps this close cost span at the fine end, leaving the finest
// feature hundreds of metres; a fourth layer would buy it back, and the cell
// counts -- 64, 1024, 16384, stepping by sixteen as well -- leave room for one.
//
// The frequency need not be a power of two: what has to land exactly is the cell
// count the caller's lattice comes to, which it rounds and asserts is whole. The
// shift is what keeps the layers off one lattice: each repeats a whole number of
// times per coarser tile and would otherwise land on it with the same phase,
// every repeat reinforcing the last.
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
