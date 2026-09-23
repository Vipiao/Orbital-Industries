// TerrainDisplacement.cpp
#include "TerrainDisplacement.h"
#include <cassert>
#include <cmath>
#include <iterator>

namespace {

// The same map laid down at several scales and added up: frequency multiplier
// and amplitude as a fraction of the relief, coarsest first, with the base layer
// leading. Frequency rises by what amplitude falls by, which the constructor
// checks and which leaves every layer the same rise over run as the one above
// it: no scale is steeper than another, so the shape a layer contributes is the
// shape the whole sum has, only smaller. A frequency stepping faster than
// amplitude would give the fine end the steep ground instead, which is how real
// landforms sit but not how a multifractal is built.
//
// Amplitude is not a slope. A tile does not rise by its own range across its own
// width -- the map carries octaves of its own, and what they come to across a
// tile is a factor in its own right -- so rise over run is amplitude over tile
// times what the map does.
//
// How often the coarsest lattice layer is laid down, which the rest are written
// as multiples of: the multiplier is how much oftener than it a layer comes, and
// the amplitude is one over that same number, so the ladder rule is on the page
// rather than only in the assert below. The scale itself need not be a power of
// two -- what has to land exactly is the cell count the lattice comes to, which
// PlanetSurface rounds and asserts is whole -- but the multipliers must be, and
// the assert there checks it.
constexpr double k_coarsestFrequency{0.0256};

const glm::dvec2 k_levels[]{glm::dvec2{0.0, 1.0},
                            glm::dvec2{k_coarsestFrequency * 1.0, 1.0},
                            glm::dvec2{k_coarsestFrequency * 8.0, 1.0 / 8.0},
                            glm::dvec2{k_coarsestFrequency * 64.0, 1.0 / 64.0},
                            glm::dvec2{k_coarsestFrequency * 512.0, 1.0 / 512.0}};

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

    // A step is free in size but must rise in frequency by what it falls by in
    // amplitude, or the level it lands on is steeper than its neighbours. Every
    // step is a power of two, so these hold to the bit. The base layer lays down
    // no tiles, so it is left out.
    for (int level{k_firstLatticeLevel + 1}; level < k_levelCount; ++level) {
        const double step{k_levels[level].x / k_levels[level - 1].x};
        assert(step > 1.0 && "Each layer is laid down oftener than the one above it");
        assert(k_levels[level].y * step == k_levels[level - 1].y &&
               "Frequency rises by what amplitude falls by");
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
        case Synthesis::TURBULENCE_POWER:
            return turbulencePower(levels);
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

TerrainDisplacement::Displacement TerrainDisplacement::turbulencePower(
    const Levels& levels) const {
    double latticeRelief{0.0};
    double latticeSum{0.0};
    glm::dvec3 latticeGradient{0.0};
    for (int level{k_firstLatticeLevel}; level < k_levelCount; ++level) {
        const double relief{levelRelief(level)};
        latticeRelief += relief;
        latticeSum += relief * levels.m_height[level];
        latticeGradient += relief * levels.m_gradient[level];
    }

    // The sum as a fraction of the whole it could have come to, so the exponent
    // remaps that fraction onto itself and the layers still stand their own
    // relief between them and no more. Lattice layers at no relief carry nothing
    // to take a fraction of.
    //
    // The magnitude of it, because the map is unsigned but a level's reading is
    // not: the lattice blend holds its contrast flat with weights that sum past
    // one and takes the overshoot back off against the field's mean, so a
    // reading below that mean lands below zero, where pow is undefined. Folding
    // it costs a crease along the level the mean sits at, at the few centimetres
    // of ground that reach under it.
    const double carried{latticeRelief > 0.0 ? std::abs(latticeSum / latticeRelief)
                                             : 0.0};

    // The chain rule on relief times carried to the exponent: the exponent comes
    // down, and the relief cancels against the one the fraction was taken over.
    // The fold's own turn is left out, at the same few centimetres.
    const double slopeFactor{k_turbulenceExponent *
                             std::pow(carried, k_turbulenceExponent - 1.0)};

    const double baseRelief{levelRelief(k_baseLevel)};

    Displacement displacement{};
    displacement.m_height = baseRelief * levels.m_height[k_baseLevel] +
                            latticeRelief * std::pow(carried, k_turbulenceExponent);
    displacement.m_gradient =
        baseRelief * levels.m_gradient[k_baseLevel] + slopeFactor * latticeGradient;

    return displacement;
}
