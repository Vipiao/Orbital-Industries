// PlanetSurfaceGlsl.cpp
#include "PlanetSurfaceGlsl.h"
#include <array>
#include <cassert>
#include <charconv>
#include <sstream>

namespace {

// One number as GLSL source, at the shortest spelling that reads back as the
// number itself, so the shader compiles the figure it was given rather than a
// rounding of it.
std::string glslFloat(double value) {
    std::array<char, 32> buffer{};
    const std::to_chars_result printed{
        std::to_chars(buffer.data(), buffer.data() + buffer.size(), value)};
    assert(printed.ec == std::errc{} && "A figure must fit its buffer");

    std::string text{buffer.data(), printed.ptr};
    // GLSL reads a literal with no point as an int, which will not initialise a
    // float without complaint. Everything written here is a float.
    if (text.find_first_of(".eE") == std::string::npos) {
        text += ".0";
    }

    double readBack{0.0};
    assert(std::from_chars(text.data(), text.data() + text.size(), readBack).ec ==
               std::errc{} &&
           readBack == value && "A figure must read back as what it was printed from");

    return text;
}

std::string glslVec3(const glm::dvec3& value) {
    return "vec3(" + glslFloat(value.x) + ", " + glslFloat(value.y) + ", " +
           glslFloat(value.z) + ")";
}

}  // namespace

std::string planetSurfaceGlsl(const PlanetSurface& surface, double shoreMetres,
                              const glm::dvec3& waterAbsorptionPerMetre,
                              const glm::dvec3& waterColor, const glm::dvec3& bodyTint) {
    const TerrainDisplacement& displacement{surface.getDisplacement()};
    const int levelCount{TerrainDisplacement::k_levelCount};

    std::ostringstream glsl{};
    glsl << "// Written by planetSurfaceGlsl.\n"
         << "const float k_radiusMetres = " << glslFloat(surface.getRadius()) << ";\n"
         << "// Height above the sphere the sand is laid along.\n"
         << "const float k_shoreMetres = " << glslFloat(shoreMetres) << ";\n"
         << "// The water below the shore, as the sea is drawn from far above.\n"
         << "const vec3 k_waterAbsorptionPerMetre = " << glslVec3(waterAbsorptionPerMetre)
         << ";\n"
         << "const vec3 k_waterColor = " << glslVec3(waterColor) << ";\n"
         << "// What the renderer multiplies the surface's colour by.\n"
         << "const vec3 k_bodyTint = " << glslVec3(bodyTint) << ";\n\n"
         << "// One repeat of the map, as the two whole numbers its width is the ratio\n"
         << "// of: the width itself is not exact in a float, and a plane coordinate\n"
         << "// stands thousands of tiles out.\n"
         << "const float k_tileSpanMetres = " << glslFloat(surface.getTileSpanMetres())
         << ";\n"
         << "const float k_tilesPerSpan = " << glslFloat(surface.getTilesPerSpan()) << ";\n"
         << "// What the packed slope spans either way, per unit of tile.\n"
         << "const float k_gradientScale = " << glslFloat(surface.getGradientScale())
         << ";\n\n"
         << "// The lattice: points a cell is bounded by, tiles of its own layer a cell\n"
         << "// spans, and levels the geometry and the shading each carry.\n"
         << "const int k_latticeCorners = " << PlanetSurface::k_latticeCorners << ";\n"
         << "const float k_cellTiles = " << glslFloat(PlanetSurface::k_cellTiles) << ";\n"
         << "const int k_positionLevels = " << PlanetSurface::k_positionLevels << ";\n"
         << "const int k_shadingLevels = " << PlanetSurface::k_shadingLevels << ";\n\n"
         << "// k_levelCount layers, coarsest first. k_baseLevel leads and is read by\n"
         << "// direction; the rest, from k_firstLatticeLevel on, are laid down through\n"
         << "// a lattice sized to their own tile.\n"
         << "const int k_levelCount = " << levelCount << ";\n"
         << "const int k_baseLevel = " << TerrainDisplacement::k_baseLevel << ";\n"
         << "const int k_firstLatticeLevel = "
         << TerrainDisplacement::k_firstLatticeLevel << ";\n\n"
         << "// How the layers are put together.\n"
         << "const int k_synthesisFbm = "
         << static_cast<int>(TerrainDisplacement::Synthesis::FBM) << ";\n"
         << "const int k_synthesisRidgedMultifractal = "
         << static_cast<int>(TerrainDisplacement::Synthesis::RIDGED_MULTIFRACTAL)
         << ";\n"
         << "const int k_synthesisTurbulencePower = "
         << static_cast<int>(TerrainDisplacement::Synthesis::TURBULENCE_POWER)
         << ";\n"
         << "const int k_synthesisDunes = "
         << static_cast<int>(TerrainDisplacement::Synthesis::DUNES) << ";\n"
         << "const int k_synthesis = "
         << static_cast<int>(TerrainDisplacement::k_synthesis) << ";\n\n"
         << "// What each synthesis carries besides the table, one group apiece. A\n"
         << "// median is where the middle of its sum falls, as a fraction of the\n"
         << "// relief it sums, which it takes off what it returns so they all\n"
         << "// stand at one height.\n"
         << "const float k_fbmMedian = "
         << glslFloat(TerrainDisplacement::k_fbm.m_median) << ";\n\n"
         << "const float k_ridgedMultifractalMedian = "
         << glslFloat(TerrainDisplacement::k_ridgedMultifractal.m_median) << ";\n\n"
         << "const float k_turbulencePowerExponent = "
         << glslFloat(TerrainDisplacement::k_turbulencePower.m_exponent) << ";\n"
         << "const float k_turbulencePowerMedian = "
         << glslFloat(TerrainDisplacement::k_turbulencePower.m_median) << ";\n\n"
         << "const int k_dunesLevel = " << TerrainDisplacement::k_dunes.m_level << ";\n"
         << "const float k_dunesFrequency = "
         << glslFloat(TerrainDisplacement::k_dunes.m_frequency) << ";\n"
         << "const float k_dunesCrestRounding = "
         << glslFloat(TerrainDisplacement::k_dunes.m_crestRounding) << ";\n"
         << "const float k_dunesRidgeScale = "
         << glslFloat(TerrainDisplacement::k_dunes.m_ridgeScale) << ";\n"
         << "const float k_dunesMedian = "
         << glslFloat(TerrainDisplacement::k_dunes.m_median) << ";\n\n"
         << "// Metres each layer stands between its floor and its ceiling. The map is\n"
         << "// unsigned, so this is also the highest it reaches.\n"
         << "const float k_levelReliefMetres[k_levelCount] = float[k_levelCount](";

    for (int level{0}; level < levelCount; ++level) {
        glsl << (level == 0 ? "\n   " : ",\n   ")
             << glslFloat(displacement.levelRelief(level));
    }

    glsl << ");\n\n"
         << "// How much oftener than the field it was built at a layer's map is laid\n"
         << "// down. Zero at k_baseLevel, which lays down no tiles.\n"
         << "const float k_levelFrequency[k_levelCount] = float[k_levelCount](";

    for (int level{0}; level < levelCount; ++level) {
        glsl << (level == 0 ? "\n   " : ",\n   ")
             << glslFloat(displacement.levelFrequency(level));
    }
    glsl << ");\n";

    return glsl.str();
}
