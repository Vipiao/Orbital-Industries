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

}  // namespace

std::string planetSurfaceGlsl(const PlanetSurface& surface) {
    const TerrainDisplacement& displacement{surface.displacement()};
    const int levelCount{TerrainDisplacement::k_levelCount};

    std::ostringstream glsl{};
    glsl << "// Written by planetSurfaceGlsl.\n"
         << "const float k_radiusMetres = " << glslFloat(surface.radius()) << ";\n\n"
         << "// One repeat of the map, as the two whole numbers its width is the ratio\n"
         << "// of: the width itself is not exact in a float, and a plane coordinate\n"
         << "// stands thousands of tiles out.\n"
         << "const float k_tileSpanMetres = " << glslFloat(surface.tileSpanMetres())
         << ";\n"
         << "const float k_tilesPerSpan = " << glslFloat(surface.tilesPerSpan()) << ";\n\n"
         << "// The lattice: points a cell is bounded by, tiles of its own layer a cell\n"
         << "// spans, and levels the drawn geometry carries.\n"
         << "const int k_latticeCorners = " << PlanetSurface::k_latticeCorners << ";\n"
         << "const float k_cellTiles = " << glslFloat(PlanetSurface::k_cellTiles) << ";\n"
         << "const int k_positionLevels = " << PlanetSurface::k_positionLevels << ";\n\n"
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
         << "const int k_synthesis = "
         << static_cast<int>(TerrainDisplacement::k_synthesis) << ";\n\n"
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
