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
    const int octaveCount{TerrainDisplacement::k_octaveCount};

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
         << "// spans, and octaves the drawn geometry carries.\n"
         << "const int k_latticeCorners = " << PlanetSurface::k_latticeCorners << ";\n"
         << "const float k_cellTiles = " << glslFloat(PlanetSurface::k_cellTiles) << ";\n"
         << "const int k_positionOctaves = " << PlanetSurface::k_positionOctaves << ";\n\n"
         << "// k_levelCount layers, of which the first k_octaveCount are laid down\n"
         << "// through a lattice and the last is the base layer, which is not.\n"
         << "const int k_octaveCount = " << octaveCount << ";\n"
         << "const int k_levelCount = " << levelCount << ";\n\n"
         << "// Metres each layer stands between its floor and its ceiling. The map is\n"
         << "// unsigned, so this is also the highest it reaches.\n"
         << "const float k_levelReliefMetres[k_levelCount] = float[k_levelCount](";

    for (int level{0}; level < levelCount; ++level) {
        glsl << (level == 0 ? "\n   " : ",\n   ")
             << glslFloat(displacement.levelRelief(level));
    }

    glsl << ");\n\n"
         << "// How an octave's map is laid down: how much oftener than the field it\n"
         << "// was built at, and the tiles its coordinate is carried by first.\n"
         << "const vec3 k_octaveTiling[k_octaveCount] = vec3[k_octaveCount](";

    for (int octave{0}; octave < octaveCount; ++octave) {
        const glm::dvec2 shift{displacement.octaveShift(octave)};
        glsl << (octave == 0 ? "\n   vec3(" : ",\n   vec3(")
             << glslFloat(displacement.octaveFrequency(octave)) << ", "
             << glslFloat(shift.x) << ", " << glslFloat(shift.y) << ")";
    }
    glsl << ");\n";

    return glsl.str();
}
