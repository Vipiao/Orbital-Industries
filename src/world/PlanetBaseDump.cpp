// PlanetBaseDump.cpp
#include "PlanetBaseDump.h"
#include <cassert>
#include <fstream>
#include <string>
#include <vector>

namespace {

// GL's face order, as the maps are stored in.
const char* k_faceNames[PlanetBaseMaps::k_faceCount]{"px", "nx", "py", "ny", "pz", "nz"};

bool writeFace(const std::filesystem::path& path, int resolution,
               const std::vector<std::uint16_t>& texels) {
    // The rows below are walked by the resolution, not by what the face holds.
    assert(texels.size() == static_cast<std::size_t>(resolution) * resolution &&
           "A face must hold exactly the texels its resolution names");

    std::ofstream file{path, std::ios::binary | std::ios::trunc};
    if (!file) {
        return false;
    }

    file << "P5\n" << resolution << ' ' << resolution << "\n65535\n";

    // PGM puts the high byte first whatever the machine does, so the samples are
    // swapped into a row at a time rather than written where they lie.
    std::vector<char> row(static_cast<std::size_t>(resolution) * 2);
    for (int y{0}; y < resolution; ++y) {
        const std::uint16_t* source{texels.data() + static_cast<std::size_t>(y) * resolution};
        for (int x{0}; x < resolution; ++x) {
            row[static_cast<std::size_t>(x) * 2] = static_cast<char>(source[x] >> 8);
            row[static_cast<std::size_t>(x) * 2 + 1] = static_cast<char>(source[x] & 0xFF);
        }
        file.write(row.data(), static_cast<std::streamsize>(row.size()));
    }

    return static_cast<bool>(file);
}

}   // namespace

bool PlanetBaseDump::writeElevationFaces(const std::filesystem::path& directory,
                                         const PlanetBaseLayerConfig& config,
                                         const PlanetBaseMaps& maps) {
    bool written{true};
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        const std::filesystem::path path{
            directory / (std::string{"planet_base_"} + k_faceNames[face] + ".pgm")};
        written = writeFace(path, config.m_elevationResolution, maps.m_elevation[face]) && written;
    }
    return written;
}
