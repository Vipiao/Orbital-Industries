// PlanetBaseCache.cpp
#include "PlanetBaseCache.h"
#include "utils/ByteStream.h"
#include <cassert>
#include <fstream>

namespace {

// Tells this cache from anything else that landed on the name; the version
// retires a layout change, so an older file reads as a miss and not as texels.
constexpr std::uint32_t k_magic{0x504C4243};   // 'PLBC'
constexpr std::uint32_t k_version{1};

// Field by field rather than a memcmp, whose padding carries whatever the stack
// held.
bool sameConfig(const PlanetBaseLayerConfig& a, const PlanetBaseLayerConfig& b) {
    return a.m_elevationResolution == b.m_elevationResolution &&
           a.m_slopeResolution == b.m_slopeResolution && a.m_reliefMetres == b.m_reliefMetres &&
           a.m_octaveCount == b.m_octaveCount && a.m_baseFrequency == b.m_baseFrequency &&
           a.m_gain == b.m_gain && a.m_seed == b.m_seed;
}

// Texels one face holds. The config fixes these, so the payload carries no
// lengths of its own and a file whose config does not match is dropped before
// its texels are reached.
std::size_t elevationTexels(const PlanetBaseLayerConfig& config) {
    return static_cast<std::size_t>(config.m_elevationResolution) * config.m_elevationResolution;
}

std::size_t slopeFloats(const PlanetBaseLayerConfig& config) {
    return static_cast<std::size_t>(config.m_slopeResolution) * config.m_slopeResolution * 3;
}

}   // namespace

std::optional<PlanetBaseMaps> PlanetBaseCache::load(const std::filesystem::path& path,
                                                    const PlanetBaseLayerConfig& config) {
    std::ifstream file{path, std::ios::binary | std::ios::ate};
    if (!file) {
        return std::nullopt;
    }

    const std::streamoff size{file.tellg()};
    if (size <= 0) {
        return std::nullopt;
    }
    file.seekg(0);

    std::vector<std::byte> blob(static_cast<std::size_t>(size));
    if (!file.read(reinterpret_cast<char*>(blob.data()), size)) {
        return std::nullopt;
    }

    ByteReader reader{blob};
    std::uint32_t magic{0};
    std::uint32_t version{0};
    PlanetBaseLayerConfig stamp{};
    if (!reader.read(magic) || !reader.read(version) || !reader.read(stamp)) {
        return std::nullopt;
    }
    if (magic != k_magic || version != k_version || !sameConfig(stamp, config)) {
        return std::nullopt;
    }

    // Every read is checked, so a file cut short by a crash mid-write falls out
    // here and is baked over rather than read as terrain.
    PlanetBaseMaps maps{};
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        maps.m_elevation[face].resize(elevationTexels(config));
        if (!reader.readArray(maps.m_elevation[face].data(), elevationTexels(config))) {
            return std::nullopt;
        }
    }
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        maps.m_slope[face].resize(slopeFloats(config));
        if (!reader.readArray(maps.m_slope[face].data(), slopeFloats(config))) {
            return std::nullopt;
        }
    }

    return maps;
}

bool PlanetBaseCache::save(const std::filesystem::path& path, const PlanetBaseLayerConfig& config,
                           const PlanetBaseMaps& maps) {
    // Load derives its lengths from the stamp, so a face of some other size would
    // leave stamp and contents disagreeing and slide every later face off its
    // start rather than fail.
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        assert(maps.m_elevation[face].size() == elevationTexels(config) &&
               maps.m_slope[face].size() == slopeFloats(config) &&
               "The maps must be the size the config stamped beside them names");
    }

    ByteWriter writer{};
    writer.write(k_magic);
    writer.write(k_version);
    writer.write(config);
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        writer.writeArray(maps.m_elevation[face].data(), elevationTexels(config));
    }
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        writer.writeArray(maps.m_slope[face].data(), slopeFloats(config));
    }

    const std::vector<std::byte> blob{writer.take()};
    std::ofstream file{path, std::ios::binary | std::ios::trunc};
    if (!file) {
        return false;
    }
    file.write(reinterpret_cast<const char*>(blob.data()),
               static_cast<std::streamsize>(blob.size()));
    return static_cast<bool>(file);
}
