// TerrainMapDump.h
#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

/**
 * @brief The lattice layers' map and its gradient, written out to be looked at.
 *
 * Written on every run from the same bakes the textures are uploaded from, and
 * overwritten in place, so what is on disk is what the body was built from
 * rather than whatever the map looked like the last time somebody thought to
 * dump it.
 *
 * The elevation image is the bake itself. The gradient is two signed floats with
 * no bound the field implies, so it is put on a scale of its own to be seen at
 * all: mid grey is flat, red is the first axis and green the second, and the
 * scale is the steepest texel in the map, which makes the image readable within
 * itself and not comparable across maps.
 */
namespace TerrainMapDump {

// Named for what they hold and written into the directory given. False if either
// could not be written.
bool writeMaps(const std::filesystem::path& directory, int resolution,
               const std::vector<std::uint16_t>& elevation,
               const std::vector<float>& gradient);

}   // namespace TerrainMapDump
