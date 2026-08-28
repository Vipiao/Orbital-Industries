// PlanetBaseDump.h
#pragma once

#include <filesystem>
#include "PlanetBaseField.h"

/**
 * @brief The base layer's elevation faces, written out to be looked at.
 *
 * Elevation only: the slope map is three signed floats of body-frame direction,
 * which no greyscale carries and no eye reads.
 *
 * Sixteen-bit binary PGM -- a three-line header over the samples the map already
 * holds, so a debug dump brings no encoder with it.
 */
namespace PlanetBaseDump {

// One file per cube face, named for the axis it looks along. False if any of the
// six could not be written.
bool writeElevationFaces(const std::filesystem::path& directory,
                         const PlanetBaseLayerConfig& config, const PlanetBaseMaps& maps);

}   // namespace PlanetBaseDump
