// PlanetBaseCache.h
#pragma once

#include <filesystem>
#include <optional>
#include "PlanetBaseField.h"

/**
 * @brief The base layer's bake, kept on disk between runs.
 *
 * The bake is a pure function of its config, so a file carrying that config
 * beside its texels can be trusted while the two match and dropped when they do
 * not. Whether to use a cache at all, and where it lives, is the caller's.
 */
namespace PlanetBaseCache {

// The maps a config was baked at, or nothing when the file is absent, damaged,
// truncated, or written for a different config. A miss is not a failure; the
// caller bakes.
std::optional<PlanetBaseMaps> load(const std::filesystem::path& path,
                                   const PlanetBaseLayerConfig& config);

// Writes the bake with the config that produced it. False costs the next run a
// bake and nothing else.
bool save(const std::filesystem::path& path, const PlanetBaseLayerConfig& config,
          const PlanetBaseMaps& maps);

}   // namespace PlanetBaseCache
