// PlanetTypeConfig.h
#pragma once

#include "math/TileableNoiseMap.h"
#include "src/world/PlanetBaseField.h"
#include "PlanetWaterConfig.h"
#include <optional>

// Everything that decides a planet's shape, handed over once at creation. The
// figures are those PlanetSurface takes.
struct PlanetTypeConfig {
    double m_radiusMetres{};
    double m_tileSpanMetres{};
    double m_tilesPerSpan{};
    double m_reliefMetres{};
    double m_baseReliefMetres{};
    TileableNoiseMapConfig m_terrain{};
    PlanetBaseLayerConfig m_base{};
    // Empty for a dry planet
    std::optional<PlanetWaterConfig> m_water{};
};
