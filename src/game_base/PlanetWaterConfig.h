// PlanetWaterConfig.h
#pragma once

#include <glm/glm.hpp>

// A sphere of water filling the terrain up to sea level. Light through it decays
// exponentially with the distance travelled.
struct PlanetWaterConfig {
    // Above the planet's radius, the same datum the terrain rises from
    double m_seaLevelMetres{};
    double m_absorptionPerMetre{};
    glm::dvec3 m_color{};
};
