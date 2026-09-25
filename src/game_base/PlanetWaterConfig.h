// PlanetWaterConfig.h
#pragma once

#include <glm/glm.hpp>

// A sphere of water filling the terrain up to sea level. Light through it decays
// exponentially with the distance travelled, at its own rate in each channel.
struct PlanetWaterConfig {
    // Above the planet's radius, the same datum the terrain rises from
    double m_seaLevelMetres{};
    // Per RGB channel; what water takes out of the view by absorbing and
    // scattering both
    glm::dvec3 m_absorptionPerMetre{};
    // What the water looks like where it is too deep to see through, lit by the
    // full ambient light
    glm::dvec3 m_color{};
    // Depth below sea level at which the scene's light has fallen to 1/e with the
    // camera there, for the sun's direct light and for the ambient light
    double m_directFalloffMetres{};
    double m_ambientFalloffMetres{};
};
