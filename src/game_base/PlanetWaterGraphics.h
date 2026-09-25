// PlanetWaterGraphics.h
#pragma once

#include <memory>

class GraphicsEngine;
class Geometry;
class Instance;
struct PlanetWaterConfig;

// One planet's water: a ray-volume instance on the shared shell, drawn through
// the planet's SSBO slot so it moves with the planet.
class PlanetWaterGraphics {
public:
    PlanetWaterGraphics(GraphicsEngine* graphics, std::weak_ptr<Geometry> shellGeometry,
                        int ssboIndex, double seaLevelRadius,
                        const PlanetWaterConfig& config);
    ~PlanetWaterGraphics();

    PlanetWaterGraphics(const PlanetWaterGraphics&) = delete;
    PlanetWaterGraphics& operator=(const PlanetWaterGraphics&) = delete;

private:
    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
};
