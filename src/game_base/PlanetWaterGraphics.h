// PlanetWaterGraphics.h
#pragma once

#include <cstdint>
#include <memory>
#include <optional>

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

    // Called every physics step with the camera's side of the surface at that
    // tick. A change of side starts the splash the shader plays over the view.
    void updateSplash(bool cameraUnderWater, uint64_t physicsTick);

private:
    // Hands the shader the tick the splash started at, or k_noSplash
    void writeSplashStamp(double stamp);

    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
    std::optional<bool>     m_cameraUnderWater;
    std::optional<uint64_t> m_splashStartTick;
};
