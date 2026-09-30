// PlanetWaterGraphics.h
#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class Geometry;
class Instance;
struct CdlodSurface;
struct PlanetWaterConfig;

// One planet's water: a ray-volume instance on the shared shell, drawn through
// the planet's SSBO slot so it moves with the planet, and what the terrain draws
// under it.
class PlanetWaterGraphics {
public:
    PlanetWaterGraphics(GraphicsEngine* graphics, std::weak_ptr<Geometry> shellGeometry,
                        std::weak_ptr<CdlodSurface> cdlodSurface, int ssboIndex,
                        double seaLevelRadius, const PlanetWaterConfig& config);
    ~PlanetWaterGraphics();

    PlanetWaterGraphics(const PlanetWaterGraphics&) = delete;
    PlanetWaterGraphics& operator=(const PlanetWaterGraphics&) = delete;

    // Called every physics step with the camera in the planet's frame, from its
    // centre. Sets how the terrain draws the sea floor and the far sea, and a
    // change of side starts the splash.
    void updateView(const glm::dvec3& cameraLocal, const glm::dquat& bodyOrientation,
                    uint64_t physicsTick);

private:
    void updateSplash(bool cameraUnderWater, uint64_t physicsTick);
    // Hands the shader the tick the splash started at, or k_noSplash
    void writeSplashStamp(double stamp);

    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<CdlodSurface> m_cdlodSurface;
    std::weak_ptr<Instance> m_instance;
    double                  m_seaLevelRadius;
    std::optional<bool>     m_cameraUnderWater;
    std::optional<uint64_t> m_splashStartTick;
};
