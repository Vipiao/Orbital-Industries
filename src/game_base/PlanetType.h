// PlanetType.h
#pragma once

#include <memory>

class GraphicsEngine;
class PlanetSurface;
class ICdlodPatchBounds;
struct PlanetTypeConfig;
struct CdlodSurface;
struct CdlodInstance;

/**
 * @brief One planet shape, shared by every planet wearing it
 *
 * Generates the maps once and uploads them onto a single CDLOD surface, whose
 * snippet has the figures compiled in. Every planet of this type is an instance
 * of that surface, so all of them are drawn in one call.
 */
class PlanetType {
public:
    PlanetType(GraphicsEngine* graphics, const PlanetTypeConfig& config);
    ~PlanetType();

    PlanetType(const PlanetType&) = delete;
    PlanetType& operator=(const PlanetType&) = delete;

    const std::shared_ptr<const PlanetSurface>& getSurface() const { return m_surface; }

    // An instance of this shape drawn through the given SSBO slot. The caller
    // removes it through the graphics engine.
    std::weak_ptr<CdlodInstance> createCdlodInstance(int ssboIndex) const;

private:
    GraphicsEngine* m_graphics;
    std::shared_ptr<const PlanetSurface> m_surface;
    std::weak_ptr<CdlodSurface> m_cdlodSurface;
    // Where any patch of this shape can render, the same for every instance
    std::shared_ptr<const ICdlodPatchBounds> m_bounds;
};
