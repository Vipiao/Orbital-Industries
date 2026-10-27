// PlanetType.h
#pragma once

#include "PlanetWaterConfig.h"
#include "src/world/PropPatchGrid.h"
#include <memory>
#include <optional>
#include <string>
#include <vector>

class GraphicsEngine;
class PlanetSurface;
class ICdlodPatchBounds;
class Texture;
struct PlanetTypeConfig;
struct CdlodSurface;
struct CdlodInstance;
struct ScatterSet;

/**
 * @brief One planet shape, shared by every planet wearing it
 *
 * Generates the maps once and uploads them onto a single CDLOD surface, whose
 * snippet has the figures compiled in. Every planet of this type is an instance
 * of that surface, so all of them are drawn in one call.
 *
 * The props' scatter set is the type's in the same way, reading the same maps,
 * and the grid of prop patches with it.
 */
class PlanetType {
public:
    PlanetType(GraphicsEngine* graphics, const PlanetTypeConfig& config);
    ~PlanetType();

    PlanetType(const PlanetType&) = delete;
    PlanetType& operator=(const PlanetType&) = delete;

    const std::shared_ptr<const PlanetSurface>& getSurface() const { return m_surface; }
    const std::optional<PlanetWaterConfig>& getWater() const { return m_water; }
    std::weak_ptr<CdlodSurface> getCdlodSurface() const { return m_cdlodSurface; }
    std::weak_ptr<ScatterSet> getPropSet() const { return m_propSet; }
    const PropPatchGrid& getPropPatchGrid() const { return m_propPatchGrid; }

    // An instance of this shape drawn through the given SSBO slot. The caller
    // removes it through the graphics engine.
    std::weak_ptr<CdlodInstance> createCdlodInstance(int ssboIndex) const;

private:
    // A map of the ground, and the name the snippets declare its sampler under
    struct GroundMap {
        std::string m_samplerName;
        std::weak_ptr<Texture> m_texture;
    };

    // To the ground and the props, kept to remove
    void shareGroundMap(const std::string& samplerName, std::weak_ptr<Texture> texture);

    GraphicsEngine* m_graphics;
    std::shared_ptr<const PlanetSurface> m_surface;
    std::optional<PlanetWaterConfig> m_water;
    std::weak_ptr<CdlodSurface> m_cdlodSurface;
    // Where any patch of this shape can render, the same for every instance
    std::shared_ptr<const ICdlodPatchBounds> m_bounds;
    PropPatchGrid m_propPatchGrid;
    std::weak_ptr<ScatterSet> m_propSet;
    // Shared by the ground and the props
    std::vector<GroundMap> m_groundMaps;
};
