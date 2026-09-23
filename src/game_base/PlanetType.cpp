// PlanetType.cpp
#include "PlanetType.h"
#include "PlanetTypeConfig.h"
#include "src/world/CdlodCubeFaces.h"
#include "src/world/CubeSphereBounds.h"
#include "src/world/PlanetBaseCache.h"
#include "src/world/PlanetBaseDump.h"
#include "src/world/PlanetSurface.h"
#include "src/world/PlanetSurfaceGlsl.h"
#include "src/world/TerrainMapDump.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/ShaderProgram.h"
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <vector>

// The maps a config was baked at, read back when the cache holds that same config
// and baked afresh when it does not. A fresh bake is written back, and the faces
// written out as images alongside it, a bake being the only occasion they change.
// Neither write is worth failing over: the cost of both is that the next run bakes
// again.
static PlanetBaseMaps loadOrBakeBaseMaps(const std::filesystem::path& cachePath,
                                         const PlanetBaseLayerConfig& config,
                                         const PlanetBaseField& field) {
    std::optional<PlanetBaseMaps> cached{PlanetBaseCache::load(cachePath, config)};
    if (cached) {
        return std::move(*cached);
    }

    PlanetBaseMaps baked{field.bake()};
    if (!PlanetBaseCache::save(cachePath, config, baked)) {
        std::cerr << "Could not write " << cachePath << "; the next run bakes again\n";
    }
    if (!PlanetBaseDump::writeElevationFaces(".", config, baked)) {
        std::cerr << "Could not write the base layer's face images\n";
    }

    return baked;
}

PlanetType::PlanetType(GraphicsEngine* graphics, const PlanetTypeConfig& config)
    : m_graphics{graphics} {
    if (!m_graphics) {
        throw std::runtime_error("PlanetType: graphics must be non-null");
    }

    m_surface = std::make_shared<const PlanetSurface>(
        config.m_radiusMetres, config.m_tileSpanMetres, config.m_tilesPerSpan,
        config.m_reliefMetres, config.m_baseReliefMetres, config.m_terrain, config.m_base);

    // The body's figures go in front of the shader that reads them, so they are
    // written once in C++ and compiled rather than sent per frame. The snippet
    // says what to do with them and holds none of them itself.
    //
    // The snippet is read each time this is asked, so a shader reload picks it up
    // as edited. The figures are taken once: they are fixed at startup, and a
    // reload is for the GLSL rather than for the shape of the body.
    m_cdlodSurface = m_graphics->createCdlodSurface(
        [shape = planetSurfaceGlsl(*m_surface)] {
            return shape
                 + ShaderProgram::loadTextFileFromPath(
                       "../src/world/lattice_surface.glsl");
        });

    const std::vector<uint16_t> noiseBake{m_surface->bakeElevation()};
    const std::vector<float> gradientBake{m_surface->bakeGradient()};

    // Beside the binary, refreshed every run: the map is generated at startup
    // from the config, so an image of it is only ever worth looking at if it
    // came from the run being looked at. Not worth failing over.
    if (!TerrainMapDump::writeMaps(".", m_surface->mapResolution(), noiseBake,
                                   gradientBake)) {
        std::cerr << "Could not write the terrain map images\n";
    }

    TextureSpec mapSpec{};
    mapSpec.m_width = m_surface->mapResolution();
    mapSpec.m_height = m_surface->mapResolution();
    // The fragment stage samples both maps once per pixel, so a body small on
    // screen would otherwise stride whole texels between neighbouring pixels:
    // aliased normals, and a working set too large for the texture cache. The
    // vertex stage takes level 0 regardless and is unaffected.
    mapSpec.m_generateMipmaps = true;
    mapSpec.m_format = TextureFormat::R16;
    mapSpec.m_pixels = noiseBake.data();
    m_graphics->setCdlodSurfaceTexture(m_cdlodSurface, "u_noiseMap", mapSpec);

    mapSpec.m_format = TextureFormat::RG16F;
    mapSpec.m_pixels = gradientBake.data();
    m_graphics->setCdlodSurfaceTexture(m_cdlodSurface, "u_gradientMap", mapSpec);

    // The base layer's pair, kept beside the binary between runs: the bake costs
    // seconds, and tuning the octaves never touches the config that drives it. A
    // file written for a different config, or cut short mid-write, reads as a
    // miss and is baked over.
    const PlanetBaseMaps baseMaps{
        loadOrBakeBaseMaps("planet_base.cache", config.m_base, m_surface->baseField())};

    CubeTextureSpec baseSpec{};
    // Read from orbit as well as from the ground, where a whole face of these
    // maps falls inside a pixel.
    baseSpec.m_generateMipmaps = true;

    baseSpec.m_size = m_surface->baseField().elevationResolution();
    baseSpec.m_format = TextureFormat::R16;
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        baseSpec.m_faces[face] = baseMaps.m_elevation[face].data();
    }
    m_graphics->setCdlodSurfaceCubeTexture(m_cdlodSurface, "u_baseElevationMap", baseSpec);

    // Smaller than the elevation, the filter's own coarseness being what sizes
    // that one and a slope having no such pressure.
    baseSpec.m_size = m_surface->baseField().slopeResolution();
    baseSpec.m_format = TextureFormat::RGB16F;
    for (int face{0}; face < PlanetBaseMaps::k_faceCount; ++face) {
        baseSpec.m_faces[face] = baseMaps.m_slope[face].data();
    }
    m_graphics->setCdlodSurfaceCubeTexture(m_cdlodSurface, "u_baseGradientMap", baseSpec);

    m_bounds = std::make_shared<const CubeSphereBounds>(m_surface);
}

PlanetType::~PlanetType() {
    // The engine keeps the surface alive until its last instance is removed
    m_graphics->removeCdlodSurface(m_cdlodSurface);
}

std::weak_ptr<CdlodInstance> PlanetType::createCdlodInstance(int ssboIndex) const {
    return m_graphics->createCdlodInstance(
        ssboIndex, CdlodConfig{}, CdlodCubeFaces::cubeRootFrames(m_surface->radius()),
        m_bounds, m_cdlodSurface);
}
