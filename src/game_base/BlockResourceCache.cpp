// BlockResourceCache.cpp
#include "BlockResourceCache.h"
#include "thruster/ThrusterBlock.h"
#include "cockpit/CockpitBlock.h"
#include "graphics/GraphicsEngine.h"
#include <stdexcept>

BlockResourceCache::BlockResourceCache(GraphicsEngine* graphics)
    : m_graphics{graphics}
{
    if (!graphics) {
        throw std::runtime_error{"BlockResourceCache: GraphicsEngine cannot be null"};
    }

    // Preload every special-block model up front so first placement is spike-free.
    m_resources.emplace(CellType::THRUSTER,
        std::make_unique<BlockResources>(
            graphics,
            ThrusterBlock::geometryParts(),
            std::string{ThrusterBlock::COLOR_TEX_PATH},
            std::string{ThrusterBlock::NORMAL_TEX_PATH}));

    m_resources.emplace(CellType::COCKPIT,
        std::make_unique<BlockResources>(
            graphics,
            CockpitBlock::geometryParts(),
            std::string{CockpitBlock::COLOR_TEX_PATH},
            std::string{CockpitBlock::NORMAL_TEX_PATH}));

    // Shared ion-plume ray-volume material (injected placeholder body) and proxy.
    m_plumeMaterial = graphics->createRayVolumeMaterial(
        "../media/models/ion_plume/plume_body.glsl");
    m_plumeGeometry = graphics->createRayVolumeGeometry(
        "../media/models/ion_plume/model.obj", m_plumeMaterial);
}

BlockResourceCache::~BlockResourceCache() {
    if (m_graphics && !m_plumeGeometry.expired()) {
        m_graphics->releaseRayVolumeGeometry(m_plumeGeometry);
    }
}

BlockResources* BlockResourceCache::get(CellType type) const {
    auto it = m_resources.find(type);
    return it != m_resources.end() ? it->second.get() : nullptr;
}
