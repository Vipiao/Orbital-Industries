// BlockResourceCache.cpp
#include "BlockResourceCache.h"
#include "thruster/ThrusterBlock.h"
#include "cockpit/CockpitBlock.h"
#include <stdexcept>

BlockResourceCache::BlockResourceCache(GraphicsEngine* graphics) {
    if (!graphics) {
        throw std::runtime_error{"BlockResourceCache: GraphicsEngine cannot be null"};
    }

    // Preload every special-block model up front so first placement is spike-free.
    m_resources.emplace(CellType::THRUSTER,
        std::make_unique<BlockResources>(
            graphics,
            std::string{ThrusterBlock::GEOMETRY_PATH},
            std::string{ThrusterBlock::COLOR_TEX_PATH},
            std::string{ThrusterBlock::NORMAL_TEX_PATH}));

    m_resources.emplace(CellType::COCKPIT,
        std::make_unique<BlockResources>(
            graphics,
            std::string{CockpitBlock::GEOMETRY_PATH},
            std::string{CockpitBlock::COLOR_TEX_PATH},
            std::string{CockpitBlock::NORMAL_TEX_PATH}));
}

BlockResources* BlockResourceCache::get(CellType type) const {
    auto it = m_resources.find(type);
    return it != m_resources.end() ? it->second.get() : nullptr;
}
