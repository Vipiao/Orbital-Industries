// BlockResourceCache.h
#pragma once

#include "CellType.h"
#include "BlockResources.h"
#include <memory>
#include <unordered_map>

class GraphicsEngine;
class Geometry;

// Owns the shared geometry and textures for every special block type
// (thruster, cockpit, ...). All models are loaded once, up front, at
// construction so that placing a block never triggers a synchronous OBJ/texture
// load (no frame spike). A single cache is shared by every Grid, so identical
// geometry is not re-parsed per grid. Also holds the shared ion-plume ray-volume
// material and proxy geometry, likewise loaded once.
class BlockResourceCache {
public:
    explicit BlockResourceCache(GraphicsEngine* graphics);
    ~BlockResourceCache();

    BlockResourceCache(const BlockResourceCache&) = delete;
    BlockResourceCache& operator=(const BlockResourceCache&) = delete;

    // Returns the shared resources for a block type, or nullptr if the type has
    // no instanced model.
    BlockResources* get(CellType type) const;

    // Shared ion-plume proxy geometry (already bound to the plume material).
    std::weak_ptr<Geometry> getPlumeGeometry() const { return m_plumeGeometry; }

private:
    GraphicsEngine* m_graphics{nullptr};
    std::unordered_map<CellType, std::unique_ptr<BlockResources>> m_resources;
    size_t m_plumeMaterial{0};
    std::weak_ptr<Geometry> m_plumeGeometry;
};
