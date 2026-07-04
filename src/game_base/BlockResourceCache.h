// BlockResourceCache.h
#pragma once

#include "CellType.h"
#include "BlockResources.h"
#include <memory>
#include <unordered_map>

class GraphicsEngine;

// Owns the shared geometry and textures for every special block type
// (thruster, cockpit, ...). All models are loaded once, up front, at
// construction so that placing a block never triggers a synchronous OBJ/texture
// load (no frame spike). A single cache is shared by every Grid, so identical
// geometry is not re-parsed per grid.
class BlockResourceCache {
public:
    explicit BlockResourceCache(GraphicsEngine* graphics);

    BlockResourceCache(const BlockResourceCache&) = delete;
    BlockResourceCache& operator=(const BlockResourceCache&) = delete;

    // Returns the shared resources for a block type, or nullptr if the type has
    // no instanced model.
    BlockResources* get(CellType type) const;

private:
    std::unordered_map<CellType, std::unique_ptr<BlockResources>> m_resources;
};
