// BlockGraphics.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class BlockResources;
class Geometry;
class Instance;

// Per-instance rendering for a special block (thruster, cockpit, etc.).
// Attaches one OBJ instance to the grid's SSBO slot so it moves with the grid.
// pivot is the centre of rotation in model space (e.g. {0.5,0.5,0.5} for a 1-cell
// anchor, {1.0,1.0,1.0} for a 2×2×2 block anchored at one corner).
class BlockGraphics {
public:
    BlockGraphics(BlockResources* resources,
                  int ssboIndex,
                  const glm::ivec3& anchorCoord,
                  const glm::dquat& orientation,
                  const glm::dvec3& modelCentre);
    ~BlockGraphics();

    BlockGraphics(const BlockGraphics&) = delete;
    BlockGraphics& operator=(const BlockGraphics&) = delete;

private:
    BlockResources*         m_resources;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
};
