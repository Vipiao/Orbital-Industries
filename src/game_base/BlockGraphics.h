// BlockGraphics.h
#pragma once

#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class BlockResources;
class Geometry;
class Instance;

// Per-instance rendering for a special block (thruster, cockpit, etc.).
// Attaches one OBJ instance per block part to the grid's SSBO slot so the whole
// block moves with the grid. All parts currently share the same transform; each
// being a separate instance leaves room for parts to move independently later
// (e.g. cockpit doors opening).
// pivot is the centre of rotation in model space (e.g. {0.5,0.5,0.5} for a
// 1-cell anchor, {1.0,1.0,1.0} for a 2×2×2 block anchored at one corner).
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
    // One rendered part: its geometry and the instance attached to it.
    struct PartInstance {
        std::weak_ptr<Geometry> geometry;
        std::weak_ptr<Instance> instance;
    };

    // Removes every attached instance and clears m_parts. Used by the destructor
    // and to unwind a partially-constructed block if a part fails to attach.
    void releaseInstances();

    BlockResources*           m_resources;
    std::vector<PartInstance> m_parts;
};
