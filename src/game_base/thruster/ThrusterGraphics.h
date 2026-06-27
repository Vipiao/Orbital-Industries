// ThrusterGraphics.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class ThrusterResources;
class Geometry;
class Instance;

/**
 * @brief Per-thruster instance rendering.
 * Attaches one OBJ instance to the grid's SSBO slot so it moves with the grid.
 * Local position is set to the anchor grid coord; local orientation is the full
 * block orientation quaternion so arbitrary rotations render correctly.
 */
class ThrusterGraphics {
public:
    ThrusterGraphics(ThrusterResources* resources,
                     int ssboIndex,
                     const glm::ivec3& anchorCoord,
                     const glm::dquat& orientation);
    ~ThrusterGraphics();

    ThrusterGraphics(const ThrusterGraphics&) = delete;
    ThrusterGraphics& operator=(const ThrusterGraphics&) = delete;

private:
    ThrusterResources* m_resources;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
};
