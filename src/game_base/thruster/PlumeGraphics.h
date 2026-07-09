// PlumeGraphics.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GraphicsEngine;
class Geometry;
class Instance;

// Per-thruster ion-plume rendering. Attaches one ray-volume instance on the
// shared plume proxy to the grid's SSBO slot, so the plume moves with the ship.
// Placement mirrors BlockGraphics (rotation about the model centre). Always
// present for now; thrust-driven appearance will later ride the value channel.
class PlumeGraphics {
public:
    PlumeGraphics(GraphicsEngine* graphics,
                  std::weak_ptr<Geometry> geometry,
                  int ssboIndex,
                  const glm::ivec3& anchorCoord,
                  const glm::dquat& orientation,
                  const glm::dvec3& modelCentre);
    ~PlumeGraphics();

    PlumeGraphics(const PlumeGraphics&) = delete;
    PlumeGraphics& operator=(const PlumeGraphics&) = delete;

private:
    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
};
