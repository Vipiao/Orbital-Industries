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
// Placement mirrors BlockGraphics (rotation about the model centre). The thrust
// level rides the instance value channel (state.x) and drives the plume density
// in the shader; a plume starts invisible until a thrust level is set.
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

    // Throttle in [0, 1], passed to the shader as the instance value state.x.
    void setThrustLevel(double level);

private:
    GraphicsEngine*         m_graphics;
    std::weak_ptr<Geometry> m_geometry;
    std::weak_ptr<Instance> m_instance;
};
