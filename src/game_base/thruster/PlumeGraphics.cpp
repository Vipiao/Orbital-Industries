// PlumeGraphics.cpp
#include "PlumeGraphics.h"
#include "../BlockPlacement.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"  // Geometry / Instance
#include <stdexcept>

PlumeGraphics::PlumeGraphics(GraphicsEngine* graphics,
                             std::weak_ptr<Geometry> geometry,
                             int ssboIndex,
                             const glm::ivec3& anchorCoord,
                             const glm::dquat& orientation,
                             const glm::dvec3& modelCentre)
    : m_graphics{graphics}, m_geometry{geometry}
{
    if (!m_graphics) {
        throw std::runtime_error{"PlumeGraphics: GraphicsEngine cannot be null"};
    }

    // Plume tint (the body drives alpha from the marched density and uses this
    // colour). Alpha is unused by the body.
    const glm::dvec4 plumeColor{0.55, 0.30, 0.95, 1.0};
    m_instance = m_graphics->addRayVolumeInstance(m_geometry, ssboIndex, plumeColor);

    auto instance = m_instance.lock();
    auto geo = m_geometry.lock();
    if (!instance || !geo) {
        throw std::runtime_error{"PlumeGraphics: failed to create plume instance"};
    }

    // Offset the plume along the thruster's local +Y (thrust axis) so it emerges
    // from the nozzle and only slightly overlaps the 2-cell thruster body, rather
    // than sitting on top of it. Rotated by the block orientation so it holds for
    // any thruster facing.
    constexpr double plumeOffsetY{1.8};

    // Set the local transform on the returned instance and re-upload, exactly
    // like the instance handler path.
    instance->m_localPosition    = blockLocalPosition(anchorCoord, orientation, modelCentre)
                                 + orientation * glm::dvec3{0.0, plumeOffsetY, 0.0};
    instance->m_localOrientation = orientation;
    instance->m_localScale       = glm::dvec3{1.0};
    geo->updateInstanceInBuffer(instance.get());
}

PlumeGraphics::~PlumeGraphics() {
    if (m_graphics) {
        m_graphics->removeRayVolumeInstance(m_geometry, m_instance);
    }
}
