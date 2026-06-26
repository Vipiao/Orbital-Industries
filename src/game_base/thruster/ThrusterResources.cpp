// ThrusterResources.cpp
#include "ThrusterResources.h"
#include "graphics/GraphicsEngine.h"
#include "graphics/instanceHandler/InstanceHandler.h"
#include <stdexcept>
#include <iostream>

ThrusterResources::ThrusterResources(GraphicsEngine* graphics)
    : m_graphics(graphics)
{
    if (!m_graphics) {
        throw std::runtime_error("ThrusterResources: GraphicsEngine cannot be null");
    }
    loadResources();
}

ThrusterResources::~ThrusterResources() {
    if (!m_geometry.expired()) {
        m_graphics->getInstanceHandler()->releaseGeometry(m_geometry);
    }
}

void ThrusterResources::loadResources() {
    m_colorTextureUnit  = m_graphics->getInstanceHandler()->createTexture("../media/models/thruster/albedo.png");
    m_normalTextureUnit = m_graphics->getInstanceHandler()->createTexture("../media/models/thruster/normal.png");

    if (m_colorTextureUnit == -1 || m_normalTextureUnit == -1) {
        throw std::runtime_error("ThrusterResources: failed to load textures");
    }

    m_geometry = m_graphics->getInstanceHandler()->createGeometry("../media/models/thruster/thruster_v2.obj");
    if (m_geometry.expired()) {
        throw std::runtime_error("ThrusterResources: failed to load geometry");
    }

    std::cout << "ThrusterResources: loaded geometry and textures" << std::endl;
}
