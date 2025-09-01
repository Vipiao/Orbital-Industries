// RadialMenu.cpp
#include "RadialMenu.h"
#include "GraphicsEngine.h"
#include "InstanceHandler.h"
#include "SSBOManager.h"
#include <iostream>

// Static member initialization
uint64_t RadialMenu::s_nextNodeId = 0;

RadialMenu::RadialMenu(GraphicsEngine* graphics) : m_graphics(graphics) {
    if (!m_graphics) {
        throw std::runtime_error("GraphicsEngine cannot be null");
    }
    
    // Load geometry using InstanceHandler
    try {
        m_geometry = m_graphics->getInstanceHandler()->createGeometry("../media/blender/03_face.obj");
        if (m_geometry.expired()) {
            throw std::runtime_error("Failed to create geometry from 03_face.obj");
        }
        std::cout << "RadialMenu: Loaded geometry from 03_face.obj" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "RadialMenu: Failed to load geometry: " << e.what() << std::endl;
        throw;
    }

    // Configure geometry for alpha blending
    if (auto geometry = m_geometry.lock()) {
        geometry->setAlphaBlending(true);
        std::cout << "RadialMenu: Enabled alpha blending for geometry" << std::endl;
    }

    // Allocate SSBO index for mesh transform data
    try {
        m_meshId = m_graphics->m_ssboManager->allocateIndex();
        std::cout << "RadialMenu: Allocated SSBO mesh ID " << m_meshId << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "RadialMenu: Failed to allocate SSBO index: " << e.what() << std::endl;
        throw;
    }
    
    // Load texture using InstanceHandler  
    try {
        m_textureIndex = m_graphics->getInstanceHandler()->createTexture("../media/radial_menu_graphics_generator/u0.png");
        if (m_textureIndex < 0) {
            throw std::runtime_error("Failed to create texture from u0.png");
        }
        std::cout << "RadialMenu: Loaded texture from u0.png with index " << m_textureIndex << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "RadialMenu: Failed to load texture: " << e.what() << std::endl;
        throw;
    }

    // Set initial mesh transform in SSBO
    updateMeshTransform();
}

RadialMenu::~RadialMenu() {
    // Cleanup handled by InstanceHandler when geometry/instances are destroyed
    if (m_meshId >= 0) {
        m_graphics->m_ssboManager->deallocateIndex(m_meshId);
    }
    
    if (m_textureIndex >= 0) {
        m_graphics->getInstanceHandler()->releaseTexture(m_textureIndex);
    }
    
    if (!m_geometry.expired()) {
        m_graphics->getInstanceHandler()->releaseGeometry(m_geometry);
    }
}

uint64_t RadialMenu::createNode() {
    auto geometry = m_geometry.lock();
    if (!geometry) {
        std::cerr << "RadialMenu: Cannot create node - geometry not available" << std::endl;
        return 0;
    }
    
    // Create instance with allocated mesh ID
    auto instance = geometry->addInstance(m_meshId, m_textureIndex, -1);
    instance.lock()->m_color = m_color;
    if (instance.expired()) {
        std::cerr << "RadialMenu: Failed to create instance for node" << std::endl;
        return 0;
    }
    
    // Create node and add to tracking
    uint64_t nodeId = s_nextNodeId++;
    m_nodes.emplace_back(nodeId);
    m_nodes.back().instance = instance;
    
    // Set instance to default transforms
    updateInstanceTransform(m_nodes.back());

    std::cout << "RadialMenu: Created node " << nodeId << std::endl;
    return nodeId;
}

void RadialMenu::setPosition(const glm::dvec3& position) {
    m_position = position;
    // Update mesh transform in SSBO
    updateMeshTransform();
}

void RadialMenu::setOrientation(const glm::dquat& orientation) {
    m_orientation = orientation;
    // Update mesh transform in SSBO
    updateMeshTransform();
}

void RadialMenu::setVisible(bool visible) {
    if (m_visible == visible) return;
    
    m_visible = visible;
    auto geometry = m_geometry.lock();
    if (!geometry) return;
    
    if (m_visible) {
        // Make instances visible by recreating them
        for (auto& node : m_nodes) {
            if (node.instance.expired()) {
                auto instance = geometry->addInstance(m_meshId, m_textureIndex, -1);
                instance.lock()->m_color = m_color;
                node.instance = instance;
                updateInstanceTransform(node);
            }
        }
    } else {
        // Make instances invisible by removing them
        for (auto& node : m_nodes) {
            if (!node.instance.expired()) {
                geometry->removeInstance(node.instance);
                // instance becomes expired after removal
            }
        }
    }
}

void RadialMenu::updateInstanceTransform(RadialMenuNode& node) {
    if (!m_visible) return;
    
    auto instance = node.instance.lock();
    if (!instance) return;
    
    auto geometry = m_geometry.lock();
    if (!geometry) return;
    
    // Set instance to default transforms - mesh transform handled by SSBO
    instance->m_localPosition = glm::vec3(0.0f);
    instance->m_localOrientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    instance->m_localScale = glm::vec3(1.0f);
    geometry->updateInstanceInBuffer(instance.get());
}

void RadialMenu::updateMeshTransform() {
    m_graphics->m_ssboManager->updateMeshTransform(
        m_meshId,
        m_position,                                    // position
        glm::dvec3(0.0),                              // velocity
        m_orientation,                                 // orientation
        glm::dvec3(0.0, 1.0, 0.0), 0.0,              // angular velocity (axis, magnitude)
        glm::dvec3(0.0),                              // center of rotation
        glm::dvec3(1.0),                              // scale
        m_textureIndex, -1,                           // color texture, normal texture
        0);                                           // time
}