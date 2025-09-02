// RadialMenu.cpp
#include "RadialMenu.h"
#include "GraphicsEngine.h"
#include "InstanceHandler.h"
#include "SSBOManager.h"
#include <iostream>

// Static member initialization
int64_t RadialMenu::s_nextNodeId = 0;

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
    
    // Load all textures
    loadAllTextures();

    // Set initial mesh transform in SSBO
    updateMeshTransform();
}

RadialMenu::~RadialMenu() {
    // Clear all instances first
    clearCurrentInstances();
   
    // Cleanup SSBO
    if (m_meshId >= 0) {
        m_graphics->m_ssboManager->deallocateIndex(m_meshId);
    }
    
    // Release textures
    for (int i = 0; i < 9; ++i) {
        if (m_textures[i] >= 0) {
            m_graphics->getInstanceHandler()->releaseTexture(m_textures[i]);
        }
    }
    
    // Release geometry
    if (!m_geometry.expired()) {
        m_graphics->getInstanceHandler()->releaseGeometry(m_geometry);
    }
}

void RadialMenu::loadAllTextures() {
    // Initialize array
    for (int i = 0; i < 9; ++i) {
        m_textures[i] = -1;
    }
    
    // Load u0 through u8 textures
    for (int i = 0; i <= 8; ++i) {
        std::string textureName = "u" + std::to_string(i);
        std::string texturePath = "../media/radial_menu_graphics_generator/" + textureName + ".png";
        m_textures[i] = m_graphics->getInstanceHandler()->createTexture(texturePath);
    }
}

int64_t RadialMenu::createNode(int64_t parentId, int symbolTextureIndex, std::function<void()> callback) {
    int64_t nodeId = s_nextNodeId++;

    // Create node in-place to avoid default constructor requirement
    auto& node = m_nodes.emplace(nodeId, nodeId).first->second;
    node.m_parentId = parentId;
    node.m_callback = callback;
    node.m_symbolTextureIndex = symbolTextureIndex;

    // If this is a root node (parentId == -1), set as current
    if (parentId == -1) {
        m_currentNodeId = nodeId;
    } else {
        // Add to parent's children
        auto parentIt = m_nodes.find(parentId);
        if (parentIt == m_nodes.end()) {
            return -1;
        }
        parentIt->second.m_childIds.push_back(nodeId);
    }
    
    // Update rendering if needed
    if ((parentId == -1 || m_currentNodeId == parentId) && m_visible) {
        updateRendering();
    }
    
    return nodeId;
}

void RadialMenu::setPosition(const glm::dvec3& position) {
    m_position = position;
    updateMeshTransform();
}

void RadialMenu::setOrientation(const glm::dquat& orientation) {
    m_orientation = orientation;
    updateMeshTransform();
}

void RadialMenu::setVisible(bool visible) {
    if (m_visible == visible) return;
    
    m_visible = visible;
    
    if (m_visible) {
        updateRendering();
    } else {
        clearCurrentInstances();
    }
}

void RadialMenu::run(const glm::dvec2& screenPosition, bool doSelect) {
    // TODO: Implement interaction logic
    (void)screenPosition;
    (void)doSelect;
}

void RadialMenu::updateRendering() {
    if (!m_visible) return;
    
    // Clear existing instances
    clearCurrentInstances();
    
    // Create new instances for current node
    auto currentNodeIt = m_nodes.find(m_currentNodeId);
    if (currentNodeIt == m_nodes.end()) return;
    
    const RadialMenuNode& currentNode = currentNodeIt->second;
    size_t childCount = currentNode.m_childIds.size();
    
    // Case 1: No children - render nothing
    if (childCount == 0) {
        return;
    }
    
    auto geometry = m_geometry.lock();
    if (!geometry) return;
    
    // Case 2: Has children - render center + segments
    
    // Create center instance (first child)
    if (m_textures[0] >= 0) {
        auto centerInstance = geometry->addInstance(m_meshId, m_textures[0], -1);
        if (!centerInstance.expired()) {
            auto inst = centerInstance.lock();
            inst->m_localPosition = glm::dvec3(0.0);
            inst->m_localOrientation = glm::dquat(1.0, 0.0, 0.0, 0.0);
            inst->m_localScale = glm::dvec3(1.0);
            geometry->updateInstanceInBuffer(inst.get());
            m_currentInstances.push_back(centerInstance);
        }
    }
    
    // Create segment instances for remaining children
    if (childCount > 1 && childCount <= 9 && m_textures[childCount-1] >= 0) {
        double angleStep = 2.0 * glm::pi<double>() / static_cast<double>(childCount - 1);
        
        // Skip first child (that's the center button)
        for (size_t i = 1; i < childCount; ++i) {
            auto segmentInstance = geometry->addInstance(m_meshId, m_textures[childCount-1], -1);
            if (!segmentInstance.expired()) {
                auto inst = segmentInstance.lock();
                inst->m_localPosition = glm::dvec3(0.0);
                
                // Rotate around Z-axis by angle for this segment
                double angle = static_cast<double>(i - 1) * angleStep;
                inst->m_localOrientation = glm::angleAxis(angle, glm::dvec3(0.0, 0.0, 1.0));
                inst->m_localScale = glm::dvec3(1.0);
                
                geometry->updateInstanceInBuffer(inst.get());
                m_currentInstances.push_back(segmentInstance);
            }
        }
    }
}

void RadialMenu::clearCurrentInstances() {
    auto geometry = m_geometry.lock();
    if (!geometry) return;
    
    for (auto& instanceWeak : m_currentInstances) {
        if (!instanceWeak.expired()) {
            geometry->removeInstance(instanceWeak);
        }
    }
    m_currentInstances.clear();
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
        -1, -1,                                       // no default textures
        0);                                           // time
}