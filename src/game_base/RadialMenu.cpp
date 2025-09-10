// RadialMenu.cpp
#include "RadialMenu.h"
#include "../graphics/GraphicsEngine.h"
#include "../graphics/InstanceHandler.h"
#include "../graphics/SSBOManager.h"
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

glm::dvec3 RadialMenu::worldToLocal(const glm::dvec3& worldPos) const {
    // Transform from world space to local space
    // local = inverse_orientation * (world - position)
    glm::dvec3 translated = worldPos - m_position;
    return glm::conjugate(m_orientation) * translated;
}

void RadialMenu::navigateToParent() {
    auto currentNodeIt = m_nodes.find(m_currentNodeId);
    if (currentNodeIt == m_nodes.end()) return;
    
    int64_t parentId = currentNodeIt->second.m_parentId;
    if (parentId == -1) return; // Already at root
    
    auto parentIt = m_nodes.find(parentId);
    if (parentIt == m_nodes.end()) return;
    
    m_currentNodeId = parentId;
    updateRendering();
    
    std::cout << "Navigated to parent node " << parentId << std::endl;
}

void RadialMenu::run(const glm::dvec3& localRayStart, const glm::dvec3& localRayEnd, bool doSelect) {
    if (!m_visible) return;

    // Get current node info
    auto currentNodeIt = m_nodes.find(m_currentNodeId);
    if (currentNodeIt == m_nodes.end()) return;
    
    const RadialMenuNode& currentNode = currentNodeIt->second;
    size_t childCount = currentNode.m_childIds.size();
    
    if (childCount == 0) return;
    
    // Intersect ray with Z=0 plane
    glm::dvec3 rayDir = localRayEnd - localRayStart;
    int selectedIndex = -1;
    
    if (glm::abs(rayDir.z) >= 1e-6) { // Ray not parallel to plane
        double t = -localRayStart.z / rayDir.z;
        if (t >= 0.0 && t <= 1.0) { // Intersection within ray segment
            glm::dvec3 intersection = localRayStart + t * rayDir;
            glm::dvec2 pos2D(intersection.x, intersection.y);
            // Determine selection
            double distance = glm::length(pos2D);
            const double centerThreshold = 0.17;
            const double outerThreshold = 1.0;
            
            if (distance < centerThreshold && childCount > 0) {
                selectedIndex = 0; // Center
            } else if (distance <= outerThreshold && childCount > 1) {
                // Calculate angle and map to segment
                double angle = glm::atan(pos2D.y, pos2D.x);
                if (angle < 0) angle += 2.0 * glm::pi<double>();
                
                double angleStep = 2.0 * glm::pi<double>() / static_cast<double>(childCount - 1);
                int segmentIndex = static_cast<int>(angle / angleStep);
                selectedIndex = segmentIndex + 1; // +1 because center is index 0
            }
        }
    }
    
    // Handle selection
    if (doSelect && selectedIndex >= 0) {
        int64_t targetChildId = currentNode.m_childIds[selectedIndex];
        
        auto targetIt = m_nodes.find(targetChildId);
        if (targetIt != m_nodes.end()) {
            // Execute callback if it exists
            if (targetIt->second.m_callback) {
                targetIt->second.m_callback();
            }
            
            // Navigate to child if it has children
            if (!targetIt->second.m_childIds.empty()) {
                m_currentNodeId = targetChildId;
                updateRendering();
            }
        }
    }
    
    // Update instance colors and scales
    auto geometry = m_geometry.lock();
    if (!geometry) return;
    
    double angleStep = (childCount > 1) ? 2.0 * glm::pi<double>() / static_cast<double>(childCount - 1) : 0.0;
    
    for (size_t i = 0; i < m_currentInstances.size() && i < childCount; ++i) {
        auto inst = m_currentInstances[i].lock();
        if (!inst) continue;
        
        bool isSelected = (selectedIndex >= 0 && static_cast<size_t>(selectedIndex) == i);
        
        inst->m_color = isSelected ? m_selectColor : m_unSelectColor;
        inst->m_localScale = glm::dvec3(1.0);
        
        // Calculate position with selection offset
        glm::dvec3 basePosition = glm::dvec3(0.0);
        glm::dvec3 selectionOffset = glm::dvec3(0.0);
        
        if (isSelected) {
            if (i == 0 || childCount < 3) {
                // Center button.
                selectionOffset = glm::dvec3(0.0, 0.0, 0.0);
                inst->m_localScale = glm::dvec3{1.1};
            } else {
                // Segment button: offset in angle direction + 180 degrees (only if enough children)
                double offsetAngle = angleStep * (static_cast<double>(i-1) + 0.5) + glm::pi<double>();
                selectionOffset = -glm::dvec3(glm::cos(offsetAngle), glm::sin(offsetAngle), 0.0) * 0.05;
            }
        }
        
        inst->m_localPosition = basePosition + selectionOffset;
        geometry->updateInstanceInBuffer(inst.get());
    }
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
            inst->m_color = m_unSelectColor;
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
                inst->m_color = m_unSelectColor;
                
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
        -1,                                           // no material texture
        0,                                            // time
        1.0);                                         // emissive scalar
}