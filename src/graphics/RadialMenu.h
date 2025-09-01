// RadialMenu.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

// Forward declarations
class GraphicsEngine;
class Geometry;
class Instance;

/**
 * @brief Simple node structure for radial menu
 */
struct RadialMenuNode {
    uint64_t id;
    std::weak_ptr<Instance> instance;
    
    RadialMenuNode(uint64_t nodeId) : id(nodeId) {}
};

/**
 * @brief Minimalistic radial menu implementation
 */
class RadialMenu {
public:
    explicit RadialMenu(GraphicsEngine* graphics);
    ~RadialMenu();
    
    // Node management
    uint64_t createNode();
    
    // Transform control
    void setPosition(const glm::dvec3& position);
    void setOrientation(const glm::dquat& orientation);

    // Visibility control
    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }
    
private:
    GraphicsEngine* m_graphics;
    glm::dvec3 m_position{0.0, 0.0, 0.0};
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
    
    std::weak_ptr<Geometry> m_geometry;
    int m_textureIndex{-1};
    int m_meshId{-1};
    bool m_visible{true};
    glm::dvec4 m_color = {1,0,0,0};
    
    std::vector<RadialMenuNode> m_nodes;
    
    void updateInstanceTransform(RadialMenuNode& node);
    void updateMeshTransform();
    
    static uint64_t s_nextNodeId;
};
