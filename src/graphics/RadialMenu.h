// RadialMenu.h
#pragma once

#include <memory>
#include <unordered_map>
#include <functional>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

// Forward declarations
class GraphicsEngine;
class Geometry;
class Instance;

/**
 * @brief Node structure for radial menu with hierarchy support
 */
struct RadialMenuNode {
    int64_t m_id;
    int64_t m_parentId = -1; // -1 means root (no parent)
    std::vector<int64_t> m_childIds;
    std::function<void()> m_callback;
    int m_symbolTextureIndex = -1; // Optional symbol texture for this node
    std::weak_ptr<Instance> instance;
    
    RadialMenuNode(int64_t nodeId) : m_id(nodeId) {}
};

/**
 * @brief Minimalistic radial menu implementation
 */
class RadialMenu {
public:
    explicit RadialMenu(GraphicsEngine* graphics);
    ~RadialMenu();
    
    // Node management
    int64_t createNode(int64_t parentId = -1, int symbolTextureIndex = -1, std::function<void()> callback = nullptr);
    
    // Transform control
    void setPosition(const glm::dvec3& position);
    void setOrientation(const glm::dquat& orientation);

    // Visibility control
    void setVisible(bool visible);
    bool isVisible() const { return m_visible; }

    // Interaction
    void run(const glm::dvec2& screenPosition, bool doSelect);
    
private:
    GraphicsEngine* m_graphics;
    glm::dvec3 m_position{0.0, 0.0, 0.0};
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
    
    std::weak_ptr<Geometry> m_geometry;
    int m_meshId{-1};
    bool m_visible{true};
    
    // Texture management - u0 through u8 (9 textures)
    int m_textures[9]; // u0, u1, u2, ..., u8
    
    // Node hierarchy
    std::unordered_map<int64_t, RadialMenuNode> m_nodes;
    int64_t m_currentNodeId{-1};
    
    // Current rendering instances
    std::vector<std::weak_ptr<Instance>> m_currentInstances;
     
    // Internal methods
    void loadAllTextures();
    void updateRendering();
    void clearCurrentInstances();
    void updateMeshTransform();
    
    static int64_t s_nextNodeId;
};
