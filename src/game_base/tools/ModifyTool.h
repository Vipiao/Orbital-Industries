// ModifyTool.h
#pragma once

#include <memory>
#include <functional>
#include <glm/glm.hpp>
#include <vector>
#include <glm/gtc/quaternion.hpp>
#include <vector>
#include <array>
#include "graphics/MeshManager2D/GeometryData.h"

// Forward declarations
class GameBase;
class Grid;
class RadialMenu;
class GeometryInstance;
class Geometry;
class Instance;
class Grid;

class ModifyTool {
public:
    ModifyTool(GameBase* gameBase, RadialMenu* radialMenu, int64_t parentNodeId, double interactionRange);
    ~ModifyTool();
    
    // Activation control
    void activate();
    void deactivate();
    bool isActive() const { return m_active; }
    
    // Callback hooks
    void preRenderCallback(bool doModify, bool doCancel);
    void onPhysicsUpdateComplete(const std::vector<std::weak_ptr<Grid>>& availableGrids);
    
private:
    // Interaction range
    
    GameBase* m_gameBase;
    RadialMenu* m_radialMenu;
    double m_interactionRange;
    
    // Internal state
    bool m_active{false};
    
    // Input flags
    bool m_doModify{false};
    bool m_doCancel{false};
    
    // Block selection state
    std::weak_ptr<Grid> m_selectedGrid;
    glm::ivec3 m_selectedBlockCoord;
    bool m_hasSelectedBlock{false};
    bool m_cursorNearMarker{false};
    int m_nearestMarkerIndex{-1};
    
    // 2D marker geometry and instances
    std::weak_ptr<GeometryData> m_marker;
    std::vector<std::weak_ptr<GeometryInstance>> m_markerInstances;
    
    // Selected marker coordinate and direction data
    glm::ivec3 m_selectedMarkerCoordinate;
    glm::dvec3 m_selectedMarkerDirection;
    
    // 3D arrow markers
    std::weak_ptr<Geometry> m_arrowGeometry;
    std::vector<std::weak_ptr<Instance>> m_arrowInstances;
    int m_currentSelectedGridSsboIndex{-1};
    
    // Cell modification data
    std::weak_ptr<Grid> m_modificationGrid;
    glm::ivec3 m_modificationCoord;
    std::array<glm::ivec3, 8> m_modificationVertices;
    
    // Menu structure
    int64_t m_modifyToolParentId{-1};
    int64_t m_centerNodeId{-1};

    // Modify crosshair
    std::weak_ptr<GeometryData> m_modifyCrosshairGeometry;
    std::weak_ptr<GeometryInstance> m_modifyCrosshairInstance;
    double m_modifyCrosshairTransparency = 0.75;
    double m_targetAngle = 0.0;
    double m_currentAngle = 0.0;
    glm::dvec2 m_crosshairOffset;
    glm::dvec2 m_crosshairScale;
    
    // Icon texture
    int m_modifyIconTextureIndex{-1};

    // Helper methods
    void createMenuStructure(int64_t parentNodeId);
    void updateMarkerPositions();
    static glm::dquat getArrowOrientation(const glm::ivec3& direction);
};