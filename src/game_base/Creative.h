// Creative.h
#pragma once

#include "Mode.h"
#include <glm/glm.hpp>
#include <array>
#include <memory>
#include <vector>
class RadialMenu;
class GeometryData;
class Geometry;
class Instance;
class GeometryInstance;

/**
 * @brief Creative mode implementation with block placement/removal and force application
 */
class Creative : public Mode {
public:
    Creative(GameBase* gameBase);
    virtual ~Creative();
    
    virtual void processInputs() override;
    virtual void physics() override;
    
private:
    // Input flags
    bool doCreate = false;
    bool doRemove = false;
    bool doConfigure = false;
    bool doForce = false;
    bool doTrackSpeed = false;
    bool doModifyCell = false;
    double forceMultiplier = 1.0;
    double m_moveSpeed = 0.05;

    // Color management
    bool doCopy = false;
    bool doPaste = false;
    bool doUpdateColor = false;
    glm::dvec4 color{1.0, 1.0, 1.0, 1.0};        // UI adjustment buffer
    glm::dvec4 copiedColor{1.0, 1.0, 1.0, 1.0};  // Copy/paste buffer
    
    // Helper methods
    void addGridBlock(class Grid* grid, int x, int y, int z);
    void removeGridBlock(class Grid* grid, int x, int y, int z);
    void applyDragForces();
    void processInputLogic();
    void handleConfigureMode(bool blockFound, std::weak_ptr<class Grid> targetGridWeak, const glm::ivec3& hitPos);
    void updateMarkerPositions();
    void handleColorInput();

    // Helper for arrow orientation calculation
    static glm::quat getArrowOrientation(const glm::ivec3& direction);
    
    // Configuration mode state
    std::weak_ptr<class Grid> m_selectedGrid;
    glm::ivec3 m_selectedBlockCoord;
    bool m_hasSelectedBlock = false;
    bool m_cursorNearMarker = false;
    int m_nearestMarkerIndex = -1;
    std::weak_ptr<GeometryData> m_marker;
    std::vector<std::weak_ptr<GeometryInstance>> m_markerInstances;

    // Selected marker coordinate and direction data
    glm::ivec3 m_selectedMarkerCoordinate;
    glm::dvec3 m_selectedMarkerDirection;

    // 3D arrow markers
    std::weak_ptr<Geometry> m_arrowGeometry;
    int m_arrowTextureIndex = -1;
    std::vector<std::weak_ptr<Instance>> m_arrowInstances;
    int m_currentSelectedGridMeshId = -1;

    // Cell modification data
    std::weak_ptr<class Grid> m_modificationGrid;
    glm::ivec3 m_modificationCoord;
    std::array<glm::ivec3, 8> m_modificationVertices;

    // Radial menu
    std::unique_ptr<RadialMenu> m_radialMenu;

    // Radial menu positioning
    double m_radialMenuDistance = 4.0; // Distance from camera when visible
    glm::dvec3 m_radialMenuRelativePosition = {0,0,0};
};