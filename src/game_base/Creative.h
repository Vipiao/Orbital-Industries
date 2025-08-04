// Creative.h
#pragma once

#include "Mode.h"
#include <glm/glm.hpp>
#include "../graphics/MeshManager2D.h"
#include "../graphics/GeometryInstance.h"

/**
 * @brief Creative mode implementation with block placement/removal and force application
 */
class Creative : public Mode {
public:
    Creative(GameBase* gameBase);
    Creative(GameBase* gameBase, MeshManager2D* meshManager);
    virtual ~Creative() = default;
    
    virtual void processInputs() override;
    virtual void physics() override;
    
private:
    // Input flags
    bool doCreate = false;
    bool doRemove = false;
    bool doConfigure = false;
    bool doForce = false;
    bool doTrackSpeed = false;
    double forceMultiplier = 1.0;
    double m_moveSpeed = 0.05;
    
    // Helper methods
    void addGridBlock(class Grid* grid, int x, int y, int z);
    void removeGridBlock(class Grid* grid, int x, int y, int z);
    void applyDragForces();
    void processInputLogic();
    void handleConfigureMode(bool blockFound, std::weak_ptr<class Grid> targetGridWeak, const glm::ivec3& hitPos);
    void updateMarkerPositions();
    
    // Configuration mode state
    std::weak_ptr<class Grid> m_selectedGrid;
    glm::ivec3 m_selectedBlockCoord;
    bool m_hasSelectedBlock = false;
    bool m_cursorNearMarker = false;
    int m_nearestMarkerIndex = -1;
    MeshManager2D* m_meshManager;
    std::weak_ptr<GeometryData> m_marker;
    std::vector<std::weak_ptr<GeometryInstance>> m_markerInstances;
};