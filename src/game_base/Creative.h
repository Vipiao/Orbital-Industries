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
class ColorTool;
class ModifyTool;

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
    bool doForce = false;
    bool doTrackSpeed = false;
    double forceMultiplier = 1.0;
    double m_moveSpeed = 8.;

    // Helper methods
    void addGridBlock(class Grid* grid, int x, int y, int z);
    void removeGridBlock(class Grid* grid, int x, int y, int z);
    void applyDragForces();
    void processInputLogic();

    // Radial menu
    std::unique_ptr<RadialMenu> m_radialMenu;

    // Radial menu positioning
    double m_radialMenuDistance = 4.0; // Distance from camera when visible
    glm::dvec3 m_radialMenuRelativePosition = {0,0,0};

    // Color tool
    std::unique_ptr<ColorTool> m_colorTool;

    // Modify tool
    std::unique_ptr<ModifyTool> m_modifyTool;

    // Regular crosshair
    std::weak_ptr<GeometryData> m_crosshairGeometry;
    std::weak_ptr<GeometryInstance> m_crosshairInstance;
};