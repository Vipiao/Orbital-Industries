// Creative.h
#pragma once

#include "Mode.h"
#include <glm/glm.hpp>
#include <array>
#include <memory>
#include <vector>
class RadialMenu;
class Geometry2D;
class Geometry;
class Instance;
class CharacterSelectionTool;
class Instance2D;
class ColorTool;
class ModifyTool;
class BuildTool;
class FreeCameraController;
class DigibotPlayerController;
class Collider;

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
    // Interaction range for tools and sensors
    double m_interactionRange = 20.0;

    // Input flags
    bool doForce = false;
    bool doTrackSpeed = false;
    double forceMultiplier = 1.0;
    double m_moveSpeed = 8.;

    // Helper methods
    void applyDragForces();
    void processInputLogic();

    // Radial menu
    std::unique_ptr<RadialMenu> m_radialMenu;

    // Radial menu positioning
    double m_radialMenuDistance = 4.0; // Distance from camera when visible
    glm::dvec3 m_radialMenuRelativePosition = {0,0,0};

    // Frame timing for surface rotation
    double m_lastTimeRemainder{0.0};

    // Color tool
    std::unique_ptr<ColorTool> m_colorTool;

    // Free camera controller
    std::unique_ptr<FreeCameraController> m_freeCameraController;

    // Digibot player controller
    std::unique_ptr<DigibotPlayerController> m_digibotPlayerController;

    // Character selection tool
    std::unique_ptr<CharacterSelectionTool> m_characterSelectionTool;

    // Modify tool
    std::unique_ptr<ModifyTool> m_modifyTool;

    // Build tool
    std::unique_ptr<BuildTool> m_buildTool;

    // Interaction sensor for spatial filtering
    std::weak_ptr<Collider> m_interactionSensor;

    // Regular crosshair
    std::weak_ptr<Geometry2D> m_crosshairGeometry;
    std::weak_ptr<Instance2D> m_crosshairInstance;
};