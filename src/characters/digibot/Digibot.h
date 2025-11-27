// Digibot.h
#pragma once

#include "../Character.h"
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class DigibotPhysics;
class DigibotGraphics;
class DigibotController;
class DigibotResources;
struct DigibotTargetPose;
class GridSubsystem;

/**
 * @brief Bipedal robot character with reverse-articulated legs
 * 
 * Digibot consists of two stacked cube segments forming a simple
 * robotic body. Uses GridCollider for efficient collision detection.
 */
class Digibot : public Character {
public:
    Digibot(PhysicsEngine* physics, GraphicsEngine* graphics,
             JobManager* jobManager, TimeHandler* timeHandler,
             DigibotResources* resources, GridSubsystem* gridSubsystem);
    ~Digibot() override;

    void preRenderCallback(uint64_t frameNum, double timeRemainder) override;
    void onPhysicsUpdateComplete() override;

    void showCollisionBox() override;
    void hideCollisionBox() override;
    bool isCollisionBoxVisible() const override;

    // Coordinate transforms
    glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const override;
    glm::dvec3 localToWorld(const glm::dvec3& localPos) const override;

    // Movement control
    void setMovementDirection(const glm::ivec3& direction);

    // View direction control
    void setViewDirection(const glm::dvec3& direction);
    glm::dvec3 getViewDirection() const;

    // Roll input control
    void setRollInput(int rollInput);

    // Controller access
    DigibotController* getController() { return m_digibotController.get(); }

private:
    void updateVisualTransform();

    // Subsystems
    std::unique_ptr<DigibotPhysics> m_digibotPhysics;
    std::unique_ptr<DigibotGraphics> m_digibotGraphics;

    // Controller for movement
    std::unique_ptr<DigibotController> m_digibotController;

    //
    glm::dvec3 m_graphicsPosition = {0.0, 0.0, -1.0};

    // Cache GraphicsEngine pointer for convenience
    GraphicsEngine* m_graphicsEngine;

    // View direction (in world space)
    glm::dvec3 m_viewDirection{0.0, 1.0, 0.0}; // Default forward

    // Frame timing for per-frame updates
    double m_lastTimeRemainder{0.0};
};