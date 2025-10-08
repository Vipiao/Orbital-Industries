// Digitbot.h
#pragma once

#include "Character.h"
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class DigitbotPhysics;
class DigitbotGraphics;
struct DigitbotTargetPose;

/**
 * @brief Bipedal robot character with reverse-articulated legs
 * 
 * Digitbot consists of two stacked cube segments forming a simple
 * robotic body. Uses GridCollider for efficient collision detection.
 */
class Digitbot : public Character {
public:
    Digitbot(PhysicsEngine* physics, GraphicsEngine* graphics,
             JobManager* jobManager, TimeHandler* timeHandler);
    ~Digitbot() override;

    void preRenderCallback(uint64_t frameNum) override;
    void onPhysicsUpdateComplete() override;

    void showCollisionBox() override;
    void hideCollisionBox() override;
    bool isCollisionBoxVisible() const override;

    // Coordinate transforms
    glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const override;
    glm::dvec3 localToWorld(const glm::dvec3& localPos) const override;

private:
    void updateVisualTransform();

    // Subsystems
    std::unique_ptr<DigitbotPhysics> m_digitbotPhysics;
    std::unique_ptr<DigitbotGraphics> m_digitbotGraphics;

    // Cache GraphicsEngine pointer for convenience
    GraphicsEngine* m_graphicsEngine;
};