// Digitbot.h
#pragma once

#include "Character.h"
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class GridCollider;
class Geometry;
class Instance;

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
    bool isCollisionBoxVisible() const override { return m_collisionBoxMeshId != -1; }

    // Coordinate transforms
    glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const override;
    glm::dvec3 localToWorld(const glm::dvec3& localPos) const override;

private:
    void createCollisionBoxMesh();
    void updateCollisionBoxTransform();
    void updateVisualModelTransform();

    std::weak_ptr<GridCollider> m_colliderWeak;
    int m_collisionBoxMeshId;

    // Visual model (instanced rendering)
    std::weak_ptr<Geometry> m_visualGeometry;
    std::weak_ptr<Instance> m_visualInstance;
    int m_visualMeshSSBOIndex;
};