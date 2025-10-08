// DigitbotPhysics.h
#pragma once

#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class PhysicsEngine;
class GraphicsEngine;
class JobManager;
class TimeHandler;
class GridCollider;
class RigidBody;

/**
 * @brief Manages physics simulation for Digitbot character
 * 
 * Handles collision geometry, rigid body dynamics, and mass properties.
 */
class DigitbotPhysics {
public:
    DigitbotPhysics(PhysicsEngine* physics, JobManager* jobManager, TimeHandler* timeHandler);
    ~DigitbotPhysics();

    // Physics state access
    RigidBody* getRigidBody() const { return m_rigidBody; }
    glm::dvec3 getCenterOfMass() const { return m_centerOfMass; }

    // Collision box visualization
    void showCollisionBox(GraphicsEngine* graphics);
    void hideCollisionBox(GraphicsEngine* graphics);
    bool isCollisionBoxVisible() const { return m_collisionBoxMeshId != -1; }
    void updateCollisionBoxTransform(GraphicsEngine* graphics, uint64_t currentPhysicsTimeStep);

    // Coordinate transforms
    glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const;
    glm::dvec3 localToWorld(const glm::dvec3& localPos) const;

private:
    void createCollisionBoxMesh(GraphicsEngine* graphics);

    // Physics engine reference
    PhysicsEngine* m_physics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Physics state
    std::weak_ptr<GridCollider> m_colliderWeak;
    RigidBody* m_rigidBody;
    glm::dvec3 m_centerOfMass;

    // Debug visualization
    int m_collisionBoxMeshId;
};