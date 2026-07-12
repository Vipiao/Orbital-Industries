// DigibotPhysics.h
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
class Collider;

/**
 * @brief Manages physics simulation for Digibot character
 * 
 * Handles collision geometry, rigid body dynamics, and mass properties.
 */
class DigibotPhysics {
public:
    DigibotPhysics(PhysicsEngine* physics, JobManager* jobManager, TimeHandler* timeHandler,
                   int colliderClassificationJobPriority);
    ~DigibotPhysics();

    // Physics state access
    std::weak_ptr<RigidBody> getRigidBody() const { return m_rigidBody; }
    //glm::dvec3 getCenterOfMass() const { return m_centerOfMass; }

    // Collision box visualization
    void showCollisionBox(GraphicsEngine* graphics);
    void hideCollisionBox(GraphicsEngine* graphics);
    bool isCollisionBoxVisible() const { return m_collisionBoxMeshId != -1; }
    void updateCollisionBoxTransform(GraphicsEngine* graphics, uint64_t currentPhysicsTimeStep);

    // Coordinate transforms
    // Walking sensor for ground detection
    std::weak_ptr<Collider> getWalkingSensor() const { return m_walkingSensor; }
    
    // Get walking sensor radius
    double getWalkingSensorRadius() const { return m_walkingSensorRadius; }

    // Collision exceptions between the digibot body and another collider (typically a single
    // cockpit cell). Every body cell ignores the target, so the whole digibot phases through
    // it while still colliding with everything else. clearBodyCollisionExceptions drops them
    // all again; the body only ever holds docking exceptions, so clearing is exact.
    void addBodyCollisionExceptionWith(Collider* other);
    void clearBodyCollisionExceptions();

    glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const;
    glm::dvec3 localToWorld(const glm::dvec3& localPos) const;

private:
    void createCollisionBoxMesh(GraphicsEngine* graphics);

    // Physics engine reference
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics{nullptr};  // Held only for collision box debug visualization cleanup in destructor
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Physics state
    std::weak_ptr<GridCollider> m_colliderWeak;
    std::weak_ptr<RigidBody> m_rigidBody;
    // Store collider local transform for coordinate conversions
    glm::dvec3 m_colliderLocalPosition;
    //glm::dvec3 m_centerOfMass;

    // Debug visualization
    int m_collisionBoxMeshId;

    // Walking sensor
    std::weak_ptr<Collider> m_walkingSensor;

    // Walking sensor configuration
    double m_walkingSensorRadius;
};