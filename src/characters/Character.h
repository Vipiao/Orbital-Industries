// Character.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

class PhysicsEngine;
class GraphicsEngine;
class JobManager;
class TimeHandler;
class RigidBody;

/**
 * @brief Abstract base class for all characters
 * 
 * Characters are entities with physics bodies, colliders, and graphics.
 * Each character type implements its own collision shape and visual appearance.
 */
class Character {
public:
    /**
     * @brief Construct a new Character
     * @param physics Physics engine reference
     * @param graphics Graphics engine reference
     * @param jobManager Job manager for async operations
     * @param timeHandler Time handler reference
     */
    Character(PhysicsEngine* physics, GraphicsEngine* graphics,
              JobManager* jobManager, TimeHandler* timeHandler);
    
    virtual ~Character();

    // Update hooks
    /**
     * @brief Called before rendering each frame
     * @param frameNum Current frame number
     * @param timeRemainder Interpolation parameter [0-1] since last physics step
     */
    virtual void preRenderCallback(uint64_t frameNum, double timeRemainder) = 0;
    
    /**
     * @brief Called after physics update completes
     */
    virtual void onPhysicsUpdateComplete() = 0;

    // Collision box visualization
    virtual void showCollisionBox() = 0;
    virtual void hideCollisionBox() = 0;
    virtual bool isCollisionBoxVisible() const = 0;

    // Access
    std::weak_ptr<RigidBody> getRigidBody() const { return m_rigidBody; }
    int getUniqueId() const { return m_uniqueId; }

    // Coordinate transforms
    virtual glm::dvec3 worldToLocal(const glm::dvec3& worldPos) const = 0;
    virtual glm::dvec3 localToWorld(const glm::dvec3& localPos) const = 0;

protected:
    // Engine references (non-owning)
    PhysicsEngine* m_physics;
    GraphicsEngine* m_graphics;
    JobManager* m_jobManager;
    TimeHandler* m_timeHandler;

    // Physics
    std::weak_ptr<RigidBody> m_rigidBody;
    //glm::dvec3 m_centerOfMass; // In local/body coordinates

    // Unique ID for deterministic sorting
    int m_uniqueId;
    static int s_nextId;
};
