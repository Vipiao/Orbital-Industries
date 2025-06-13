// PhysicsEngine.h
#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "CollisionDetector.h"

class PhysicsEngine {
public:
    // Simple rigid body structure
    struct RigidBody {
        int id;                     // Unique identifier
        glm::dvec3 position;        // Position in world space
        glm::dvec3 velocity;        // Linear velocity (already includes deltaTime)
        glm::dvec3 forces;          // Accumulated forces
        
        glm::dquat orientation;     // Orientation quaternion
        glm::dvec3 angularVelocity; // Angular velocity (already includes deltaTime)
        glm::dvec3 torques;         // Accumulated torques
        
        double mass;                // Mass in kg
        double momentOfInertia;     // Simplified moment of inertia (scalar)
        
        bool isStatic;              // If true, this body won't move

        Collider* collider;         // Associated collider for collision detection
    };
    
    PhysicsEngine();
    ~PhysicsEngine();
    
    // Add a rigid body to the simulation
    int addRigidBody(const glm::dvec3& position, 
                     const glm::dquat& orientation,
                     double mass = 1.0, 
                     double momentOfInertia = 1.0,
                     bool isStatic = false,
                     Collider* collider = nullptr);
    
    //
    uint64_t getCurrentPhysicsTimeStep() const { return m_currentPhysicsTimeStep; }
    
    // Remove a rigid body from the simulation
    void removeRigidBody(int id);
    
    // Get a rigid body by ID
    RigidBody* getRigidBody(int id);
    
    // Apply a force at the center of mass
    void applyForce(int id, const glm::dvec3& force);
    
    // Apply a force at a specific point (will generate torque)
    void applyForceAtPoint(int id, const glm::dvec3& force, const glm::dvec3& point);
    
    // Apply a torque directly
    void applyTorque(int id, const glm::dvec3& torque);
    
    // Set gravity
    void setGravity(const glm::dvec3& gravity);
    
    // Run physics simulation
    void run();
    
private:
    // Physics simulation steps
    void applyForces();
    void updatePositions();
    void handleCollisions();
    void resolveCollision(CollisionResult& collision);
    
    // Static helper functions for collision resolution
    static double getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                  const glm::dvec3& contactPoint, const glm::dvec3& normal);
    static double getImpulse(RigidBody* bodyA, RigidBody* bodyB, const glm::dvec3& contactPoint,
                            const glm::dvec3& normal, double collisionMass, double restitution = 0.0);

    
    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;
    int m_nextBodyId;
    glm::dvec3 m_gravity{0.0, 0.0, 0.0}; // Default zero gravity
    uint64_t m_currentPhysicsTimeStep{0};

    CollisionDetector m_collisionDetector;
};