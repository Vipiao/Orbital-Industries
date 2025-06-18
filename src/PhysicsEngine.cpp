// PhysicsEngine.cpp
#include "PhysicsEngine.h"
#include <iostream>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>

PhysicsEngine::PhysicsEngine() 
{
}

PhysicsEngine::~PhysicsEngine() {
    // Vector of unique_ptr will handle cleanup automatically
}

PhysicsEngine::RigidBody* PhysicsEngine::addRigidBody(const glm::dvec3& position, 
                               const glm::dquat& orientation,
                               double mass, 
                               double momentOfInertia,
                               bool isStatic,
                               Collider* collider) {
    
    auto body = std::make_unique<RigidBody>();
    body->position = position;
    body->velocity = glm::dvec3{0.0, 0.0, 0.0};
    body->forces = glm::dvec3{0.0, 0.0, 0.0};
    
    body->orientation = glm::normalize(orientation); // Ensure unit quaternion
    body->angularVelocity = glm::dvec3{0.0, 0.0, 0.0};
    body->torques = glm::dvec3{0.0, 0.0, 0.0};
    
    body->mass = mass;
    body->momentOfInertia = momentOfInertia;
    body->isStatic = isStatic;
    body->collider = collider;
    body->colliderOffset = glm::dvec3{0.0, 0.0, 0.0}; // Initialize to zero offset
     
    // Add collider to collision detection system if provided
    if (collider) {
        m_collisionDetector.addCollider(collider);
    }

    RigidBody* bodyPtr = body.get();
    
    m_rigidBodies.push_back(std::move(body));
    
    return bodyPtr;
}

void PhysicsEngine::removeRigidBody(RigidBody* bodyToRemove) {
    if (!bodyToRemove) return;

    // Find the body to get its collider before removal
    Collider* colliderToRemove = bodyToRemove->collider;
    
    // Remove from rigid bodies
    auto removeIt = std::remove_if(m_rigidBodies.begin(), m_rigidBodies.end(),
        [bodyToRemove](const std::unique_ptr<RigidBody>& body) {
            return body.get() == bodyToRemove;
    });
     
    if (removeIt != m_rigidBodies.end()) {
        m_rigidBodies.erase(removeIt, m_rigidBodies.end());
    }
    
    // Remove collider from collision detection system
    if (colliderToRemove) {
        m_collisionDetector.removeCollider(colliderToRemove);
    }
}

void PhysicsEngine::applyForce(RigidBody* body, const glm::dvec3& force) {
    if (body && !body->isStatic) {
        body->forces += force;
    }
}

void PhysicsEngine::applyForceAtPoint(RigidBody* body, const glm::dvec3& force, const glm::dvec3& point) {
    if (body && !body->isStatic) {
        // Add the force to overall forces
        body->forces += force;
        
        // Calculate torque: τ = r × F
        // where r is the vector from center of mass to point of application
        glm::dvec3 r = point - body->position;
        glm::dvec3 torque = glm::cross(r, force);
        body->torques += torque;
    }
}

void PhysicsEngine::applyTorque(RigidBody* body, const glm::dvec3& torque) {
    if (body && !body->isStatic) {
        body->torques += torque;
    }
}

void PhysicsEngine::setGravity(const glm::dvec3& gravity) {
    m_gravity = gravity;
}

void PhysicsEngine::updateColliderTransform(RigidBody* body) {
    if (body && body->collider) {
        body->collider->m_position = body->position - body->orientation * body->colliderOffset;
        body->collider->m_orientation = body->orientation;
    }
}

void PhysicsEngine::run() {
    // Run one step of physics simulation
    applyForces();
    updatePositions();
    handleCollisions();
    
    // New - increment physics time step
    m_currentPhysicsTimeStep++;
}

void PhysicsEngine::applyForces() {
    // Apply gravity to each body
    for (auto& body : m_rigidBodies) {
        if (!body->isStatic) {
            // Apply gravitational force: F = m*g
            body->forces += m_gravity * body->mass;
        }
    }
}

void PhysicsEngine::updatePositions() {
    // Update positions based on physics
    for (auto& body : m_rigidBodies) {
        if (body->isStatic) {
            continue; // Static bodies don't move
        }
        
        // Linear motion update
        // Calculate acceleration: a = F/m
        glm::dvec3 acceleration = body->forces / body->mass;
        
        // Update velocity: v = v + a
        // (deltaTime already incorporated into velocity)
        body->velocity += acceleration;
        
        // Update position: p = p + v
        // (deltaTime already incorporated into velocity)
        body->position += body->velocity;
        
        // Angular motion update
        // Calculate angular acceleration: α = τ/I
        glm::dvec3 angularAcceleration = body->torques / body->momentOfInertia;
        
        // Update angular velocity: ω = ω + α
        // (deltaTime already incorporated into angular velocity)
        body->angularVelocity += angularAcceleration;
        
        // Update orientation: q = q + (q * ω) * 0.5
        // This is a simplified quaternion integration
        //glm::dquat angularVelocityQuat{0.0, 
        //                             body->angularVelocity.x, 
        //                             body->angularVelocity.y, 
        //                             body->angularVelocity.z};
        //body->orientation += (angularVelocityQuat * body->orientation) * 0.5;
        double angle{ glm::length(body->angularVelocity) };
        glm::dquat angularVelocityQuat{};
        if (angle > 0.) {
            angularVelocityQuat = glm::angleAxis(angle, body->angularVelocity / angle);
        } else {
            angularVelocityQuat = glm::dquat{1, 0, 0, 0};
        }
        body->orientation = (angularVelocityQuat * body->orientation);
        body->orientation = glm::normalize(body->orientation); // Renormalize to prevent drift
        
        // Reset forces and torques for next frame
        body->forces = glm::dvec3{0.0, 0.0, 0.0};
        body->torques = glm::dvec3{0.0, 0.0, 0.0};
        
        // Update collider position and orientation if it exists
        updateColliderTransform(body.get());
    }
}

void PhysicsEngine::handleCollisions() {
    // Collect collision results
    std::vector<CollisionResult> collisions;
    m_collisionDetector.run(collisions);
    
    // Resolve each collision
    for (const auto& collision : collisions) {
        resolveCollision(const_cast<CollisionResult&>(collision));
    }
}

void PhysicsEngine::resolveCollision(CollisionResult& collision) {
    // Find the rigid bodies associated with these colliders
    RigidBody* bodyA = nullptr;
    RigidBody* bodyB = nullptr;
    
    for (auto& body : m_rigidBodies) {
        if (body->collider == collision.colliderA) {
            bodyA = body.get();
        }
        if (body->collider == collision.colliderB) {
            bodyB = body.get();
        }
    }
    
    if (!bodyA || !bodyB) {
        return; // Skip if we can't find both bodies or both are static
    }

    // Calculate collision masses if not already done
    if (!collision.collisionMassesCalculated) {
        collision.collisionMasses.reserve(collision.normals.size());
        for (size_t i = 0; i < collision.normals.size(); ++i) {
            double collisionMass = getCollisionMass(bodyA, bodyB, collision.contactPoints[i], collision.normals[i]);
            collision.collisionMasses.push_back(collisionMass);
        }
        collision.collisionMassesCalculated = true;
    }

    // Process each contact point
    for (size_t i = 0; i < collision.normals.size(); ++i) {
        glm::dvec3 normal = collision.normals[i];
        glm::dvec3 contactPoint = collision.contactPoints[i];
        
        // Calculate relative position vectors from center of mass to contact point
        glm::dvec3 rA = contactPoint - bodyA->position;
        glm::dvec3 rB = contactPoint - bodyB->position;
        
        // Calculate relative velocity at contact point
        glm::dvec3 velA = bodyA->velocity + glm::cross(bodyA->angularVelocity, rA);
        glm::dvec3 velB = bodyB->velocity + glm::cross(bodyB->angularVelocity, rB);
        glm::dvec3 relativeVel = velA - velB;
        
        // Project relative velocity onto collision normal
        double relativeVelNormal = glm::dot(relativeVel, normal);
        
        // Do not resolve if velocities are separating
        if (relativeVelNormal > 0) {
            continue;
        }
        
        // Get pre-calculated collision mass and calculate impulse
        double collisionMass = collision.collisionMasses[i];
        double impulseMagnitude = getImpulse(bodyA, bodyB, contactPoint, normal, collisionMass);

        // Calculate inverse masses and inertias
        double invMassA = 1.0 / bodyA->mass;
        double invMassB = 1.0 / bodyB->mass;
        double invInertiaA = 1.0 / bodyA->momentOfInertia;
        double invInertiaB = 1.0 / bodyB->momentOfInertia;

        // Apply impulse
        glm::dvec3 impulse = normal * impulseMagnitude;
        
        bodyA->velocity += impulse * invMassA;
        bodyA->angularVelocity += glm::cross(rA, impulse) * invInertiaA;
        
        bodyB->velocity -= impulse * invMassB;
        bodyB->angularVelocity -= glm::cross(rB, impulse) * invInertiaB;
    }
}

double PhysicsEngine::getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                      const glm::dvec3& contactPoint, const glm::dvec3& normal) {
    // Calculate relative position vectors from center of mass to contact point
    glm::dvec3 rA = contactPoint - bodyA->position;
    glm::dvec3 rB = contactPoint - bodyB->position;
    
    // Calculate inverse masses and inertias
    double invMassA = 1.0 / bodyA->mass;
    double invMassB = 1.0 / bodyB->mass;
    double invInertiaA = 1.0 / bodyA->momentOfInertia;
    double invInertiaB = 1.0 / bodyB->momentOfInertia;

    // Cross products for rotational terms
    glm::dvec3 rAcrossN = glm::cross(rA, normal);
    glm::dvec3 rBcrossN = glm::cross(rB, normal);
    
    // Calculate collision mass using the formula: 1/(invMassA + invMassB + (rA x n)²*invInertiaA + (rB x n)²*invInertiaB)
    double collisionMass = 1.0 / (
        invMassA + invMassB + 
        glm::dot(rAcrossN, rAcrossN) * invInertiaA + 
        glm::dot(rBcrossN, rBcrossN) * invInertiaB
    );
    
    return collisionMass;
}

double PhysicsEngine::getImpulse(RigidBody* bodyA, RigidBody* bodyB, const glm::dvec3& contactPoint,
                                const glm::dvec3& normal, double collisionMass, double restitution) {
    // Calculate relative position vectors from center of mass to contact point
    glm::dvec3 rA = contactPoint - bodyA->position;
    glm::dvec3 rB = contactPoint - bodyB->position;
    
    // Calculate relative velocity at contact point
    glm::dvec3 velA = bodyA->velocity + glm::cross(bodyA->angularVelocity, rA);
    glm::dvec3 velB = bodyB->velocity + glm::cross(bodyB->angularVelocity, rB);
    glm::dvec3 relativeVel = velA - velB;
    
    // Project relative velocity onto collision normal
    double relativeVelNormal = glm::dot(relativeVel, normal);
    
    // Calculate impulse magnitude: -(1+e)*v_rel_normal * collision_mass
    double impulseMagnitude = -(1.0 + restitution) * relativeVelNormal * collisionMass;
    
    return impulseMagnitude;
}