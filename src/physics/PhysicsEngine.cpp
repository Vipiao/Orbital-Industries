// PhysicsEngine.cpp
#include "PhysicsEngine.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include "../debug/DebugRenderer.h"
#include "../utils/TimeHandler.h"

// RigidBody cached getter implementations
const glm::dmat3& PhysicsEngine::RigidBody::getOrientationMatrix() const {
    if (m_orientationMatrixDirty) {
        m_cachedOrientationMatrix = glm::mat3_cast(m_orientation);
        m_orientationMatrixDirty = false;
    }
    return m_cachedOrientationMatrix;
}

const glm::dvec3& PhysicsEngine::RigidBody::getAngularVelocityBody() const {
    if (m_angularVelocityDirty) {
        m_cachedAngularVelocityBody = m_invInertiaTensor * m_angularMomentumBody;
        m_angularVelocityDirty = false;
    }
    return m_cachedAngularVelocityBody;
}

const glm::dvec3& PhysicsEngine::RigidBody::getAngularVelocityWorld() const {
    if (m_angularVelocityDirty) {
        m_cachedAngularVelocityWorld = getOrientationMatrix() * getAngularVelocityBody();
        m_angularVelocityDirty = false;
    }
    return m_cachedAngularVelocityWorld;
}

const glm::dmat3& PhysicsEngine::RigidBody::getWorldInertiaTensor() const {
    if (m_worldInertiaDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInertiaTensor = orientationMatrix * m_inertiaTensor * glm::transpose(orientationMatrix);
        m_worldInertiaDirty = false;
    }
    return m_cachedWorldInertiaTensor;
}

const glm::dmat3& PhysicsEngine::RigidBody::getWorldInvInertiaTensor() const {
    if (m_worldInertiaDirty) {
        const glm::dmat3& orientationMatrix = getOrientationMatrix();
        m_cachedWorldInvInertiaTensor = orientationMatrix * m_invInertiaTensor * glm::transpose(orientationMatrix);
        m_worldInertiaDirty = false;
    }
    return m_cachedWorldInvInertiaTensor;
}

// RigidBody invalidation methods
void PhysicsEngine::RigidBody::invalidateOrientation() {
    m_orientationMatrixDirty = true;
    m_angularVelocityDirty = true;
    m_worldInertiaDirty = true;
}

void PhysicsEngine::RigidBody::invalidateAngularMomentum() {
    m_angularVelocityDirty = true;
}

void PhysicsEngine::RigidBody::invalidateInertiaTensor() {
    m_angularVelocityDirty = true;
    m_worldInertiaDirty = true;
}

PhysicsEngine::PhysicsEngine(TimeHandler* timeHandler)
    : m_timeHandler(timeHandler)
{
    if (!m_timeHandler) {
        throw std::invalid_argument("TimeHandler cannot be null");
    }
}

PhysicsEngine::~PhysicsEngine() {
    // Vector of unique_ptr will handle cleanup automatically
}

PhysicsEngine::RigidBody* PhysicsEngine::addRigidBody(const glm::dvec3& position, 
                               const glm::dquat& orientation,
                               double mass, 
                               const glm::dmat3& inertiaTensor,
                               bool isStatic,
                               Collider* collider) {
    
    auto body = std::make_unique<RigidBody>();
    body->m_position = position;
    body->m_velocity = glm::dvec3{0.0, 0.0, 0.0};
    body->m_forces = glm::dvec3{0.0, 0.0, 0.0};
    
    body->m_orientation = glm::normalize(orientation); // Ensure unit quaternion
    body->m_angularMomentumBody = glm::dvec3{0.0, 0.0, 0.0};
    body->m_torques = glm::dvec3{0.0, 0.0, 0.0};
    
    body->m_mass = mass;
    body->m_inertiaTensor = inertiaTensor;
    // Handle zero mass/inertia cases
    body->m_invMass = (mass > 1e-15) ? (1.0 / mass) : std::numeric_limits<double>::max();
    body->m_invInertiaTensor = (glm::determinant(inertiaTensor) > 1e-15) ? glm::inverse(inertiaTensor) : glm::dmat3(std::numeric_limits<double>::max());
    // Static bodies should have infinite mass (zero inverse mass)
    if (isStatic) {
        body->m_invMass = 0.0;
        body->m_invInertiaTensor = glm::dmat3(0.0);
    }
    body->m_isStatic = isStatic;
    body->m_collider = collider;
    body->m_colliderOffset = glm::dvec3{0.0, 0.0, 0.0}; // Initialize to zero offset

    RigidBody* bodyPtr = body.get();
     
    // Add collider to collision detection system if provided
    if (collider) {
        // Set the rigid body as the reference for this collider
        collider->m_reference = bodyPtr;
        m_collisionDetector.addCollider(collider);
    }

    m_rigidBodies.push_back(std::move(body));
    
    return bodyPtr;
}

void PhysicsEngine::removeRigidBody(RigidBody* bodyToRemove) {
    if (!bodyToRemove) return;

    // Find the body to get its collider before removal
    Collider* colliderToRemove = bodyToRemove->m_collider;

    // Clear the collider reference
    if (colliderToRemove && colliderToRemove->m_reference) {
        colliderToRemove->m_reference = nullptr;
    }
    
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
    if (body && !body->m_isStatic) {
        body->m_forces += force;
    }
}

void PhysicsEngine::applyForceAtPoint(RigidBody* body, const glm::dvec3& force, const glm::dvec3& point) {
    if (body && !body->m_isStatic) {
        // Add the force to overall forces
        body->m_forces += force;
        
        // Calculate torque: τ = r × F
        // where r is the vector from center of mass to point of application
        glm::dvec3 r = point - body->m_position;
        glm::dvec3 torque = glm::cross(r, force);
        body->m_torques += torque;
    }
}

void PhysicsEngine::applyTorque(RigidBody* body, const glm::dvec3& torque) {
    if (body && !body->m_isStatic) {
        body->m_torques += torque;
    }
}

void PhysicsEngine::setGravity(const glm::dvec3& gravity) {
    m_gravity = gravity;
}

void PhysicsEngine::updateColliderTransform(RigidBody* body) {
    if (body && body->m_collider) {
        body->m_collider->m_position = body->m_position - body->m_orientation * body->m_colliderOffset;
        body->m_collider->m_orientation = body->m_orientation;
    }
}

bool PhysicsEngine::runUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    while (m_timeHandler->now() < endTime) {
        switch (m_runState) {
            case RunState::APPLY_FORCES:
                applyForces();
                m_runState = RunState::UPDATE_POSITIONS;
                
                break;
                
            case RunState::UPDATE_POSITIONS:
                updatePositions();
                m_runState = RunState::HANDLE_COLLISIONS;
                m_collisionProcessState = CollisionProcessState::DETECT; // Reset collision state
                
                break;
                
            case RunState::HANDLE_COLLISIONS:
                if (handleCollisionsUntil(endTime)) {
                    return true; // Collision processing needs more time
                }
                m_runState = RunState::DONE;
                break;
                
            case RunState::DONE:
                // Physics step complete - reset state and increment counter
                m_currentPhysicsTimeStep++;
                m_runState = RunState::APPLY_FORCES;
                m_collisionProcessState = CollisionProcessState::DETECT;
                m_currentCollisionIndex = 0;
                m_separationIteration = 0;
                m_activeCollisions.clear();
                return false; // No more work needed
        }
    }

    return true; // Time ran out, more work needed
}

bool PhysicsEngine::handleCollisionsUntil(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    const size_t COLLISION_BATCH_SIZE = 5; // Process this many collisions before checking time
    
    while (m_timeHandler->now() < endTime) {
        switch (m_collisionProcessState) {
            case CollisionProcessState::DETECT:
                // Collect collision results
                m_activeCollisions.clear();
                m_collisionDetector.run(m_activeCollisions);
                m_currentCollisionIndex = 0;
                m_collisionProcessState = CollisionProcessState::RESOLVE;
                break;
                
            case CollisionProcessState::RESOLVE:
                // Process collision resolution in batches
                for (size_t i = 0; i < COLLISION_BATCH_SIZE && m_currentCollisionIndex < m_activeCollisions.size(); i++) {
                    resolveCollision(m_activeCollisions[m_currentCollisionIndex]);
                    m_currentCollisionIndex++;
                }
                
                if (m_currentCollisionIndex >= m_activeCollisions.size()) {
                    // All collisions resolved, move to separation
                    m_collisionProcessState = CollisionProcessState::SEPARATE;
                    m_separationIteration = 0;
                }
                break;
                
            case CollisionProcessState::SEPARATE:
                // Process separation iterations (originally 8 iterations)
                for (const auto& collision : m_activeCollisions) {
                    separateOverlaps(const_cast<CollisionResult&>(collision));
                }
                m_separationIteration++;
                
                if (m_separationIteration >= 8) {
                    m_collisionProcessState = CollisionProcessState::DONE;
                }
                break;
                
            case CollisionProcessState::DONE:
                return false; // Collision processing complete
        }
    }
    
    return true; // Time ran out, more work needed
}

void PhysicsEngine::applyForces() {
    // Apply gravity to each body
    for (auto& body : m_rigidBodies) {
        if (!body->m_isStatic) {
            // Apply gravitational force: F = m*g
            body->m_forces += m_gravity * body->m_mass;
        }
    }
}

void PhysicsEngine::updatePositions() {
    // Update positions based on physics
    for (auto& body : m_rigidBodies) {
        if (body->m_isStatic) {
            continue; // Static bodies don't move
        }
        
        // Linear motion update
        // Calculate acceleration: a = F/m
        glm::dvec3 acceleration = body->m_forces * body->m_invMass;
        
        // Update velocity: v = v + a
        // (deltaTime already incorporated into velocity)
        body->m_velocity += acceleration;
        
        // Update position: p = p + v
        // (deltaTime already incorporated into velocity)
        body->m_position += body->m_velocity;
        
        // Angular motion update using angular momentum (ensures conservation)
        // Transform world torques to body space and update angular momentum
        glm::dmat3 orientationMatrix = glm::mat3_cast(body->m_orientation);
        glm::dvec3 torquesBody = glm::transpose(orientationMatrix) * body->m_torques;

        // Apply external torques first, then correct for rotating coordinate system
        body->m_angularMomentumBody += torquesBody;
        double currentMagnitude = glm::length(body->m_angularMomentumBody);
        
        glm::dvec3 angularVelocityBody = body->m_invInertiaTensor * body->m_angularMomentumBody;
        glm::dvec3 fictitiousTorque = glm::cross(angularVelocityBody, body->m_angularMomentumBody);
        body->m_angularMomentumBody += -fictitiousTorque;
        double turnedMagnitude = glm::length(body->m_angularMomentumBody);
        if (turnedMagnitude > 0.) {
            body->m_angularMomentumBody *= currentMagnitude / turnedMagnitude;
        }
        
        // Calculate final angular velocity for orientation update
        angularVelocityBody = body->m_invInertiaTensor * body->m_angularMomentumBody;
        
        // Transform angular velocity back to world space for orientation update
        glm::dvec3 angularVelocity = orientationMatrix * angularVelocityBody;

        // Debug: Print global angular momentum to verify conservation
        //static int debugCounter = 0;
        //if (++debugCounter % 10 == 0) { // Print every ~2 seconds at 60fps
        //    glm::dvec3 globalAngularMomentum = orientationMatrix * body->m_angularMomentumBody;
        //    double globalMagnitude = glm::length(globalAngularMomentum);
        //    std::cout << "Global L: (" << globalAngularMomentum.x << ", " << globalAngularMomentum.y << ", " << globalAngularMomentum.z 
        //              << ") |L| = " << globalMagnitude << std::endl;
        //}
        
        // Update orientation: q = q + (q * ω) * 0.5
        double angle{ glm::length(angularVelocity) };
        glm::dquat angularVelocityQuat{};
        if (angle > 0.) {
            angularVelocityQuat = glm::angleAxis(angle, angularVelocity / angle);
        } else {
            angularVelocityQuat = glm::dquat{1, 0, 0, 0};
        }
        body->m_orientation = (angularVelocityQuat * body->m_orientation);
        body->m_orientation = glm::normalize(body->m_orientation); // Renormalize to prevent drift

        // Invalidate cached values after updating orientation and angular momentum
        body->invalidateOrientation();
        body->invalidateAngularMomentum();

        // Reset forces and torques for next frame
        body->m_forces = glm::dvec3{0.0, 0.0, 0.0};
        body->m_torques = glm::dvec3{0.0, 0.0, 0.0};
        
        // Update collider position and orientation if it exists
        updateColliderTransform(body.get());
    }
}

//static int ttt = 0;

void PhysicsEngine::resolveCollision(CollisionResult& collision) {
    // Find the rigid bodies associated with these colliders
    RigidBody* bodyA = static_cast<RigidBody*>(collision.m_colliderA->m_reference);
    RigidBody* bodyB = static_cast<RigidBody*>(collision.m_colliderB->m_reference);

    if (!bodyA || !bodyB) {
        return; // Skip if we can't find bodies
    }
    
    if (bodyA->m_isStatic && bodyB->m_isStatic) {
        return; // Skip if both bodies are static
    }
    //ttt += collision.m_normals.size();

    // Calculate collision masses if not already done
    if (!collision.m_collisionMassesCalculated) {
        collision.m_collisionMasses.reserve(collision.m_normals.size());
        for (size_t i = 0; i < collision.m_normals.size(); ++i) {
            double collisionMass = getCollisionMass(bodyA, bodyB, collision.m_contactPoints[i], collision.m_normals[i]);
            collision.m_collisionMasses.push_back(collisionMass);
        }
        collision.m_collisionMassesCalculated = true;
    }

    // Process each contact point
    for (size_t i = 0; i < collision.m_normals.size(); ++i) {
        glm::dvec3 normal = collision.m_normals[i];
        glm::dvec3 contactPoint = collision.m_contactPoints[i];
        
        // Calculate relative position vectors from center of mass to contact point
        glm::dvec3 rA = contactPoint - bodyA->m_position;
        glm::dvec3 rB = contactPoint - bodyB->m_position;

        // Calculate relative velocity at contact point
        glm::dvec3 velA = bodyA->m_velocity + glm::cross(bodyA->getAngularVelocityWorld(), rA);
        glm::dvec3 velB = bodyB->m_velocity + glm::cross(bodyB->getAngularVelocityWorld(), rB);
        glm::dvec3 relativeVel = velA - velB;
        
        // Project relative velocity onto collision normal
        double relativeVelNormal = glm::dot(relativeVel, normal);
        
        // Do not resolve if velocities are separating
        if (relativeVelNormal < 0) {
            continue;
        }
        
        // Get pre-calculated collision mass and calculate impulse
        double collisionMass = collision.m_collisionMasses[i];
        // Calculate impulse magnitude: -(1+e)*v_rel_normal * collision_mass
        double impulseMagnitude = -(1.0 + 0.0) * relativeVelNormal * collisionMass; // restitution = 0.0

        // Apply impulse
        glm::dvec3 impulse = normal * impulseMagnitude;
        
        // Apply impulse only to non-static bodies
        if (!bodyA->m_isStatic) {
            bodyA->m_velocity += impulse * bodyA->m_invMass;
            glm::dvec3 angularImpulseBody = glm::transpose(bodyA->getOrientationMatrix()) * glm::cross(rA, impulse);
            bodyA->m_angularMomentumBody += angularImpulseBody;
            bodyA->invalidateAngularMomentum();
        }
        
        if (!bodyB->m_isStatic) {
            bodyB->m_velocity -= impulse * bodyB->m_invMass;
            glm::dvec3 angularImpulseBody = glm::transpose(bodyB->getOrientationMatrix()) * glm::cross(rB, impulse);
            bodyB->m_angularMomentumBody -= angularImpulseBody;
            bodyB->invalidateAngularMomentum();
        }
    }
}

void PhysicsEngine::separateOverlaps(CollisionResult& collision) {
    // Find the rigid bodies associated with these colliders
    RigidBody* bodyA = static_cast<RigidBody*>(collision.m_colliderA->m_reference);
    RigidBody* bodyB = static_cast<RigidBody*>(collision.m_colliderB->m_reference);
     
    if (!bodyA || !bodyB) {
        return; // Skip if we can't find bodies
    }
    
    if (bodyA->m_isStatic && bodyB->m_isStatic) {
        return; // Skip if both bodies are static
    }

    // Use pre-calculated collision masses
    if (!collision.m_collisionMassesCalculated) {
        // This should already be calculated in resolveCollision, but just in case
        collision.m_collisionMasses.reserve(collision.m_normals.size());
        for (size_t i = 0; i < collision.m_normals.size(); ++i) {
            double collisionMass = getCollisionMass(bodyA, bodyB, collision.m_contactPoints[i], collision.m_normals[i]);
            collision.m_collisionMasses.push_back(collisionMass);
        }
        collision.m_collisionMassesCalculated = true;
    }

    // Process each contact point for position correction with dynamic overlap calculation
    for (size_t ii = 0; ii < collision.m_normals.size(); ++ii) {
        glm::dvec3 normal = collision.m_normals[ii];
        glm::dvec3 contactPoint = collision.m_contactPoints[ii];

        // Calculate current overlap using dynamic contact point positions
        // Transform local contact points back to current world space using collider methods
        glm::dvec3 currentContactA = collision.m_colliderA->localToWorld(collision.m_contactPointsLocalA[ii]);
        glm::dvec3 currentContactB = collision.m_colliderB->localToWorld(collision.m_contactPointsLocalB[ii]);
        
        // Calculate how much the contact points have separated since collision detection
        glm::dvec3 separation = currentContactA - currentContactB;
        double separationAlongNormal = glm::dot(separation, normal);
        
        // Adjust the overlap: positive separation means objects moved apart, so reduce overlap
        double overlap = collision.m_penetrationDepths[ii] + separationAlongNormal;
        
        // Only separate if there's positive overlap
        if (overlap <= 0) {
            continue;
        }
        
        // Get collision mass and calculate position correction "impulse"
        double collisionMass = collision.m_collisionMasses[ii];

        // Calculate position correction magnitude: overlap * collision_mass
        double margin = 0.03;
        double correctionMagnitude = (overlap - margin) * collisionMass;
        if (correctionMagnitude < 0.) {
            correctionMagnitude *= 0.1 / collision.m_normals.size();
        }

        // Calculate relative position vectors from center of mass to contact point
        glm::dvec3 rA = contactPoint - bodyA->m_position;
        glm::dvec3 rB = contactPoint - bodyB->m_position;

        // Apply position correction
        double scale = 0.3;
        glm::dvec3 correction = normal * correctionMagnitude * scale;

        // Apply linear position corrections only to non-static bodies
        if (!bodyA->m_isStatic) {
            bodyA->m_position -= correction * bodyA->m_invMass;
        }
        if (!bodyB->m_isStatic) {
            bodyB->m_position += correction * bodyB->m_invMass;
        }
        
        // Apply angular position corrections (to orientation)
        glm::dvec3 angularCorrectionA = -bodyA->getWorldInvInertiaTensor() * glm::cross(rA, correction);
        glm::dvec3 angularCorrectionB = +bodyB->getWorldInvInertiaTensor() * glm::cross(rB, correction);
        
        // Convert angular corrections to quaternion rotations and apply
        double angularCorrectionALengthSq = glm::dot(angularCorrectionA, angularCorrectionA);
        if (!bodyA->m_isStatic && angularCorrectionALengthSq > 1e-18) { // 1e-9 squared
            double angleA = glm::length(angularCorrectionA);
            glm::dvec3 axisA = angularCorrectionA / angleA;
            glm::dquat rotationA = glm::angleAxis(angleA, axisA);
            bodyA->m_orientation = rotationA * bodyA->m_orientation;
            bodyA->m_orientation = glm::normalize(bodyA->m_orientation);
            bodyA->invalidateOrientation();
        }
        
        double angularCorrectionBLengthSq = glm::dot(angularCorrectionB, angularCorrectionB);
        if (!bodyB->m_isStatic && angularCorrectionBLengthSq > 1e-18) { // 1e-9 squared
            double angleB = glm::sqrt(angularCorrectionBLengthSq);
           glm::dvec3 axisB = angularCorrectionB / angleB;
            glm::dquat rotationB = glm::angleAxis(angleB, axisB);
            bodyB->m_orientation = rotationB * bodyB->m_orientation;
            bodyB->m_orientation = glm::normalize(bodyB->m_orientation);
            bodyB->invalidateOrientation();
        }
        
        // Update collider transforms after position changes
        if (!bodyA->m_isStatic) {
            updateColliderTransform(bodyA);
        }
        if (!bodyB->m_isStatic) {
            updateColliderTransform(bodyB);
        }
    }
}


double PhysicsEngine::getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                      const glm::dvec3& contactPoint, const glm::dvec3& normal) {
    // Calculate relative position vectors from center of mass to contact point
    glm::dvec3 rA = contactPoint - bodyA->m_position;
    glm::dvec3 rB = contactPoint - bodyB->m_position;

    // Cross products with normal
    glm::dvec3 rA_cross_n = glm::cross(rA, normal);
    glm::dvec3 rB_cross_n = glm::cross(rB, normal);

    // Rotational contributions using inertia tensors: (r × n)ᵀ · I⁻¹ · (r × n)
    glm::dvec3 rotContribA = bodyA->getWorldInvInertiaTensor() * rA_cross_n;
    glm::dvec3 rotContribB = bodyB->getWorldInvInertiaTensor() * rB_cross_n;
    double rotTermA = glm::dot(rA_cross_n, rotContribA);
    double rotTermB = glm::dot(rB_cross_n, rotContribB);
    
    // Complete effective mass calculation
    double invEffectiveMass = bodyA->m_invMass + bodyB->m_invMass + rotTermA + rotTermB;
    return 1.0 / invEffectiveMass;
}