// PhysicsEngine.cpp
#include "PhysicsEngine.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include "CollisionDetectionUtils.h"
#include <glm/gtx/quaternion.hpp>
#include "DebugRenderer.h"

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
    body->m_position = position;
    body->m_velocity = glm::dvec3{0.0, 0.0, 0.0};
    body->m_forces = glm::dvec3{0.0, 0.0, 0.0};
    
    body->m_orientation = glm::normalize(orientation); // Ensure unit quaternion
    body->m_angularVelocity = glm::dvec3{0.0, 0.0, 0.0};
    body->m_torques = glm::dvec3{0.0, 0.0, 0.0};
    
    body->m_mass = mass;
    body->m_momentOfInertia = momentOfInertia;
    // Handle zero mass/inertia cases
    body->m_invMass = (mass > 1e-15) ? (1.0 / mass) : std::numeric_limits<double>::max();
    body->m_invMomentOfInertia = (momentOfInertia > 1e-15) ? (1.0 / momentOfInertia) : std::numeric_limits<double>::max();
    // Static bodies should have infinite mass (zero inverse mass)
    if (isStatic) {
        body->m_invMass = 0.0;
        body->m_invMomentOfInertia = 0.0;
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
        
        // Angular motion update
        // Calculate angular acceleration: α = τ/I
        glm::dvec3 angularAcceleration = body->m_torques * body->m_invMomentOfInertia;
        
        // Update angular velocity: ω = ω + α
        // (deltaTime already incorporated into angular velocity)
        body->m_angularVelocity += angularAcceleration;
        
        // Update orientation: q = q + (q * ω) * 0.5
        // This is a simplified quaternion integration
        //glm::dquat angularVelocityQuat{0.0, 
        //                             body->m_angularVelocity.x, 
        //                             body->m_angularVelocity.y, 
        //                             body->m_angularVelocity.z};
        //body->m_orientation += (angularVelocityQuat * body->m_orientation) * 0.5;
        double angle{ glm::length(body->m_angularVelocity) };
        glm::dquat angularVelocityQuat{};
        if (angle > 0.) {
            angularVelocityQuat = glm::angleAxis(angle, body->m_angularVelocity / angle);
        } else {
            angularVelocityQuat = glm::dquat{1, 0, 0, 0};
        }
        body->m_orientation = (angularVelocityQuat * body->m_orientation);
        body->m_orientation = glm::normalize(body->m_orientation); // Renormalize to prevent drift
        
        // Reset forces and torques for next frame
        body->m_forces = glm::dvec3{0.0, 0.0, 0.0};
        body->m_torques = glm::dvec3{0.0, 0.0, 0.0};
        
        // Update collider position and orientation if it exists
        updateColliderTransform(body.get());
    }
}

//static int ttt = 0;

void PhysicsEngine::handleCollisions() {
    // Collect collision results
    std::vector<CollisionResult> collisions;
    m_collisionDetector.run(collisions);

    //for (size_t ii = 0; ii < 100; ii++) {
    //    m_debugRenderer->removeMesh("contactPoint " + std::to_string(ii));
    //}
    //if (collisions.size() > 0) {
    //    //RigidBody* bodyA = static_cast<RigidBody*>(collisions[0].m_colliderA->m_reference);
    //    //m_debugRenderer->createSphere("test", bodyA->m_position, 1.0);
    //    auto cc = collisions[0];
    //    //int mm = m_debugRenderer->createSphere("body", glm::dvec3{0,0,0}, 0.1);
    //    //m_debugRenderer->setScale(mm, glm::dvec3{1,0,0});
    //    //m_debugRenderer->createSphere("head", glm::dvec3{0,0,0} + cc.m_normals[0], 0.1);
    //    for (size_t ii = 0; ii < cc.m_contactPoints.size(); ii++)
    //    {
    //        m_debugRenderer->createSphere("contactPoint " + std::to_string(ii), cc.m_contactPoints[ii], 0.1);
    //    }
    //    
    //}
    
    
    // Resolve each collision momentum.
    //ttt = 0;
    for (const auto& collision : collisions) {
        resolveCollision(const_cast<CollisionResult&>(collision));
    }
    //std::cout << "ttt: " << ttt << std::endl;
    // Resolve each collision overlap.
    for (int ii=0; ii < 8; ii++) {
        for (const auto& collision : collisions) {
            separateOverlaps(const_cast<CollisionResult&>(collision));
        }
    }
}

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
        glm::dvec3 velA = bodyA->m_velocity + glm::cross(bodyA->m_angularVelocity, rA);
        glm::dvec3 velB = bodyB->m_velocity + glm::cross(bodyB->m_angularVelocity, rB);
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
        
        bodyA->m_velocity += impulse * bodyA->m_invMass;
        bodyA->m_angularVelocity += glm::cross(rA, impulse) * bodyA->m_invMomentOfInertia;
        
        bodyB->m_velocity -= impulse * bodyB->m_invMass;
        bodyB->m_angularVelocity -= glm::cross(rB, impulse) * bodyB->m_invMomentOfInertia;
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
            correctionMagnitude *= 0.04;
        }

        // Calculate relative position vectors from center of mass to contact point
        glm::dvec3 rA = contactPoint - bodyA->m_position;
        glm::dvec3 rB = contactPoint - bodyB->m_position;

        // Apply position correction
        double scale = 0.3;
        glm::dvec3 correction = normal * correctionMagnitude * scale;

        // Apply linear position corrections
        bodyA->m_position -= correction * bodyA->m_invMass;
        bodyB->m_position += correction * bodyB->m_invMass;
        
        // Apply angular position corrections (to orientation)
        glm::dvec3 angularCorrectionA = -glm::cross(rA, correction) * bodyA->m_invMomentOfInertia;
        glm::dvec3 angularCorrectionB = +glm::cross(rB, correction) * bodyB->m_invMomentOfInertia;
        
        // Convert angular corrections to quaternion rotations and apply
        double angularCorrectionALengthSq = glm::dot(angularCorrectionA, angularCorrectionA);
        if (angularCorrectionALengthSq > 1e-18) { // 1e-9 squared
            double angleA = glm::length(angularCorrectionA);
            glm::dvec3 axisA = angularCorrectionA / angleA;
            glm::dquat rotationA = glm::angleAxis(angleA, axisA);
            bodyA->m_orientation = rotationA * bodyA->m_orientation;
            bodyA->m_orientation = glm::normalize(bodyA->m_orientation);
        }
        
        double angularCorrectionBLengthSq = glm::dot(angularCorrectionB, angularCorrectionB);
        if (angularCorrectionBLengthSq > 1e-18) { // 1e-9 squared
            double angleB = glm::sqrt(angularCorrectionBLengthSq);
           glm::dvec3 axisB = angularCorrectionB / angleB;
            glm::dquat rotationB = glm::angleAxis(angleB, axisB);
            bodyB->m_orientation = rotationB * bodyB->m_orientation;
            bodyB->m_orientation = glm::normalize(bodyB->m_orientation);
        }
        
        // Update collider transforms after position changes
        updateColliderTransform(bodyA);
        updateColliderTransform(bodyB);
    }
}


double PhysicsEngine::getCollisionMass(RigidBody* bodyA, RigidBody* bodyB, 
                                      const glm::dvec3& contactPoint, const glm::dvec3& normal) {
    // Calculate relative position vectors from center of mass to contact point
    glm::dvec3 rA = contactPoint - bodyA->m_position;
    glm::dvec3 rB = contactPoint - bodyB->m_position;

    // Cross products for rotational terms
    glm::dvec3 rAcrossN = glm::cross(rA, normal);
    glm::dvec3 rBcrossN = glm::cross(rB, normal);
    
    // Calculate collision mass using the formula: 1/(invMassA + invMassB + (rA x n)²*invInertiaA + (rB x n)²*invInertiaB)
    double collisionMass = 1.0 / (
        bodyA->m_invMass + bodyB->m_invMass + 
        glm::dot(rAcrossN, rAcrossN) * bodyA->m_invMomentOfInertia + 
        glm::dot(rBcrossN, rBcrossN) * bodyB->m_invMomentOfInertia
    );
    
    return collisionMass;
}