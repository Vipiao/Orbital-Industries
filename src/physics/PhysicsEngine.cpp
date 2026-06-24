// PhysicsEngine.cpp
#include "PhysicsEngine.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>
#include "../debug/DebugRenderer.h"
#include "../utils/TimeHandler.h"

PhysicsEngine::PhysicsEngine(TimeHandler* timeHandler)
    : m_timeHandler(timeHandler), m_collisionDetector(timeHandler),
      m_lastPhysicsStepTime(timeHandler->now())
{
    if (!m_timeHandler) {
        throw std::invalid_argument("TimeHandler cannot be null");
    }
}

PhysicsEngine::~PhysicsEngine() {
    // Vector of unique_ptr will handle cleanup automatically
}

std::weak_ptr<RigidBody> PhysicsEngine::addRigidBody(const glm::dvec3& position,
                                                      const glm::dquat& orientation,
                                                      double mass, 
                                                      const glm::dmat3& inertiaTensor,
                                                      bool isStatic) {
    
    auto body = std::make_shared<RigidBody>();
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

    m_rigidBodies.push_back(std::move(body));
    
    return m_rigidBodies.back();
}

void PhysicsEngine::attachCollider(std::weak_ptr<RigidBody> bodyWeak, std::weak_ptr<Collider> colliderWeak,
                                   const glm::dvec3& localPosition,
                                   const glm::dquat& localOrientation,
                                   bool isTrigger) {
    auto body = bodyWeak.lock();
    if (!body) return;
    
    auto collider = colliderWeak.lock();
    if (!collider) return;

    // Check if collider is already attached to any body
    if (collider->get_pointer<ColliderAttachment>() != nullptr) {
        throw std::runtime_error("PhysicsEngine::attachCollider: Collider is already attached to a rigid body");
    }

    // Create attachment
    auto attachment = std::make_unique<ColliderAttachment>();
    attachment->rigidBody = bodyWeak;
    attachment->collider = colliderWeak;
    attachment->localPosition = localPosition;
    attachment->localOrientation = glm::normalize(localOrientation);
    attachment->isTrigger = isTrigger;
    
    // Set back-reference in collider via PointerStorage
    ColliderAttachment* attachmentPtr = attachment.get();
    collider->set_pointer<ColliderAttachment>(attachmentPtr);
    
    // Store attachment in body
    body->m_attachments.push_back(std::move(attachment));
    
    // Update collider transform immediately
    glm::dvec3 worldPosition = body->m_position + body->m_orientation * localPosition;
    glm::dquat worldOrientation = body->m_orientation * localOrientation;
    collider->m_position = worldPosition;
    collider->m_orientation = worldOrientation;
}

void PhysicsEngine::detachCollider(std::weak_ptr<RigidBody> bodyWeak, Collider* collider) {
    auto body = bodyWeak.lock();
    if (!body || !collider) return;
    
    // Find and remove the attachment
    for (auto it = body->m_attachments.begin(); it != body->m_attachments.end(); ) {
        auto attachedCollider = (*it)->collider.lock();
        if (attachedCollider && attachedCollider.get() == collider) {
            // Clear back-reference before removing
            attachedCollider->remove_pointer<ColliderAttachment>();
            it = body->m_attachments.erase(it);
            return; // Found and removed, exit
        } else {
            ++it;
        }
    }
}

void PhysicsEngine::detachAllColliders(std::weak_ptr<RigidBody> bodyWeak) {
    auto body = bodyWeak.lock();
    if (!body) return;
    
    // Clear all back-references
    for (auto& attachment : body->m_attachments) {
        auto collider = attachment->collider.lock();
        if (collider) {
            collider->remove_pointer<ColliderAttachment>();
        }
    }
    
    body->m_attachments.clear();
}

void PhysicsEngine::updateColliderTransform(std::weak_ptr<RigidBody> bodyWeak) {
    auto body = bodyWeak.lock();
    if (!body) return;
    
    for (auto& attachment : body->m_attachments) {
        auto collider = attachment->collider.lock();
        if (!collider) continue;
        
        // Transform local offset to world space
        glm::dvec3 worldPosition = body->m_position + body->m_orientation * attachment->localPosition;
        glm::dquat worldOrientation = body->m_orientation * attachment->localOrientation;
        
        collider->m_position = worldPosition;
        collider->m_orientation = worldOrientation;
    }
}

void PhysicsEngine::removeRigidBody(std::weak_ptr<RigidBody> bodyWeak) {
    auto bodyToRemove = bodyWeak.lock();
    if (!bodyToRemove) return;

    // Detach all colliders before removing
    detachAllColliders(bodyWeak);
    
    // Remove from rigid bodies
    auto removeIt = std::remove_if(m_rigidBodies.begin(), m_rigidBodies.end(),
        [bodyToRemove](const std::shared_ptr<RigidBody>& body) {
            return body.get() == bodyToRemove.get();
    });
     
    if (removeIt != m_rigidBodies.end()) {
        m_rigidBodies.erase(removeIt, m_rigidBodies.end());
    }
}

void PhysicsEngine::applyForce(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& force) {
    auto body = bodyWeak.lock();
    if (!body || body->m_isStatic) return;
        body->m_forces += force;
}

void PhysicsEngine::applyForceAtPoint(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& force, const glm::dvec3& point) {
    auto body = bodyWeak.lock();
    if (!body || body->m_isStatic) return;

        // Add the force to overall forces
        body->m_forces += force;
        
        // Calculate torque: τ = r × F
        // where r is the vector from center of mass to point of application
        glm::dvec3 r = point - body->m_position;
        glm::dvec3 torque = glm::cross(r, force);
        body->m_torques += torque;
    }
}

void PhysicsEngine::applyTorque(std::weak_ptr<RigidBody> bodyWeak, const glm::dvec3& torque) {
    auto body = bodyWeak.lock();
    if (body && !body->m_isStatic) {
        body->m_torques += torque;
    }
}

void PhysicsEngine::setGravity(const glm::dvec3& gravity) {
    m_gravity = gravity;
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

                // Increment physics time step immediately after position update
                // but before collision detection for the next frame
                m_currentPhysicsTimeStep++; // Increment physics time step immediately after position update
                
                // Set timestep for collision detection to the new timestep
                m_collisionDetector.setTimestep(m_currentPhysicsTimeStep);

                // Record exact time when the physics step completes
                m_lastPhysicsStepTime = m_timeHandler->now();

                m_runState = RunState::HANDLE_COLLISIONS;
                break;
                
            case RunState::HANDLE_COLLISIONS:
                if (handleCollisionsUntil(endTime)) {
                    return true; // Collision processing needs more time
                }
                m_runState = RunState::DONE;
                break;
                
            case RunState::DONE:
                // Physics step complete - reset state and increment counter
                m_runState = RunState::APPLY_FORCES;
                m_collisionProcessState = CollisionProcessState::DETECT;
                m_separationIteration = 0;
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
                if (!m_collisionGenerator) {
                    m_collisionGenerator = std::make_unique<Generator<bool>>(m_collisionDetector.run());
                }
                
                // Update end time (for new calls or resuming)
                m_collisionDetector.setEndTime(endTime);
                
                // Continue collision detection
                ++(*m_collisionGenerator);
                
                if (!*m_collisionGenerator) {
                    // Collision detection complete
                    m_collisionGenerator.reset();
                    m_collisionProcessState = CollisionProcessState::RESOLVE;
                } else {
                    // Generator yielded - needs more time
                    return true;
                }
                break;
                
            case CollisionProcessState::RESOLVE:
                // Process rigid bodies in batches
                {
                    static size_t currentBodyIndex = 0;
                    if (currentBodyIndex == 0) {
                        // Starting resolution phase
                        currentBodyIndex = 0;
                    }
                    
                    for (size_t i = 0; i < COLLISION_BATCH_SIZE && currentBodyIndex < m_rigidBodies.size(); i++) {
                        resolveCollision(m_rigidBodies[currentBodyIndex]);
                        currentBodyIndex++;
                    }
                    
                    if (currentBodyIndex >= m_rigidBodies.size()) {
                        currentBodyIndex = 0;
                        m_collisionProcessState = CollisionProcessState::SEPARATE;
                        m_separationIteration = 0;
                    }
                }
                
                break;
                
            case CollisionProcessState::SEPARATE:
                // Process separation iterations
                for (auto& body : m_rigidBodies) {
                    separateOverlaps(body);
                }
                m_separationIteration++;
                
                if (m_separationIteration >= 4) {
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

        // Skip physics calculations for zero mass bodies
        if (body->m_mass == 0.0) {
            body->m_forces = glm::dvec3{0.0, 0.0, 0.0};
            body->m_torques = glm::dvec3{0.0, 0.0, 0.0};
            continue;
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
        
        updateColliderTransform(body);
    }
}

//static int ttt = 0;

void PhysicsEngine::resolveCollision(std::shared_ptr<RigidBody> bodyShared) {
    RigidBody* body = bodyShared.get();
    if (!body || body->m_isStatic) {
        return;
    }
 
    // Iterate through all attached colliders
    for (auto& attachment : body->m_attachments) {
        if (attachment->isTrigger) {
            continue; // Skip triggers
        }
        
        auto collider = attachment->collider.lock();
        if (!collider) {
            continue;
        }
        
        // Get all collisions for this collider
        const auto& collisions = collider->getCollisions(m_currentPhysicsTimeStep);
        
        for (const auto& collision : collisions) {
            // Avoid processing same collision twice - only process if our ID is lower
            if (collider->m_debugId >= collision.otherCollider->m_debugId) {
                continue;
            }
            
            // Get other collider's attachment
            ColliderAttachment* otherAttachment = 
                collision.otherCollider->get_pointer<ColliderAttachment>();
            if (!otherAttachment || otherAttachment->isTrigger) {
                continue; // Other is trigger or not attached
            }
            
            auto otherBody = otherAttachment->rigidBody.lock();
            if (!otherBody || otherBody->m_mass == 0.0) {
                continue;
            }
            
            if (otherBody && otherBody->m_isStatic) {
                continue;
            }
            
            // Process each contact point
            for (size_t i = 0; i < collision.contactData.size(); ++i) {
                const ContactData& contact = collision.contactData[i];
                glm::dvec3 normal = contact.normal;
                glm::dvec3 contactPoint = collision.contactPoints[i];
                
                // Calculate relative position vectors
                glm::dvec3 rA = contactPoint - body->m_position;
                glm::dvec3 rB = contactPoint - otherBody->m_position;
                
                // Calculate relative velocity at contact point
                glm::dvec3 velA = body->m_velocity + glm::cross(body->getAngularVelocityWorld(), rA);
                glm::dvec3 velB = otherBody->m_velocity + glm::cross(otherBody->getAngularVelocityWorld(), rB);
                glm::dvec3 relativeVel = velA - velB;
                
                // Project relative velocity onto collision normal
                double relativeVelNormal = glm::dot(relativeVel, normal);
                
                // Do not resolve if velocities are separating
                if (relativeVelNormal < 0) {
                    continue;
                }
                
                // Calculate collision mass and impulse
                double collisionMass = getCollisionMass(body, otherBody.get(), contactPoint, normal);
                double impulseMagnitude = -(1.0 + 0.0) * relativeVelNormal * collisionMass;
                
                glm::dvec3 impulse = normal * impulseMagnitude;
                
                // Apply compliant collision handling
                if (shouldUseCompliantHandling(body, otherBody.get(), contactPoint, normal, 
                    contact.compliantNormal, contact.compliantPenetration, &relativeVel)) {
                    glm::dvec3 compliantNormal = contact.compliantNormal;
                    double normalAlignment = glm::dot(normal, compliantNormal);
                    double scaleFactor = 1.0;
                    if (true || normalAlignment > 0) {
                        scaleFactor = normalAlignment * normalAlignment;
                    }
                    impulse *= scaleFactor;
                }
                
                // Apply impulse to both bodies
                body->m_velocity += impulse * body->m_invMass;
                glm::dvec3 angularImpulseBodyA = glm::transpose(body->getOrientationMatrix()) * 
                    glm::cross(rA, impulse);
                body->m_angularMomentumBody += angularImpulseBodyA;
                body->invalidateAngularMomentum();
                
                if (!otherBody->m_isStatic) {
                    otherBody->m_velocity -= impulse * otherBody->m_invMass;
                    glm::dvec3 angularImpulseBodyB = glm::transpose(otherBody->getOrientationMatrix()) * 
                        glm::cross(rB, impulse);
                    otherBody->m_angularMomentumBody -= angularImpulseBodyB;
                    otherBody->invalidateAngularMomentum();
                }
            }
        }
    }
}

void PhysicsEngine::separateOverlaps(std::shared_ptr<RigidBody> bodyShared) {
    RigidBody* body = bodyShared.get();
    if (!body || body->m_isStatic) {
        return;
    }
    
    // Iterate through all attached colliders
    for (auto& attachment : body->m_attachments) {
        if (attachment->isTrigger) {
            continue; // Skip triggers
        }
        
        auto collider = attachment->collider.lock();
        if (!collider) {
            continue;
        }

         // Get all collisions for this collider
        const auto& collisions = collider->getCollisions(m_currentPhysicsTimeStep);
        
        for (const auto& collision : collisions) {
            // Avoid processing same collision twice
            if (collider->m_debugId >= collision.otherCollider->m_debugId) {
                continue;
            }
            
            // Get other collider's attachment
            ColliderAttachment* otherAttachment = 
                collision.otherCollider->get_pointer<ColliderAttachment>();
            if (!otherAttachment || otherAttachment->isTrigger) {
                continue;
            }
            
            auto otherBody = otherAttachment->rigidBody.lock();
            if (!otherBody || otherBody->m_mass == 0.0) {
                continue;
            }
            
            if (otherBody && otherBody->m_isStatic) {
                continue;
            }
            
            // Process each contact point for position correction
            for (size_t ii = 0; ii < collision.contactData.size(); ++ii) {
                const ContactData& contact = collision.contactData[ii];
                glm::dvec3 normal = contact.normal;
                glm::dvec3 contactPoint = collision.contactPoints[ii];
                
                // Skip overlap correction for contacts that would use compliant handling
                if (shouldUseCompliantHandling(body, otherBody.get(), contactPoint, normal,
                    contact.compliantNormal, contact.compliantPenetration)) {
                    continue;
                }
                
                // Calculate current overlap using dynamic contact point positions
                glm::dvec3 currentContactA = collider->localToWorld(collision.contactPointsLocalA[ii]);
                glm::dvec3 currentContactB = collision.otherCollider->localToWorld(collision.contactPointsLocalB[ii]);
                
                // Calculate separation
                glm::dvec3 separation = currentContactA - currentContactB;
                double separationAlongNormal = glm::dot(separation, normal);
                
                // Adjust the overlap
                double overlap = contact.penetration + separationAlongNormal;
                
                // Only separate if there's positive overlap
                if (overlap <= 0) {
                    continue;
                }
                
                // Calculate collision mass
                double collisionMass = getCollisionMass(body, otherBody.get(), contactPoint, normal);
                
                // Calculate position correction magnitude
                double margin = 0.01;
                double correctionMagnitude = (overlap - margin) * collisionMass;
                if (correctionMagnitude < 0.0) {
                    correctionMagnitude *= 0.08 / collision.contactData.size();
                }
                
                // Calculate relative position vectors
                glm::dvec3 rA = contactPoint - body->m_position;
                glm::dvec3 rB = contactPoint - otherBody->m_position;
                
                // Apply position correction
                double scale = 0.2;
                glm::dvec3 correction = normal * correctionMagnitude * scale;
                
                // Apply linear position corrections
                body->m_position -= correction * body->m_invMass;
                
                if (!otherBody->m_isStatic) {
                    otherBody->m_position += correction * otherBody->m_invMass;
                }
                
                // Apply angular position corrections
                glm::dvec3 angularCorrectionA = -body->getWorldInvInertiaTensor() * glm::cross(rA, correction);
                glm::dvec3 angularCorrectionB = +otherBody->getWorldInvInertiaTensor() * glm::cross(rB, correction);
                
                // Convert to quaternion rotations
                double angularCorrectionALengthSq = glm::dot(angularCorrectionA, angularCorrectionA);
                if (angularCorrectionALengthSq > 1e-18) {
                    double angleA = glm::length(angularCorrectionA);
                    glm::dvec3 axisA = angularCorrectionA / angleA;
                    glm::dquat rotationA = glm::angleAxis(angleA, axisA);
                    body->m_orientation = rotationA * body->m_orientation;
                    body->m_orientation = glm::normalize(body->m_orientation);
                    body->invalidateOrientation();
                }
                
                if (!otherBody->m_isStatic) {
                    double angularCorrectionBLengthSq = glm::dot(angularCorrectionB, angularCorrectionB);
                    if (angularCorrectionBLengthSq > 1e-18) {
                        double angleB = glm::sqrt(angularCorrectionBLengthSq);
                        glm::dvec3 axisB = angularCorrectionB / angleB;
                        glm::dquat rotationB = glm::angleAxis(angleB, axisB);
                        otherBody->m_orientation = rotationB * otherBody->m_orientation;
                        otherBody->m_orientation = glm::normalize(otherBody->m_orientation);
                        otherBody->invalidateOrientation();
                    }
                }
                
                // Update collider transforms after position changes
                updateColliderTransform(bodyShared);
                if (!otherBody->m_isStatic) {
                    updateColliderTransform(otherAttachment->rigidBody);
                }
            }
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

bool PhysicsEngine::shouldUseCompliantHandling(RigidBody* bodyA, RigidBody* bodyB, 
                                              const glm::dvec3& contactPoint, const glm::dvec3& normal,
                                              const glm::dvec3& compliantNormal,
                                              double compliantPenetration,
                                              const glm::dvec3* relativeVel) {
    if (compliantPenetration <= 0.0) {
        return false; // No compliant data available
    }
    
    double relativeVelNormal;
    double relativeVelCompliant;
    
    if (relativeVel) {
        // Use provided relative velocity (performance optimization)
        relativeVelNormal = glm::dot(*relativeVel, normal);
        relativeVelCompliant = glm::dot(*relativeVel, compliantNormal);
    } else {
        // Calculate relative velocity at contact point
        glm::dvec3 rA = contactPoint - bodyA->m_position;
        glm::dvec3 rB = contactPoint - bodyB->m_position;
        glm::dvec3 velA = bodyA->m_velocity + glm::cross(bodyA->getAngularVelocityWorld(), rA);
        glm::dvec3 velB = bodyB->m_velocity + glm::cross(bodyB->getAngularVelocityWorld(), rB);
        glm::dvec3 computedRelativeVel = velA - velB;
        relativeVelNormal = glm::dot(computedRelativeVel, normal);
        relativeVelCompliant = glm::dot(computedRelativeVel, compliantNormal);
    }
    
    // Check existing condition
    bool basicCondition = glm::min(relativeVelNormal * 10.0, 0.1) > compliantPenetration;
    
    // Check new condition: compliant relative velocity must be smaller than normal relative velocity
    bool velocityCondition = std::abs(relativeVelCompliant) < std::abs(relativeVelNormal);
    
    return basicCondition && velocityCondition;
}