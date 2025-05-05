// PhysicsEngine.cpp
#include "PhysicsEngine.h"
#include <iostream>
#include <algorithm>
#include <glm/gtx/quaternion.hpp>

PhysicsEngine::PhysicsEngine() 
    : m_nextBodyId{0}
{
}

PhysicsEngine::~PhysicsEngine() {
    // Vector of unique_ptr will handle cleanup automatically
}

int PhysicsEngine::addRigidBody(const glm::dvec3& position, 
                               const glm::dquat& orientation,
                               double mass, 
                               double momentOfInertia,
                               bool isStatic) {
    int id = m_nextBodyId++;
    
    auto body = std::make_unique<RigidBody>();
    body->id = id;
    body->position = position;
    body->velocity = glm::dvec3{0.0, 0.0, 0.0};
    body->forces = glm::dvec3{0.0, 0.0, 0.0};
    
    body->orientation = glm::normalize(orientation); // Ensure unit quaternion
    body->angularVelocity = glm::dvec3{0.0, 0.0, 0.0};
    body->torques = glm::dvec3{0.0, 0.0, 0.0};
    
    body->mass = mass;
    body->momentOfInertia = momentOfInertia;
    body->isStatic = isStatic;
    
    m_rigidBodies.push_back(std::move(body));
    
    return id;
}

void PhysicsEngine::removeRigidBody(int id) {
    auto it = std::remove_if(m_rigidBodies.begin(), m_rigidBodies.end(),
        [id](const std::unique_ptr<RigidBody>& body) {
            return body->id == id;
        });
    
    if (it != m_rigidBodies.end()) {
        m_rigidBodies.erase(it, m_rigidBodies.end());
    }
}

PhysicsEngine::RigidBody* PhysicsEngine::getRigidBody(int id) {
    auto it = std::find_if(m_rigidBodies.begin(), m_rigidBodies.end(),
        [id](const std::unique_ptr<RigidBody>& body) {
            return body->id == id;
        });
    
    if (it != m_rigidBodies.end()) {
        return it->get();
    }
    return nullptr;
}

void PhysicsEngine::applyForce(int id, const glm::dvec3& force) {
    RigidBody* body = getRigidBody(id);
    if (body && !body->isStatic) {
        body->forces += force;
    }
}

void PhysicsEngine::applyForceAtPoint(int id, const glm::dvec3& force, const glm::dvec3& point) {
    RigidBody* body = getRigidBody(id);
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

void PhysicsEngine::applyTorque(int id, const glm::dvec3& torque) {
    RigidBody* body = getRigidBody(id);
    if (body && !body->isStatic) {
        body->torques += torque;
    }
}

void PhysicsEngine::setGravity(const glm::dvec3& gravity) {
    m_gravity = gravity;
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
    }
}

void PhysicsEngine::handleCollisions() {
    // Currently empty as per requirements
    // This would detect and resolve collisions between bodies
}