// DigibotController.cpp
#include "DigibotController.h"
#include "DigibotPhysics.h"
#include "../physics/RigidBody.h"
#include "../physics/PhysicsEngine.h"

DigibotController::DigibotController(DigibotPhysics* physics, PhysicsEngine* physicsEngine)
    : m_physics(physics)
    , m_physicsEngine(physicsEngine)
    , m_movementDirection(0, 0, 0)
    , m_thrustStrength(0.01) // Default thrust strength
{
    if (!m_physics) {
        throw std::runtime_error("DigibotController: Physics component cannot be null");
    }
    
    if (!m_physicsEngine) {
        throw std::runtime_error("DigibotController: Physics engine cannot be null");
    }
}

void DigibotController::setViewDirection(const glm::dvec3& viewDirection) {
    // Normalize to ensure it's a unit direction vector
    m_viewDirection = glm::normalize(viewDirection);
}

void DigibotController::setMovementDirection(const glm::ivec3& direction) {
    m_movementDirection = direction;
}

void DigibotController::physics() {
    // Get the rigid body from physics component
    RigidBody* rigidBody = m_physics->getRigidBody();
    if (!rigidBody || rigidBody->m_mass <= 0.0) {
        return;
    }
    
    // Skip if no movement requested
    if (m_movementDirection == glm::ivec3(0, 0, 0)) {
        return;
    }
    
    // Convert integer direction to normalized 3D vector
    glm::dvec3 direction = glm::dvec3(
        static_cast<double>(m_movementDirection.x),
        static_cast<double>(m_movementDirection.y),
        static_cast<double>(m_movementDirection.z)
    );
    
    // Normalize if not zero
    if (glm::length(direction) > 0.0) {
        direction = glm::normalize(direction);

        // Transform direction from local to world space using rigid body orientation
        direction = rigidBody->m_orientation * direction;
    } else {
        return; // Skip if zero length
    }
    
    // Calculate thrust force based on mass
    double forceMagnitude = m_thrustStrength * rigidBody->m_mass;
    glm::dvec3 thrustForce = direction * forceMagnitude;
    
    // Apply force at center of mass
    m_physicsEngine->applyForce(rigidBody, thrustForce);
}

void DigibotController::setThrustStrength(double strength) {
    m_thrustStrength = strength;
}