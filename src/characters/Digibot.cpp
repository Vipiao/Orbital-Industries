// Digitbot.cpp
#include "Digitbot.h"
#include "DigitbotPhysics.h"
#include "DigitbotGraphics.h"
#include "DigitbotResources.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/RigidBody.h"
#include "../graphics/GraphicsEngine.h"

Digitbot::Digitbot(PhysicsEngine* physics, GraphicsEngine* graphics,
                   JobManager* jobManager, TimeHandler* timeHandler,
                   DigitbotResources* resources)
    : Character(physics, graphics, jobManager, timeHandler)
    , m_graphicsEngine(graphics)
{
    // Create physics subsystem
    m_digitbotPhysics = std::make_unique<DigitbotPhysics>(physics, jobManager, timeHandler);

    // Set Character base class members
    m_rigidBody = m_digitbotPhysics->getRigidBody();
    m_centerOfMass = m_digitbotPhysics->getCenterOfMass();

    // Create graphics subsystem
    m_digitbotGraphics = std::make_unique<DigitbotGraphics>(graphics, resources);
 
    // Perform initial transform update
    updateVisualTransform();
}

Digitbot::~Digitbot() {
    // Subsystems clean up automatically via unique_ptr
}

void Digitbot::preRenderCallback(uint64_t frameNum) {
    // Future: animation updates, visual effects
}

void Digitbot::onPhysicsUpdateComplete() {
    if (!m_rigidBody) {
        return;
    }

    // Update visual model transform
    updateVisualTransform();

    // Update collision box if visible
    if (m_digitbotPhysics->isCollisionBoxVisible()) {
        uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();
        m_digitbotPhysics->updateCollisionBoxTransform(m_graphicsEngine, currentPhysicsTimeStep);
    }
}

void Digitbot::showCollisionBox() {
    m_digitbotPhysics->showCollisionBox(m_graphicsEngine);
}

void Digitbot::hideCollisionBox() {
    m_digitbotPhysics->hideCollisionBox(m_graphicsEngine);
}

bool Digitbot::isCollisionBoxVisible() const {
    return m_digitbotPhysics->isCollisionBoxVisible();
}

void Digitbot::updateVisualTransform() {
    if (!m_rigidBody) {
        return;
    }
    
    // Calculate angular velocity components
    glm::dvec3 angVelAxis = m_rigidBody->getAngularVelocityWorld();
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }

    uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();

    // Calculate mesh position (rigid body position offset by center of mass)
    glm::dvec3 meshPosition = m_rigidBody->m_position - m_centerOfMass;

    // Update graphics subsystem with current transform
    m_digitbotGraphics->updateWorldTransform(
        meshPosition,
        m_rigidBody->m_velocity,
        m_rigidBody->m_orientation,
        angVelAxis,
        angVelMagnitude,
        m_centerOfMass,
        currentPhysicsTimeStep
    );
}

glm::dvec3 Digitbot::worldToLocal(const glm::dvec3& worldPos) const {
    return m_digitbotPhysics->worldToLocal(worldPos);
}

glm::dvec3 Digitbot::localToWorld(const glm::dvec3& localPos) const {
    return m_digitbotPhysics->localToWorld(localPos);
}