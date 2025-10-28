// Digibot.cpp
#include "Digibot.h"
#include "DigibotPhysics.h"
#include "DigibotGraphics.h"
#include "DigibotController.h"
#include "DigibotResources.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include "../../graphics/GraphicsEngine.h"

Digibot::Digibot(PhysicsEngine* physics, GraphicsEngine* graphics,
                   JobManager* jobManager, TimeHandler* timeHandler,
                   DigibotResources* resources, GridSubsystem* gridSubsystem)
    : Character(physics, graphics, jobManager, timeHandler)
    , m_graphicsEngine(graphics)
{
    // Create physics subsystem
    m_digibotPhysics = std::make_unique<DigibotPhysics>(physics, jobManager, timeHandler);

    // Set Character base class members
    m_rigidBody = m_digibotPhysics->getRigidBody();
    //m_centerOfMass = m_digibotPhysics->getCenterOfMass();

    // Create controller subsystem
    m_digibotController = std::make_unique<DigibotController>(
        m_digibotPhysics.get(), physics, gridSubsystem);

    // Create graphics subsystem
    m_digibotGraphics = std::make_unique<DigibotGraphics>(graphics, resources);

    // Perform initial transform update
    updateVisualTransform();
}

Digibot::~Digibot() {
    // Subsystems clean up automatically via unique_ptr
}

void Digibot::preRenderCallback(uint64_t frameNum) {
    //if (frameNum % 16 != 0)
    //{
    //    return;
    //}
    
    // Update body part articulation every frame
    DigibotTargetPose targetPose;
    
    targetPose.rightHand.position = glm::dvec3(0.6, 0.6, 1.4);
    double time = (double)frameNum * 0.02;
    double scale = glm::cos(time) * 0.2 + 0.8;
    targetPose.rightHand.position.x *= scale;
    targetPose.rightHand.position.y *= scale;

    targetPose.leftHand = targetPose.rightHand;
    targetPose.leftHand.position.x = -targetPose.leftHand.position.x;

    // Set foot targets (for now, just at natural positions)
    double offsetY = 0.4 * glm::cos(time);
    double offsetZ = 0.16 * glm::sin(time);
    targetPose.rightFoot.position =
        glm::dvec3(0.179225, 0.051327, 0.059608) +
        glm::dvec3(0, offsetY, glm::max(0., -offsetZ));
    targetPose.leftFoot.position =
        glm::dvec3(-0.179225, 0.051327, 0.059608) +
        glm::dvec3(0, -offsetY, glm::max(0., offsetZ));

    targetPose.leftFoot.position.x += glm::cos(time / 1.17) * 0.08 - 0.1;
    targetPose.leftFoot.position.y += glm::cos(time / 1.17) * 0.08;

    targetPose.rightFoot.position.x -= glm::cos(time / 1.17) * 0.08 + 0.1;
    targetPose.rightFoot.position.y -= glm::cos(time / 1.17) * 0.08;

    targetPose.headOrientation = glm::angleAxis(
        glm::radians(glm::cos(time/1.53) * 16.),
        glm::normalize(glm::dvec3{0,1,1})
    );

    m_digibotGraphics->updateBodyPartPositions(targetPose);
}

void Digibot::onPhysicsUpdateComplete() {
    if (!m_rigidBody) {
        return;
    }

    // Update physics subsystem
    m_digibotPhysics->updatePhysics();

    // Update visual model transform
    updateVisualTransform();

    // Update collision box if visible
    if (m_digibotPhysics->isCollisionBoxVisible()) {
        uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();
        m_digibotPhysics->updateCollisionBoxTransform(m_graphicsEngine, currentPhysicsTimeStep);
    }

    // Run controller physics
    m_digibotController->physics();
}

void Digibot::showCollisionBox() {
    m_digibotPhysics->showCollisionBox(m_graphicsEngine);
}

void Digibot::hideCollisionBox() {
    m_digibotPhysics->hideCollisionBox(m_graphicsEngine);
}

bool Digibot::isCollisionBoxVisible() const {
    return m_digibotPhysics->isCollisionBoxVisible();
}

void Digibot::updateVisualTransform() {
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
    //glm::dvec3 meshPosition = m_rigidBody->m_position - m_centerOfMass;
    glm::dvec3 meshPosition = m_rigidBody->m_position + m_graphicsPosition;

    // Update graphics subsystem with current transform
    m_digibotGraphics->updateWorldTransform(
        meshPosition,
        m_rigidBody->m_velocity,
        m_rigidBody->m_orientation,
        angVelAxis,
        angVelMagnitude,
        -m_graphicsPosition,
        currentPhysicsTimeStep
    );
}

glm::dvec3 Digibot::worldToLocal(const glm::dvec3& worldPos) const {
    return m_digibotPhysics->worldToLocal(worldPos);
}

glm::dvec3 Digibot::localToWorld(const glm::dvec3& localPos) const {
    return m_digibotPhysics->localToWorld(localPos);
}

void Digibot::setMovementDirection(const glm::ivec3& direction) {
    m_digibotController->setMovementDirection(direction);
}

void Digibot::setViewDirection(const glm::dvec3& direction) {
    m_viewDirection = glm::normalize(direction);
    m_digibotController->setViewDirection(m_viewDirection);
}

glm::dvec3 Digibot::getViewDirection() const {
    return m_viewDirection;
}

void Digibot::setRollInput(int rollInput) {
    m_digibotController->setRollInput(rollInput);
}