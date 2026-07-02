// Digibot.cpp
#include "Digibot.h"
#include "DigibotPhysics.h"
#include "DigibotGraphics.h"
#include "DigibotAnimation.h"
#include "DigibotAnimationContext.h"
#include "DigibotController.h"
#include "DigibotResources.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/RigidBody.h"
#include "graphics/GraphicsEngine.h"

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

    // Create animation subsystem
    m_digibotAnimation = std::make_unique<DigibotAnimation>();

    // Perform initial transform update
    updateVisualTransform();
}

Digibot::~Digibot() {
    // Subsystems clean up automatically via unique_ptr
}

void Digibot::preRenderCallback(uint64_t frameNum, double timeRemainder) {
    //if (frameNum % 16 != 0)
    //{
    //    return;
    //}
    
    // Assemble animation context
    auto rigidBodyForAnim = m_rigidBody.lock();
    if (rigidBodyForAnim) {
        DigibotAnimationContext animCtx;
        animCtx.m_frameNum                = frameNum;
        // The mesh is rendered with a center-of-rotation of -m_graphicsPosition, so the
        // graphics origin ends up at position + orientation * m_graphicsPosition (see
        // mesh_vertex_shader.vert). The animation's localToWorld/worldToLocal must use the
        // same rotated offset, otherwise feet computed in world space drift by
        // (orientation - identity) * m_graphicsPosition as the surface/grid rotates.
        animCtx.m_digibotWorldPos         = rigidBodyForAnim->m_position
            + rigidBodyForAnim->m_orientation * m_graphicsPosition;
        animCtx.m_digibotWorldOrientation = rigidBodyForAnim->m_orientation;
        animCtx.m_digibotWorldVelocity    = rigidBodyForAnim->m_velocity;
        animCtx.m_deltaTime               = static_cast<double>(m_physics->getPhysicsHz())
            / static_cast<double>(m_graphicsEngine->getFrameRate());

        if (m_digibotController->isJetpackEnabled()) {
            animCtx.m_mode = DigibotAnimationContext::MovementMode::Flying;
        } else {
            animCtx.m_mode             = DigibotAnimationContext::MovementMode::Walking;
            animCtx.m_hasGroundContact = m_digibotController->hasGroundContact();
            animCtx.m_surfacePoint     = m_digibotController->getGroundContactPoint();
            animCtx.m_surfaceNormal    = m_digibotController->getGroundSurfaceNormal();
            animCtx.m_surfaceBody      = m_digibotController->getWalkingTargetRigidBody();
        }

        DigibotPose pose = m_digibotAnimation->update(animCtx);
        m_digibotGraphics->updateBodyPartPositions(pose);
    }

    // Calculate delta time remainder and apply surface rotation to view
    double deltaTimeRemainder = timeRemainder - m_lastTimeRemainder;
    if (deltaTimeRemainder < 0.0) deltaTimeRemainder += 1.0; // Handle wraparound
    m_lastTimeRemainder = timeRemainder;

    m_digibotController->updatePerFrame(deltaTimeRemainder);
}

void Digibot::onPhysicsUpdateComplete() {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
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
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        return;
    }
    
    // Calculate angular velocity components
    glm::dvec3 angVelAxis = rigidBody->getAngularVelocityWorld();
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }

    uint64_t currentPhysicsTimeStep = m_physics->getCurrentPhysicsTimeStep();

    // Calculate mesh position (rigid body position offset by center of mass)
    //glm::dvec3 meshPosition = rigidBody->m_position - m_centerOfMass;
    glm::dvec3 meshPosition = rigidBody->m_position + m_graphicsPosition;

    // Update graphics subsystem with current transform
    m_digibotGraphics->updateWorldTransform(
        meshPosition,
        rigidBody->m_velocity,
        rigidBody->m_orientation,
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
    m_digibotController->setViewDirection(direction);
}

glm::dvec3 Digibot::getViewDirection() const {
    return m_digibotController->getViewDirection();
}

void Digibot::setRollInput(int rollInput) {
    m_digibotController->setRollInput(rollInput);
}