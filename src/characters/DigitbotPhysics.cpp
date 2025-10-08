// DigitbotPhysics.cpp
#include "DigitbotPhysics.h"
#include "../physics/PhysicsEngine.h"
#include "../physics/GridCollider.h"
#include "../physics/RigidBody.h"
#include "../graphics/GraphicsEngine.h"
#include "../utils/PolyhedronProcessor.h"
#include "../utils/MassInertiaCalculator.h"
#include "../utils/GridGeometry.h"
#include "../graphics/meshHandler/MeshHandler.h"

DigitbotPhysics::DigitbotPhysics(PhysicsEngine* physics, JobManager* jobManager, TimeHandler* timeHandler)
    : m_physics(physics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
    , m_rigidBody(nullptr)
    , m_centerOfMass(0.0, 0.0, 0.0)
    , m_collisionBoxMeshId(-1)
{
    // 1. Create GridCollider at origin
    m_colliderWeak = m_physics->getCollisionDetector().addGridCollider(
        glm::dvec3(0.0, 0.0, 0.0),
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        m_jobManager,
        m_timeHandler
    );

    auto collider = m_colliderWeak.lock();
    if (!collider) {
        throw std::runtime_error("DigitbotPhysics: Failed to lock GridCollider immediately after creation");
    }

    // Add two cube cells: lower body at {0,0,0}, upper body at {0,0,1}
    collider->addCubeCell(glm::ivec3(0, 0, 0), 1.0);
    collider->addCubeCell(glm::ivec3(0, 0, 1), 1.0);

    // 2. Calculate mass properties
    std::vector<glm::ivec3> coords = {glm::ivec3(0, 0, 0), glm::ivec3(0, 0, 1)};
    double totalMass = 0.0;
    glm::dvec3 centerOfMass(0.0);
    glm::dmat3 inertiaTensor(0.0);

    auto getProperties = [](const glm::ivec3& coord) -> std::tuple<double, glm::dvec3, glm::dmat3> {
        // Standard cube vertices for mass calculation
        std::vector<glm::ivec3> cubeVertices = {
            {0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0},
            {0, 0, 4}, {4, 0, 4}, {4, 4, 4}, {0, 4, 4}
        };
        auto props = MassInertiaCalculator::calculatePolyhedronMassProperties(
            cubeVertices, 4, 1.0);  // maxSize=4, density=1.0
        return {props.mass, props.centerOfMass, props.inertiaTensor};
    };

    MassInertiaCalculator::calculateInertiaForCoords(
        coords, getProperties, &totalMass, &centerOfMass, &inertiaTensor);

    // Double mass and inertia as requested
    totalMass *= 2.0;
    inertiaTensor *= 2.0;

    m_centerOfMass = centerOfMass;

    // 3. Create RigidBody at origin
    m_rigidBody = m_physics->addRigidBody(
        glm::dvec3(0.0, 0.0, 0.0),
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        totalMass,
        inertiaTensor,
        false  // Not static, should fall
    );

    m_rigidBody->m_colliderOffset = m_centerOfMass;

    // 4. Connect collider to rigid body
    m_physics->connectCollider(m_rigidBody, m_colliderWeak);
    m_physics->updateColliderTransform(m_rigidBody);
}

DigitbotPhysics::~DigitbotPhysics() {
    // Disconnect and cleanup physics
    if (m_rigidBody) {
        m_physics->disconnectCollider(m_rigidBody);
        m_physics->removeRigidBody(m_rigidBody);
    }

    // Remove collider
    m_physics->getCollisionDetector().removeCollider(m_colliderWeak);
}

void DigitbotPhysics::showCollisionBox(GraphicsEngine* graphics) {
    if (m_collisionBoxMeshId == -1) {
        m_collisionBoxMeshId = graphics->createMesh();
    }
    createCollisionBoxMesh(graphics);
}

void DigitbotPhysics::hideCollisionBox(GraphicsEngine* graphics) {
    if (m_collisionBoxMeshId != -1) {
        graphics->removeMesh(m_collisionBoxMeshId);
        m_collisionBoxMeshId = -1;
    }
}

void DigitbotPhysics::createCollisionBoxMesh(GraphicsEngine* graphics) {
    auto collider = m_colliderWeak.lock();
    if (!collider) {
        return;
    }

    glm::dvec4 collisionColor(0.2, 1.0, 0.2, 0.4);  // Semi-transparent green
    
    // Remove and recreate mesh
    if (m_collisionBoxMeshId != -1) {
        graphics->removeMesh(m_collisionBoxMeshId);
        m_collisionBoxMeshId = graphics->createMesh();
    }

    // Iterate through all cells in the GridCollider
    for (const auto& [coord, polyhedron] : collider->getCells()) {
        // Generate cube vertices
        std::vector<glm::dvec3> vertices = PolyhedronProcessor::generateCubeVertices(1.0);
        
        // Translate to cell position
        glm::dvec3 cellPos = glm::dvec3(coord.x, coord.y, coord.z);
        for (glm::dvec3& v : vertices) {
            v += cellPos;
        }

        // Convert to integer vertices for triangulation
        std::vector<glm::ivec3> verticesInt(8);
        for (int i = 0; i < 8; ++i) {
            verticesInt[i] = glm::ivec3(
                (vertices[i].x + 0.5) * 4,
                (vertices[i].y + 0.5) * 4,
                (vertices[i].z + 0.5) * 4
            );
        }

        auto triangles = PolyhedronProcessor::getTriangles(verticesInt, 4);
        auto meshData = PolyhedronProcessor::generateMeshData(triangles);

        // Create color vector
        std::vector<glm::dvec4> colors(meshData.positions.size(), collisionColor);

        // Append this cell's mesh
        graphics->m_meshHandler->appendTrianglesToMesh(
            m_collisionBoxMeshId, &meshData.positions, &meshData.normals, 
            &meshData.tangents, &meshData.uvs, nullptr, &colors);
    }

    // Set initial transform
    updateCollisionBoxTransform(graphics, m_physics->getCurrentPhysicsTimeStep());
}

void DigitbotPhysics::updateCollisionBoxTransform(GraphicsEngine* graphics, uint64_t currentPhysicsTimeStep) {
    if (m_collisionBoxMeshId == -1 || !m_rigidBody) {
        return;
    }

    // Update collision box mesh transform
    glm::dvec3 angVelAxis = m_rigidBody->getAngularVelocityWorld();
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }

    glm::dvec3 meshPosition = m_rigidBody->m_position - m_centerOfMass;

    graphics->updateMeshTransform(
        m_collisionBoxMeshId,
        meshPosition,
        m_rigidBody->m_velocity,
        m_rigidBody->m_orientation,
        angVelAxis,
        angVelMagnitude,
        m_centerOfMass,
        glm::dvec3(1.0, 1.0, 1.0),
        -1, -1, -1,
        currentPhysicsTimeStep,
        0.0
    );
}

glm::dvec3 DigitbotPhysics::worldToLocal(const glm::dvec3& worldPos) const {
    return GridGeometry::worldToGrid(
        worldPos,
        m_rigidBody->m_position,
        m_rigidBody->m_orientation,
        m_centerOfMass
    );
}

glm::dvec3 DigitbotPhysics::localToWorld(const glm::dvec3& localPos) const {
    return GridGeometry::gridToWorld(
        localPos,
        m_rigidBody->m_position,
        m_rigidBody->m_orientation,
        m_centerOfMass
    );
}