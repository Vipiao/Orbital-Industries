// DigibotPhysics.cpp
#include "DigibotPhysics.h"
#include "../../physics/PhysicsEngine.h"
#include "../../physics/GridCollider.h"
#include "../../physics/RigidBody.h"
#include "../../physics/PolyhedronCollider.h"
#include "graphics/GraphicsEngine.h"
#include "../../utils/PolyhedronProcessor.h"
#include "../../utils/MassInertiaCalculator.h"
#include "../../utils/GridGeometry.h"
#include "graphics/meshHandler/MeshHandler.h"
#include "../../physics/SensorCollider.h"
#include "../../physics/BallCollider.h"
#include <cassert>

DigibotPhysics::DigibotPhysics(PhysicsEngine* physics, JobManager* jobManager, TimeHandler* timeHandler)
    : m_physics(physics)
    , m_jobManager(jobManager)
    , m_timeHandler(timeHandler)
    , m_colliderLocalPosition(-0.5, -0.5, -1.0)
    //, m_centerOfMass(0.0, 0.0, 0.0)
    , m_collisionBoxMeshId(-1)
    , m_walkingSensorRadius(2.0)
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
        throw std::runtime_error("DigibotPhysics: Failed to lock GridCollider immediately after creation");
    }

    // Start with standard cube vertices for the base shape
    std::vector<glm::dvec3> baseVertices = PolyhedronProcessor::generateCubeVertices(1.0);
    
    // Define fixed point value for integer calculations
    const int fixedPointValue = 10;
    
    // Convert base vertices to integer space
    std::vector<glm::ivec3> lowerIntVertices, upperIntVertices;
    for (const auto& v : baseVertices) {
        lowerIntVertices.push_back(glm::ivec3(v * double(fixedPointValue)));
    }
    upperIntVertices = lowerIntVertices; // Start with same vertices for upper body
    
    // Define vertex offsets as {vertex, x_offset, y_offset, z_offset}
    struct VertexOffset {
        PolyhedronProcessor::CubeVertex vertex;
        int x, y, z;
    };
    
    // TODOO:Hardcode collision box shape. Should probably not do it like this.
    // Make it more general after adding other characters. Will become more
    // clear at that point what would be a good interface for reading this info
    // Define offsets for lower body (combined for each vertex)
    const VertexOffset lowerOffsets[] = {
        // Bottom vertices
        {PolyhedronProcessor::BOTTOM_BACK_LEFT,    2, 0, 5},
        {PolyhedronProcessor::BOTTOM_BACK_RIGHT,  -2, 0, 5},
        {PolyhedronProcessor::BOTTOM_FRONT_RIGHT, -2, -3, 5},
        {PolyhedronProcessor::BOTTOM_FRONT_LEFT,   2, -3, 5},
        
        // Top vertices
        {PolyhedronProcessor::TOP_BACK_LEFT,       2, 0, 0},
        {PolyhedronProcessor::TOP_BACK_RIGHT,     -2, 0, 0},
        {PolyhedronProcessor::TOP_FRONT_RIGHT,    -2, -2, 0},
        {PolyhedronProcessor::TOP_FRONT_LEFT,      2, -2, 0}
    };
    
    // Define offsets for upper body (combined for each vertex)
    const VertexOffset upperOffsets[] = {
        // Bottom vertices
        {PolyhedronProcessor::BOTTOM_BACK_LEFT,    2, 0, 0},
        {PolyhedronProcessor::BOTTOM_BACK_RIGHT,  -2, 0, 0},
        {PolyhedronProcessor::BOTTOM_FRONT_RIGHT, -2, -2, 0},
        {PolyhedronProcessor::BOTTOM_FRONT_LEFT,   2, -2, 0},
        
        // Top vertices
        {PolyhedronProcessor::TOP_BACK_LEFT,       2, 0, -3},
        {PolyhedronProcessor::TOP_BACK_RIGHT,     -2, 0, -3},
        {PolyhedronProcessor::TOP_FRONT_RIGHT,    -2, -2, -3},
        {PolyhedronProcessor::TOP_FRONT_LEFT,      2, -2, -3}
    };
    
    // Apply offsets to lower body
    for (const auto& offset : lowerOffsets) {
        lowerIntVertices[offset.vertex].x += offset.x;
        lowerIntVertices[offset.vertex].y += offset.y;
        lowerIntVertices[offset.vertex].z += offset.z;
    }
    
    // Apply offsets to upper body
    for (const auto& offset : upperOffsets) {
        upperIntVertices[offset.vertex].x += offset.x;
        upperIntVertices[offset.vertex].y += offset.y;
        upperIntVertices[offset.vertex].z += offset.z;
    }
    
    // Convert modified integer vertices back to float space for the collider
    std::vector<glm::dvec3> lowerBodyVertices, upperBodyVertices;
    for (const auto& v : lowerIntVertices) {
        lowerBodyVertices.push_back(glm::dvec3(v) / double(fixedPointValue));
    }
    for (const auto& v : upperIntVertices) {
        upperBodyVertices.push_back(glm::dvec3(v) / double(fixedPointValue));
    }
    
    // Calculate axes for each shape
    auto lowerAxes = PolyhedronProcessor::getAxis(lowerIntVertices, fixedPointValue);
    auto upperAxes = PolyhedronProcessor::getAxis(upperIntVertices, fixedPointValue);
    
    // Add polyhedron cells
    collider->addPolyhedronCell(glm::ivec3(0, 0, 0),
                               lowerBodyVertices,
                               lowerAxes.faceAxis, 
                               lowerAxes.edgeAxis,
                               lowerAxes.edges);
    collider->addPolyhedronCell(glm::ivec3(0, 0, 1),
                               upperBodyVertices,
                               upperAxes.faceAxis,
                               upperAxes.edgeAxis,
                               upperAxes.edges);

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

    // Double mass and inertia
    assert(totalMass > 0.0 && "computed collision-box mass must be positive before scaling");
    double scalingFactor = 80. / totalMass;
    totalMass = 80.;
    inertiaTensor *= scalingFactor;

    //m_centerOfMass = centerOfMass;
    //m_centerOfMass = {0.5,0.5,4};

    // 3. Create RigidBody at origin
    m_rigidBody = m_physics->addRigidBody(
        glm::dvec3(0.0, 0.0, 0.0),
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        totalMass,
        inertiaTensor,
        false  // Not static, should fall
    );

    // 4. Attach collider to rigid body
    // Collider local position relative to center of mass (negative of old offset)
    glm::dquat colliderLocalOrientation(1.0, 0.0, 0.0, 0.0);
    m_physics->attachCollider(
        m_rigidBody,
        m_colliderWeak,
        m_colliderLocalPosition,
        glm::dquat(1.0, 0.0, 0.0, 0.0),  // Identity orientation
        false);                           // Not a trigger

    // 5. Create walking sensor for ground detection
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("DigibotPhysics: Failed to lock RigidBody immediately after creation");
    }
    m_walkingSensor = m_physics->getCollisionDetector().addBallCollider(
        rigidBody->m_position,
        m_walkingSensorRadius);

    // Attach sensor as a trigger (detects but doesn't respond physically)
    m_physics->attachCollider(
        m_rigidBody,
        m_walkingSensor,
        glm::dvec3(0.0),                 // At center of mass
        glm::dquat(1.0, 0.0, 0.0, 0.0),  // Identity orientation
        true);                            // IS a trigger
}

DigibotPhysics::~DigibotPhysics() {
    if (m_collisionBoxMeshId != -1 && m_graphics) {
        m_graphics->removeMesh(m_collisionBoxMeshId);
        m_graphics->m_ssboManager->deallocateIndex(m_collisionBoxMeshId);
        m_collisionBoxMeshId = -1;
    }

    if (!m_rigidBody.expired()) {
        m_physics->removeRigidBody(m_rigidBody);
    }

    m_physics->getCollisionDetector().removeCollider(m_colliderWeak);

    if (!m_walkingSensor.expired()) {
        m_physics->getCollisionDetector().removeCollider(m_walkingSensor);
    }
}

void DigibotPhysics::showCollisionBox(GraphicsEngine* graphics) {
    m_graphics = graphics;
    if (m_collisionBoxMeshId == -1) {
        m_collisionBoxMeshId = graphics->m_ssboManager->allocateIndex();
        graphics->createMesh(m_collisionBoxMeshId);
    }
    createCollisionBoxMesh(graphics);
}

void DigibotPhysics::hideCollisionBox(GraphicsEngine* graphics) {
    if (m_collisionBoxMeshId != -1) {
        graphics->removeMesh(m_collisionBoxMeshId);
        graphics->m_ssboManager->deallocateIndex(m_collisionBoxMeshId);
        m_collisionBoxMeshId = -1;
    }
}

void DigibotPhysics::createCollisionBoxMesh(GraphicsEngine* graphics) {
    auto collider = m_colliderWeak.lock();
    if (!collider) {
        return;
    }

    glm::dvec4 collisionColor(0.2, 1.0, 0.2, 0.4);  // Semi-transparent green
    
    // Remove and recreate mesh (reuse same SSBO index — caller owns it)
    if (m_collisionBoxMeshId != -1) {
        graphics->removeMesh(m_collisionBoxMeshId);
        graphics->createMesh(m_collisionBoxMeshId);
    }

    // Iterate through all cells in the GridCollider
    for (const auto& [coord, polyhedron] : collider->getCells()) {
        // Get the actual vertices from the polyhedron
        PolyhedronCollider* polyCollider = static_cast<PolyhedronCollider*>(polyhedron.get());
        std::vector<glm::dvec3> vertices = polyCollider->getLocalVertices();

        // Apply cell position offset
        glm::dvec3 cellPos = glm::dvec3(coord.x, coord.y, coord.z);
        for (glm::dvec3& v : vertices) {
            // Add cell position offset
            v += cellPos + 0.5;
        }
        
        // Generate triangle indices from the actual vertices
        auto triangleIndices = PolyhedronProcessor::getTriangleIndices(vertices);
        
        // Create triangles from vertices and indices
        std::vector<std::array<glm::dvec3, 3>> triangles;
        for (const auto& indices : triangleIndices) {
            triangles.push_back({vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]});
        }

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

void DigibotPhysics::updateCollisionBoxTransform(GraphicsEngine* graphics, uint64_t currentPhysicsTimeStep) {
    auto rigidBody = m_rigidBody.lock();
    if (m_collisionBoxMeshId == -1 || !rigidBody) {
        return;
    }

    // Update collision box mesh transform
    glm::dvec3 angVelAxis = rigidBody->getAngularVelocityWorld();
    double angVelMagnitude = glm::length(angVelAxis);
    if (angVelMagnitude > 0.00001) {
        angVelAxis = angVelAxis / angVelMagnitude;
    } else {
        angVelAxis = glm::dvec3(0.0, 0.0, 1.0);
        angVelMagnitude = 0.0;
    }
    
    glm::dvec3 meshPosition = rigidBody->m_position + rigidBody->m_orientation * m_colliderLocalPosition;
    graphics->updateMeshTransform(
        m_collisionBoxMeshId,
        meshPosition,
        rigidBody->m_velocity,
        rigidBody->m_orientation,
        angVelAxis,
        angVelMagnitude,
        -m_colliderLocalPosition,  // Pass negative to maintain old offset semantics for graphics
        glm::dvec3(1.0, 1.0, 1.0),
        -1, -1, -1,
        currentPhysicsTimeStep,
        0.0
    );
}

glm::dvec3 DigibotPhysics::worldToLocal(const glm::dvec3& worldPos) const {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("DigibotPhysics::worldToLocal: RigidBody has been destroyed");
    }
    return GridGeometry::worldToGrid(
        worldPos,
        rigidBody->m_position,
        rigidBody->m_orientation,
        -m_colliderLocalPosition
    );
}

glm::dvec3 DigibotPhysics::localToWorld(const glm::dvec3& localPos) const {
    auto rigidBody = m_rigidBody.lock();
    if (!rigidBody) {
        throw std::runtime_error("DigibotPhysics::localToWorld: RigidBody has been destroyed");
    }
    return GridGeometry::gridToWorld(
        localPos,
        rigidBody->m_position,
        rigidBody->m_orientation,
        -m_colliderLocalPosition
    );
}

void DigibotPhysics::updatePhysics() {
    if (m_rigidBody.expired()) {
        return;
    }
}