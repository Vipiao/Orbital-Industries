// CollisionDetector.h
#pragma once

#include "Collider.h"
#include "Edge.h"
#include "CollisionResult.h"
#include <vector>
#include <set>
#include <unordered_map>
#include <memory>
#include <chrono>
#include "../utils/Generator.h"

// Forward declarations for factory methods
class GridCollider;
class PolyhedronCollider;
class BallCollider;
class CubeCollider;
class SensorCollider;

// Forward declaration
class TimeHandler;
class JobManager;

// Custom comparator for deterministic collision pair ordering
struct ColliderPairComparator {
    bool operator()(const std::pair<Collider*, Collider*>& a, const std::pair<Collider*, Collider*>& b) const {
        if (a.first->m_id != b.first->m_id) {
            return a.first->m_id < b.first->m_id;
        }
        return a.second->m_id < b.second->m_id;
    }
};

class CollisionDetector {
public:
    CollisionDetector(TimeHandler* timeHandler);
    ~CollisionDetector() = default;
    
    // Factory methods - create and take ownership of colliders
    std::weak_ptr<GridCollider> addGridCollider(
        const glm::dvec3& position,
        const glm::dquat& orientation,
        JobManager* jobManager,
        TimeHandler* timeHandler);
    
    std::weak_ptr<PolyhedronCollider> addPolyhedronCollider(
        const glm::dvec3& position,
        const glm::dquat& orientation,
        const std::vector<glm::dvec3>& localVertices,
        const std::vector<glm::dvec3>& localFaceAxes,
        const std::vector<glm::dvec3>& localEdgeAxes,
        const std::vector<std::array<int, 2>>& localEdges);
    
    std::weak_ptr<BallCollider> addBallCollider(
        const glm::dvec3& position,
        double radius);
    
    std::weak_ptr<CubeCollider> addCubeCollider(
        const glm::dvec3& position,
        const glm::dquat& orientation,
        double halfWidth);

    std::weak_ptr<SensorCollider> addSensorCollider(
        const glm::dvec3& position,
        const glm::dvec3& halfScale);
    
    // Remove a collider from the collision detection system (pass back what factory gave you)
    void removeCollider(std::weak_ptr<Collider> colliderWeak);
    
    // Run collision detection
    Generator<bool> run();

    // Set the end time for the current collision detection run
    void setEndTime(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Set the current timestep for AABB validity tracking
    void setTimestep(uint64_t timestep);

    // Query collision results from the last detection pass
    const std::vector<CollisionData>& getCollisions(const Collider* collider) const;
    
private:
    // Helper for factory methods
    void registerCollider(std::shared_ptr<Collider> collider);
    
    std::vector<std::shared_ptr<Collider>> m_colliders;
    std::vector<std::unique_ptr<Edge>> edgesX;
    std::vector<std::unique_ptr<Edge>> edgesY;
    std::vector<std::unique_ptr<Edge>> edgesZ;

    // Active collision pairs
    std::set<std::pair<Collider*, Collider*>, ColliderPairComparator> m_activeAABBS;

    TimeHandler* m_timeHandler;
    uint64_t m_currentTimestep = 0;
    // Until what time  are you allowed to run run(...) generator.
    std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime;

    std::unordered_map<int, std::vector<CollisionData>> m_byCollider; // keyed by Collider::m_id

    // Helper functions
    void updateAllCollidersAndAABB();
    void updateAllEdgeValues();
    void sortAndDetectPotentialCollisions(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions);
    void checkCollision(Collider* collider1, Collider* collider2);
    void insertionSort(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions);
    std::pair<Collider*, Collider*> makePair(Collider* a, Collider* b);
};