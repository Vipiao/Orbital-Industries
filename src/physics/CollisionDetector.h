// CollisionDetector.h
#pragma once

#include "Collider.h"
#include "Edge.h"
#include "CollisionResult.h"
#include <vector>
#include <set>
#include <memory>
#include <chrono>
#include "../utils/Generator.h"

// Forward declarations for factory methods
class GridCollider;
class PolyhedronCollider;
class BallCollider;
class CubeCollider;

// Forward declaration
class TimeHandler;
class JobManager;

// Custom comparator for deterministic collision pair ordering
struct ColliderPairComparator {
    bool operator()(const std::pair<Collider*, Collider*>& a, const std::pair<Collider*, Collider*>& b) const {
        if (a.first->m_debugId != b.first->m_debugId) {
            return a.first->m_debugId < b.first->m_debugId;
        }
        return a.second->m_debugId < b.second->m_debugId;
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
        const std::vector<glm::dvec3>& localEdgeAxes);
    
    std::weak_ptr<BallCollider> addBallCollider(
        const glm::dvec3& position,
        double radius);
    
    std::weak_ptr<CubeCollider> addCubeCollider(
        const glm::dvec3& position,
        const glm::dquat& orientation,
        double halfWidth);
    
    // Remove a collider from the collision detection system (pass back what factory gave you)
    void removeCollider(std::weak_ptr<Collider> colliderWeak);
    
    // Run collision detection
    Generator<bool> run(std::vector<CollisionResult>& collisions);
    
    // Set the end time for the current collision detection run
    void setEndTime(std::chrono::time_point<std::chrono::high_resolution_clock> endTime);

    // Set the current timestep for AABB validity tracking
    void setTimestep(uint64_t timestep);
    
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
    std::chrono::time_point<std::chrono::high_resolution_clock> m_endTime;

    // Helper functions
    void updateAllCollidersAndAABB();
    void updateAllEdgeValues();
    void sortAndDetectPotentialCollisions(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions);
    void checkCollision(Collider* collider1, Collider* collider2, std::vector<CollisionResult>& collisions);
    void insertionSort(
        std::vector<std::unique_ptr<Edge>>& edges,
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions);
    std::pair<Collider*, Collider*> makePair(Collider* a, Collider* b);
};