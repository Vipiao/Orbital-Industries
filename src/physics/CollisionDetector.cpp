// CollisionDetector.cpp
#include "CollisionDetector.h"
#include "CollisionDetectionUtils.h"
#include "GridCollider.h"
#include "PolyhedronCollider.h"
#include "BallCollider.h"
#include "CubeCollider.h"
#include "SensorCollider.h"
#include <algorithm>
#include "../utils/PairCache.h"
#include <iostream>
#include "../utils/TimeHandler.h"
 
CollisionDetector::CollisionDetector(TimeHandler* timeHandler) 
    : m_timeHandler(timeHandler) {
    if (!m_timeHandler) {
        throw std::invalid_argument("TimeHandler cannot be null");
    }
}

void CollisionDetector::registerCollider(std::shared_ptr<Collider> collider) {
    if (!collider) return;
    
    // Store ownership
    m_colliders.push_back(collider);
    
    // Create edges for this collider
    Collider* rawPtr = collider.get();
    edgesX.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MIN, EdgeAxis::X));
    edgesX.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MAX, EdgeAxis::X));
    
    edgesY.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MIN, EdgeAxis::Y));
    edgesY.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MAX, EdgeAxis::Y));
    
    edgesZ.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MIN, EdgeAxis::Z));
    edgesZ.push_back(std::make_unique<Edge>(rawPtr, EdgeType::MAX, EdgeAxis::Z));
}

std::weak_ptr<GridCollider> CollisionDetector::addGridCollider(
    const glm::dvec3& position,
    const glm::dquat& orientation,
    JobManager* jobManager,
    TimeHandler* timeHandler) {
    // TODO. Probably it is better if CollisionDetector just passes its own
    // pointer to jobManager and timeHandler to the new colliders instead of
    // this function taking it each time.
    
    auto collider = std::make_shared<GridCollider>(
        position, orientation, jobManager, timeHandler);
    registerCollider(collider);
    return collider;
}

std::weak_ptr<PolyhedronCollider> CollisionDetector::addPolyhedronCollider(
    const glm::dvec3& position,
    const glm::dquat& orientation,
    const std::vector<glm::dvec3>& localVertices,
    const std::vector<glm::dvec3>& localFaceAxes,
    const std::vector<glm::dvec3>& localEdgeAxes) {
    
    auto collider = std::make_shared<PolyhedronCollider>(
        position, orientation, localVertices, localFaceAxes, localEdgeAxes);
    registerCollider(collider);
    return collider;
}

std::weak_ptr<BallCollider> CollisionDetector::addBallCollider(
    const glm::dvec3& position,
    double radius) {
    
    auto collider = std::make_shared<BallCollider>(
        position,
        glm::dquat(1.0, 0.0, 0.0, 0.0),
        radius);
    registerCollider(collider);
    return collider;
}

std::weak_ptr<CubeCollider> CollisionDetector::addCubeCollider(
    const glm::dvec3& position,
    const glm::dquat& orientation,
    double halfWidth) {
    
    auto collider = std::make_shared<CubeCollider>(
        position,
        orientation,
        halfWidth);
    registerCollider(collider);
    return collider;
}

std::weak_ptr<SensorCollider> CollisionDetector::addSensorCollider(
    const glm::dvec3& position,
    const glm::dvec3& halfScale) {
    
    auto collider = std::make_shared<SensorCollider>(position, halfScale);
    registerCollider(collider);
    return collider;
}

void CollisionDetector::removeCollider(std::weak_ptr<Collider> colliderWeak) {
    auto collider = colliderWeak.lock();
    if (!collider) {
        return; // Already destroyed
    }
    
    // Remove from colliders list (releases ownership, destroying collider)
    m_colliders.erase(
        std::remove_if(m_colliders.begin(), m_colliders.end(),
            [&collider](const std::shared_ptr<Collider>& ptr) {
                return ptr.get() == collider.get();
            }),
        m_colliders.end());
    
    // Remove edges for this collider (use raw pointer for edge removal)
    Collider* rawPtr = collider.get();

    // Clear overlap cache for this collider
    rawPtr->m_overlappingColliders.clear();

    auto removeEdges = [rawPtr](std::vector<std::unique_ptr<Edge>>& edges) {
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [rawPtr](const std::unique_ptr<Edge>& edge) {
                    return edge->m_collider == rawPtr;
                }),
            edges.end()
        );
    };
    
    removeEdges(edgesX);
    removeEdges(edgesY);
    removeEdges(edgesZ);
    
    // Remove any active collision pairs involving this collider (use raw pointer)
    auto it = m_activeAABBS.begin();
    while (it != m_activeAABBS.end()) {
        if (it->first == rawPtr || it->second == rawPtr) {
            // Remove from overlap caches
            if (it->first != rawPtr) {
                it->first->m_overlappingColliders.erase(rawPtr);
            }
            if (it->second != rawPtr) {
                it->second->m_overlappingColliders.erase(rawPtr);
            }
            it = m_activeAABBS.erase(it);
        } else {
            ++it;
        }
    }
}

void CollisionDetector::setEndTime(std::chrono::time_point<std::chrono::high_resolution_clock> endTime) {
    m_endTime = endTime;
}

void CollisionDetector::setTimestep(uint64_t timestep) {
    m_currentTimestep = timestep;
}

Generator<bool> CollisionDetector::run() {
    // Update all collision boxes
    updateAllCollidersAndAABB();
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }

    std::set<std::pair<Collider*, Collider*>, ColliderPairComparator> potentialCollisions;

    // Check existing active collisions and remove ones that are no longer colliding
    auto it = m_activeAABBS.begin();
    while (it != m_activeAABBS.end()) {
        if (!it->first->checkAABBCollision(it->second)) {
            // Clear cached data for this collider pair since they're no longer colliding
            PairCache<glm::dvec3>::clearCachedData(it->first->m_debugId, it->second->m_debugId);  // SAT axes
            PairCache<int>::clearCachedData(it->first->m_debugId, it->second->m_debugId);         // Contact counts
            
            // Remove from overlap caches
            it->first->m_overlappingColliders.erase(it->second);
            it->second->m_overlappingColliders.erase(it->first);

            // Remove from active collisions
            it = m_activeAABBS.erase(it);
        } else {
            ++it;
        }
    }
    
    // Update all edge values
    updateAllEdgeValues();
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }
    
    // Sort and detect collisions for each axis
    sortAndDetectPotentialCollisions(edgesX, potentialCollisions);
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }
    sortAndDetectPotentialCollisions(edgesY, potentialCollisions);
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }
    sortAndDetectPotentialCollisions(edgesZ, potentialCollisions);
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }

    // Now check potential collisions that overlap on ALL axes
    int processed = 0;
    for (const auto& pair : potentialCollisions) {
        if (pair.first->checkAABBCollision(pair.second)) {
            // Add to active collisions
            m_activeAABBS.insert(pair);

            // Add to overlap caches
            pair.first->m_overlappingColliders.insert(pair.second);
            pair.second->m_overlappingColliders.insert(pair.first);

            // Check time every 50 successful AABB checks
            if (++processed % 50 == 0 && m_timeHandler->now() >= m_endTime) {
                co_yield true; // Need more time
            }
        }
    }
    
    // Perform detailed collision detection on all active collisions
    for (const auto& pair : m_activeAABBS) {
        if (m_timeHandler->now() >= m_endTime) {
            co_yield true; // Need more time
        }
        checkCollision(pair.first, pair.second);
    }

    co_return;
}

void CollisionDetector::updateAllCollidersAndAABB() {
    for (auto& collider : m_colliders) {
        if (collider) {
            collider->updateSimpleAABB(m_currentTimestep);
        }
    }
}

void CollisionDetector::updateAllEdgeValues() {
    for (auto& edge : edgesX) {
        edge->updateValue();
    }
    for (auto& edge : edgesY) {
        edge->updateValue();
    }
    for (auto& edge : edgesZ) {
        edge->updateValue();
    }
}

void CollisionDetector::sortAndDetectPotentialCollisions(std::vector<std::unique_ptr<Edge>>& edges, 
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions) {
    // Sort and collect potential collision pairs during sorting
    insertionSort(edges, potentialCollisions);
}

void CollisionDetector::insertionSort(std::vector<std::unique_ptr<Edge>>& edges, 
        std::set<std::pair<Collider*, Collider*>, ColliderPairComparator>& potentialCollisions) {
    for (size_t i = 1; i < edges.size(); ++i) {
        size_t j = i;
        while (j > 0 && edges[j]->m_value < edges[j-1]->m_value) {
           // Check if we're swapping a MAX edge past a MIN edge
           if (edges[j-1]->m_type == EdgeType::MAX && edges[j]->m_type == EdgeType::MIN) {
               // Potential collision detected
               auto pair = makePair(edges[j-1]->m_collider, edges[j]->m_collider);
               potentialCollisions.insert(pair);
           }
           
            std::swap(edges[j], edges[j-1]);
            j--;
        }
    }
}

std::pair<Collider*, Collider*> CollisionDetector::makePair(Collider* a, Collider* b) {
    // Ensure consistent ordering to avoid duplicates
    return (a->m_debugId < b->m_debugId) ? std::make_pair(a, b) : std::make_pair(b, a);
}

void CollisionDetector::checkCollision(Collider* collider1, Collider* collider2) {
    if (!collider1 || !collider2 || collider1 == collider2) return;

    // Skip collision detection if either collider is a sensor
    if (collider1->getTypeId() == SensorCollider::TYPE_ID || 
        collider2->getTypeId() == SensorCollider::TYPE_ID) {
        return;
    }

    // Update advanced AABBs for precise collision detection
    collider1->updateAdvancedAABB(m_currentTimestep);
    collider2->updateAdvancedAABB(m_currentTimestep);
    
    // Do precise AABB test with updated bounds
    if (!collider1->checkAABBCollision(collider2)) {
        return; // No collision even with precise AABBs
    }
    
    // Perform collision detection between the two colliders
    CollisionResult result = CollisionDetectionUtils::collideWith(collider1, collider2, m_currentTimestep);
    
    if (result.m_hasCollision) {
        // Store collision data on BOTH colliders
        
        // Data for collider1 (from collider1's perspective)
        CollisionData data1;
        data1.otherCollider = collider2;
        data1.contactData = result.m_contactData;
        data1.contactPoints = result.m_contactPoints;
        data1.contactPointsLocalA = result.m_contactPointsLocalA;
        data1.contactPointsLocalB = result.m_contactPointsLocalB;
        
        // Data for collider2 (from collider2's perspective - swap A/B)
        CollisionData data2;
        data2.otherCollider = collider1;
        data2.contactData = result.m_contactData;
        data2.contactPoints = result.m_contactPoints;
        data2.contactPointsLocalA = result.m_contactPointsLocalB;  // Swap
        data2.contactPointsLocalB = result.m_contactPointsLocalA;  // Swap
        
        // Write to colliders with current timestamp
        collider1->addCollision(data1, m_currentTimestep);
        collider2->addCollision(data2, m_currentTimestep);
    }
}