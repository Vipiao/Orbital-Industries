// CollisionDetector.cpp
#include "CollisionDetector.h"
#include "CollisionDetectionUtils.h"
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

void CollisionDetector::addCollider(Collider* collider) {
    if (!collider) return;
    
    // Add to colliders list
    colliders.push_back(collider);
    
    // Create edges for this collider
    edgesX.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::X));
    edgesX.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::X));
    
    edgesY.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::Y));
    edgesY.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::Y));
    
    edgesZ.push_back(std::make_unique<Edge>(collider, EdgeType::MIN, EdgeAxis::Z));
    edgesZ.push_back(std::make_unique<Edge>(collider, EdgeType::MAX, EdgeAxis::Z));
}

void CollisionDetector::removeCollider(Collider* collider) {
    if (!collider) return;
    
    // Remove from colliders list
    colliders.erase(std::remove(colliders.begin(), colliders.end(), collider), colliders.end());
    
    // Remove edges for this collider
    auto removeEdges = [collider](std::vector<std::unique_ptr<Edge>>& edges) {
        edges.erase(
            std::remove_if(edges.begin(), edges.end(),
                [collider](const std::unique_ptr<Edge>& edge) {
                    return edge->m_collider == collider;
                }),
            edges.end()
        );
    };
    
    removeEdges(edgesX);
    removeEdges(edgesY);
    removeEdges(edgesZ);

    
    // Remove any active collision pairs involving this collider
    auto it = m_activeAABBS.begin();
    while (it != m_activeAABBS.end()) {
        if (it->first == collider || it->second == collider) {
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

Generator<bool> CollisionDetector::run(std::vector<CollisionResult>& collisions) {
    // Update all collision boxes
    updateAllCollidersAndAABB();
    if (m_timeHandler->now() >= m_endTime) {
        co_yield true; // Need more time
    }

    std::set<std::pair<Collider*, Collider*>> potentialCollisions;

    // Check existing active collisions and remove ones that are no longer colliding
    auto it = m_activeAABBS.begin();
    while (it != m_activeAABBS.end()) {
        if (!it->first->checkAABBCollision(it->second)) {
            // Clear cached data for this collider pair since they're no longer colliding
            PairCache<glm::dvec3>::clearCachedData(it->first, it->second);  // SAT axes
            PairCache<int>::clearCachedData(it->first, it->second);         // Contact counts
            
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
        checkCollision(pair.first, pair.second, collisions);
    }
}

void CollisionDetector::updateAllCollidersAndAABB() {
    for (Collider* collider : colliders) {
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
                                               std::set<std::pair<Collider*, Collider*>>& potentialCollisions) {
    // Sort and collect potential collision pairs during sorting
    insertionSort(edges, potentialCollisions);
}

void CollisionDetector::insertionSort(std::vector<std::unique_ptr<Edge>>& edges, 
                                     std::set<std::pair<Collider*, Collider*>>& potentialCollisions) {
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
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

void CollisionDetector::checkCollision(Collider* collider1, Collider* collider2, std::vector<CollisionResult>& collisions) {
    if (!collider1 || !collider2 || collider1 == collider2) return;

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
        //std::cout << "Collision detected between two colliders!" << std::endl;
        // Set collider references for resolution
        result.m_colliderA = collider1;
        result.m_colliderB = collider2;
        
        // Store collision for resolution
        collisions.push_back(result);
    }
}