// src/characters/ArticulationUtils.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <memory>

// Forward declarations
class Geometry;
class Instance;

class ArticulationUtils {
public:
    /**
     * @brief Apply distance constraint to keep a point at fixed distance from anchor
     * @param point Point to constrain
     * @param anchor Fixed anchor point
     * @param targetDistance Desired distance from anchor
     * @return Constrained point at targetDistance from anchor
     */
    static glm::dvec3 applyDistanceConstraint(const glm::dvec3& point, const glm::dvec3& anchor, double targetDistance);

    /**
     * @brief Nudge a point toward a preferred direction
     * @param point Point to nudge
     * @param preferredDirection Direction to nudge toward (should be normalized)
     * @param nudgeStrength How far to nudge (small value like 0.01)
     * @return Nudged point
     */
    static glm::dvec3 applyDirectionNudge(const glm::dvec3& point, const glm::dvec3& preferredDirection, double nudgeStrength);

    /**
     * @brief Update an articulated body part's transform to point from one location to another
     * @param geometry Geometry weak pointer for the body part
     * @param instance Instance weak pointer for the body part
     * @param directionFrom Where the limb pivots from (current pivot position)
     * @param directionTo Where the limb points toward
     * @param upVector Reference vector for orientation (determines twist)
     * @param correctionQuat Model-specific correction (e.g., 90° Z-rotation if model points along X instead of Y)
     * @param naturalPivot Pivot point in T-pose/rest pose
     */
    static void updateArticulatedBodyPart(
        std::weak_ptr<Geometry> geometry,
        std::weak_ptr<Instance> instance,
        const glm::dvec3& directionFrom,
        const glm::dvec3& directionTo,
        const glm::dvec3& upVector,
        const glm::dquat& correctionQuat,
        const glm::dvec3& naturalPivot
    );

    /**
     * @brief Create quaternion that makes +Y axis point along direction
     * @param direction Direction for +Y to point along (should be normalized)
     * @param up Reference up vector for controlling twist
     * @return Quaternion representing the orientation
     */
    static glm::dquat quatLookAtYForward(const glm::dvec3& direction, const glm::dvec3& up);
};