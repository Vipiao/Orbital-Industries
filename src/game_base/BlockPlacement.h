// BlockPlacement.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Local position for an instance anchored at a grid cell and rotated about its
// model centre, so the piece stays aligned with its collider cells regardless of
// orientation. Shared by the block and plume instance wrappers.
inline glm::dvec3 blockLocalPosition(const glm::ivec3& anchorCoord,
                                     const glm::dquat& orientation,
                                     const glm::dvec3& modelCentre) {
    return glm::dvec3{anchorCoord} + modelCentre - glm::dvec3{orientation * modelCentre};
}
