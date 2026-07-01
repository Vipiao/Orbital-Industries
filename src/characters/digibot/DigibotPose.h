// DigibotPose.h
#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct BodyPartTransform {
    glm::dvec3 position;
    glm::dquat orientation;

    BodyPartTransform()
        : position{0.0, 0.0, 0.0}
        , orientation{1.0, 0.0, 0.0, 0.0}
    {}

    BodyPartTransform(const glm::dvec3& pos, const glm::dquat& ori)
        : position{pos}
        , orientation{ori}
    {}
};

struct DigibotPose {
    BodyPartTransform leftHand;
    BodyPartTransform rightHand;
    BodyPartTransform leftFoot;
    BodyPartTransform rightFoot;
    glm::dquat headOrientation;

    DigibotPose()
        : leftHand{}
        , rightHand{}
        , leftFoot{}
        , rightFoot{}
        , headOrientation{1.0, 0.0, 0.0, 0.0}
    {}
};
