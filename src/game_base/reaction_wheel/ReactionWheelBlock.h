// ReactionWheelBlock.h
#pragma once

#include "../BlockAnchor.h"
#include "../BlockGeometryPart.h"
#include <string_view>
#include <tuple>
#include <vector>

// Anchor cell for a single-cell reaction wheel.
// Torques the grid about any axis, so orientation only selects how the model is
// drawn — the wheel has the same authority whichever way it is turned.
class ReactionWheelBlock : public BlockAnchor {
public:
    static constexpr CellType TYPE = CellType::REACTION_WHEEL;

    static constexpr std::string_view COLOR_TEX_PATH =
        "../media/models/reaction_wheel/albedo.png";
    static constexpr std::string_view NORMAL_TEX_PATH =
        "../media/models/reaction_wheel/normal.png";
    static constexpr glm::dvec3 MODEL_CENTRE{0.5, 0.5, 0.5};

    // Torque one wheel produces at full command, in newton metres.
    static constexpr double TORQUE_NEWTON_METERS{50000.0};

    // An opaque frame plus a translucent glass window onto the spinning rotor.
    static std::vector<BlockGeometryPart> geometryParts();

    ReactionWheelBlock(const glm::ivec3& anchorCoord, const glm::dquat& orientation);
    virtual ~ReactionWheelBlock() = default;

    // All offsets occupied by this block. One cell, in every orientation.
    static std::vector<glm::ivec3> footprintOffsets(const glm::dquat& orientation);

    // Torque the wheel is producing, in grid-local space, as a fraction of
    // TORQUE_NEWTON_METERS; its magnitude never exceeds 1. Recomputed from the
    // pilot's request and the ship's rotation on every step, so it never outlives
    // the conditions that produced it.
    glm::dvec3 m_torqueCommand{0.0, 0.0, 0.0};

    virtual std::vector<glm::ivec3> secondaryCoords() const override;
    virtual std::tuple<double, glm::dvec3, glm::dmat3> getMassProperties() const override;
    virtual size_t computeHash() const override;
};
