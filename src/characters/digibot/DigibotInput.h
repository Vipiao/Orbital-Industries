// DigibotInput.h — a digibot's per-step control intent as a plain value: what the
// character is asked to do this tick, decoupled from how it was produced (keyboard,
// a stored command, or a replay). Serializable so the same intent can be reproduced
// wherever it is applied.
#pragma once

#include "DigibotModeTypes.h"

#include "utils/ByteStream.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <glm/glm.hpp>

struct DigibotInput {
    // Value of m_lockTargetGridId meaning "no target"; distinct from every real grid
    // id (including 0), so a genuine lock target is never mistaken for none.
    static constexpr std::uint64_t k_noLockTargetGridId{
        std::numeric_limits<std::uint64_t>::max()};

    // Largest rotation command accepted, in rad/s^2. Deserialized input is
    // untrusted, and this one is a floating point value straight off a mouse.
    static constexpr double k_maxRotationCommand{1.0e3};

    glm::ivec3 m_movementDirection{0, 0, 0};
    glm::dvec3 m_viewDirection{0.0, 1.0, 0.0};
    // Rotation the pilot asks of the ship they are seated in, as an angular
    // acceleration in the cockpit's frame (rad/s^2). Zero when not seated.
    glm::dvec3 m_rotationCommand{0.0, 0.0, 0.0};
    int m_rollInput{0};
    bool m_upDirectionLocked{false};
    bool m_jetpackEnabled{true};

    DigibotLockState m_lockState{DigibotLockState::UNLOCKED};
    // Grid whose body the character locks to; k_noLockTargetGridId when unlocked. An
    // id rather than a pointer so it can be serialized and resolved where applied.
    std::uint64_t m_lockTargetGridId{k_noLockTargetGridId};

    void serialize(ByteWriter& writer) const {
        writer.write(m_movementDirection.x);
        writer.write(m_movementDirection.y);
        writer.write(m_movementDirection.z);
        writer.write(m_viewDirection.x);
        writer.write(m_viewDirection.y);
        writer.write(m_viewDirection.z);
        writer.write(m_rotationCommand.x);
        writer.write(m_rotationCommand.y);
        writer.write(m_rotationCommand.z);
        writer.write(m_rollInput);
        writer.write(m_upDirectionLocked);
        writer.write(m_jetpackEnabled);
        writer.write(m_lockState);
        writer.write(m_lockTargetGridId);
    }

    bool deserialize(ByteReader& reader) {
        return reader.read(m_movementDirection.x) && reader.read(m_movementDirection.y) &&
               reader.read(m_movementDirection.z) && reader.read(m_viewDirection.x) &&
               reader.read(m_viewDirection.y) && reader.read(m_viewDirection.z) &&
               reader.read(m_rotationCommand.x) && reader.read(m_rotationCommand.y) &&
               reader.read(m_rotationCommand.z) &&
               reader.read(m_rollInput) && reader.read(m_upDirectionLocked) &&
               reader.read(m_jetpackEnabled) && reader.read(m_lockState) &&
               reader.read(m_lockTargetGridId);
    }

    // Whether every field is in its legal domain. Deserialized input is untrusted;
    // an out-of-range value (a NaN view direction, a raw enum byte) must be dropped
    // at the door, not applied.
    bool isValid() const {
        auto axisValid = [](int value) { return value >= -1 && value <= 1; };
        return axisValid(m_movementDirection.x) && axisValid(m_movementDirection.y) &&
               axisValid(m_movementDirection.z) && axisValid(m_rollInput) &&
               std::isfinite(m_viewDirection.x) && std::isfinite(m_viewDirection.y) &&
               std::isfinite(m_viewDirection.z) &&
               glm::length(m_viewDirection) > 1e-6 &&
               std::isfinite(m_rotationCommand.x) && std::isfinite(m_rotationCommand.y) &&
               std::isfinite(m_rotationCommand.z) &&
               glm::length(m_rotationCommand) <= k_maxRotationCommand &&
               m_lockState >= DigibotLockState::UNLOCKED &&
               m_lockState <= DigibotLockState::FULL_LOCK;
    }
};
