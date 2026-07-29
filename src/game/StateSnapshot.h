// StateSnapshot.h — one tick's authoritative world state as a value: every grid and
// character body, plus each character's input and docking status (levels riding the
// unreliable channel; a lost snapshot is superseded by the next tick's). Pure data
// with serialization; building it from the world and applying it are the caller's.
#pragma once

#include "../characters/digibot/DigibotInput.h"
#include "../game_base/RigidBodyState.h"
#include "../game_base/cockpit/CockpitDockingCoordinator.h"

#include <cstdint>
#include <vector>

struct StateSnapshot {
    struct GridEntry {
        std::uint64_t m_id{0};
        RigidBodyState m_state{};
    };
    struct CharacterEntry {
        std::int32_t m_id{0};
        RigidBodyState m_state{};
        DigibotInput m_input{};
        CockpitDockingCoordinator::DockingStatus m_docking{};
    };

    std::uint64_t m_tick{0};
    std::vector<GridEntry> m_grids{};
    std::vector<CharacterEntry> m_characters{};

    void serialize(ByteWriter& writer) const;
    // Whole-message parse; on false the partially filled snapshot must be discarded.
    bool deserialize(ByteReader& reader);
};
