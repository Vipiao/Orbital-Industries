#include "StateSnapshot.h"

void StateSnapshot::serialize(ByteWriter& writer) const {
    writer.write(m_tick);
    writer.write(static_cast<std::uint32_t>(m_grids.size()));
    for (const GridEntry& grid : m_grids) {
        writer.write(grid.m_id);
        grid.m_state.serialize(writer);
    }
    writer.write(static_cast<std::uint32_t>(m_characters.size()));
    for (const CharacterEntry& character : m_characters) {
        writer.write(character.m_id);
        character.m_state.serialize(writer);
        character.m_input.serialize(writer);
        character.m_docking.serialize(writer);
    }
}

bool StateSnapshot::deserialize(ByteReader& reader) {
    std::uint32_t gridCount{0};
    if (!reader.read(m_tick) || !reader.read(gridCount)) {
        return false;
    }
    // Entries append one at a time so a hostile count cannot allocate ahead of the
    // actual payload; a short buffer fails on its first missing field.
    for (std::uint32_t ii = 0; ii < gridCount; ii++) {
        GridEntry grid{};
        if (!reader.read(grid.m_id) || !grid.m_state.deserialize(reader)) {
            return false;
        }
        m_grids.push_back(grid);
    }
    std::uint32_t characterCount{0};
    if (!reader.read(characterCount)) {
        return false;
    }
    for (std::uint32_t ii = 0; ii < characterCount; ii++) {
        CharacterEntry character{};
        if (!reader.read(character.m_id) || !character.m_state.deserialize(reader) ||
            !character.m_input.deserialize(reader) ||
            !character.m_docking.deserialize(reader)) {
            return false;
        }
        m_characters.push_back(character);
    }
    return true;
}
