// GridSerializer.h — a grid's buildable content as bytes and as a fingerprint:
// what the grid is (cells, shapes, colors, special blocks), independent of how it
// was built, how it is moving, or how its containers iterate. Used wherever grid
// content is compared or transferred between simulations (world sync, save files).
#pragma once

#include "../serialization/ByteStream.h"

#include <cstddef>
#include <cstdint>
#include <memory>

class Grid;
class GridSubsystem;

class GridSerializer {
public:
    // Fingerprint of the structure: structural cells (coord, vertices, color) and
    // thruster/cockpit anchors (coord, orientation), folded in sorted coord order.
    // Excludes pose, motion, throttle and per-peer ids, so two simulations holding
    // the same structure agree on the hash regardless of edit history. Cached per
    // grid id, invalidated by Grid::getStructureVersion() — cheap to call every
    // time rather than only when something might have changed.
    static std::size_t structureHash(const Grid& grid);

    // Grid id, pose (position + orientation) and the structure, in the same sorted
    // order as the hash. Motion is deliberately excluded: the consumer syncs it
    // separately (state snapshots) or stores a RigidBodyState alongside (saves).
    static void serialize(const Grid& grid, ByteWriter& writer);

    // Create the grid described by serialize() through the subsystem and rebuild
    // its content. Returns an empty pointer on malformed data or if a grid with
    // that id already exists (the caller decides whether to despawn first).
    static std::weak_ptr<Grid> deserialize(ByteReader& reader,
                                           GridSubsystem& gridSubsystem);
};
