#include "GridSerializer.h"

#include "CoordOrder.h"
#include "Grid.h"
#include "GridSubsystem.h"
#include "RigidBodyState.h"
#include "StructuralBlock.h"
#include "cockpit/CockpitBlock.h"
#include "thruster/ThrusterBlock.h"
#include "../physics/RigidBody.h"
#include "utils/HashFunctions.h"

#include <algorithm>
#include <array>
#include <tuple>
#include <unordered_map>
#include <vector>

static void writeQuat(ByteWriter& writer, const glm::dquat& quat) {
    writer.write(quat.w);
    writer.write(quat.x);
    writer.write(quat.y);
    writer.write(quat.z);
}

static bool readQuat(ByteReader& reader, glm::dquat& quat) {
    return reader.read(quat.w) && reader.read(quat.x) &&
           reader.read(quat.y) && reader.read(quat.z);
}

// Coord and orientation are all any special block needs to be rebuilt, so one
// anchor map is handled the same way whatever type of block it holds. The tag
// separates the sections: a thruster and a cockpit anchored at the same coord
// with the same orientation must not hash alike.
template <typename Map>
static std::size_t hashAnchors(std::size_t hash, int tag, const Map& anchors) {
    hash = Hash::combineHashes(hash, std::hash<int>{}(tag));
    for (const glm::ivec3& coord : sortedCoords(anchors)) {
        hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coord));
        hash = Hash::combineHashes(hash, Hash::DQuatHash{}(anchors.at(coord).m_orientation));
    }
    return hash;
}

template <typename Map>
static void writeAnchors(ByteWriter& writer, const Map& anchors) {
    writer.write(static_cast<std::uint32_t>(anchors.size()));
    for (const glm::ivec3& coord : sortedCoords(anchors)) {
        writer.write(coord.x);
        writer.write(coord.y);
        writer.write(coord.z);
        writeQuat(writer, anchors.at(coord).m_orientation);
    }
}

// Rebuild one section through the grid's own placement method. Returns false on
// malformed data, leaving the grid built as far as the stream was readable.
static bool readAnchors(ByteReader& reader, Grid& grid,
                        void (Grid::*placeBlock)(const glm::ivec3&, const glm::dquat&)) {
    std::uint32_t count{0};
    if (!reader.read(count)) {
        return false;
    }
    for (std::uint32_t ii = 0; ii < count; ii++) {
        glm::ivec3 coord{0};
        glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
        if (!(reader.read(coord.x) && reader.read(coord.y) && reader.read(coord.z) &&
              readQuat(reader, orientation))) {
            return false;
        }
        (grid.*placeBlock)(coord, orientation);
    }
    return true;
}

// Cells then anchor blocks, in the order they were written. A truncated stream
// simply stops the rebuild where it ran out.
static void readStructure(ByteReader& reader, Grid& grid) {
    std::uint32_t cellCount{0};
    if (!reader.read(cellCount)) {
        return;
    }
    for (std::uint32_t ii = 0; ii < cellCount; ii++) {
        glm::ivec3 coord{0};
        std::array<glm::ivec3, 8> vertices{};
        glm::dvec4 color{1.0};
        bool ok{reader.read(coord.x) && reader.read(coord.y) && reader.read(coord.z)};
        for (glm::ivec3& vertex : vertices) {
            ok = ok && reader.read(vertex.x) && reader.read(vertex.y) &&
                 reader.read(vertex.z);
        }
        for (int jj = 0; jj < 4; jj++) {
            ok = ok && reader.read(color[jj]);
        }
        if (!ok) {
            return;
        }
        grid.addCell(coord, vertices, color);
    }

    for (void (Grid::*placeBlock)(const glm::ivec3&, const glm::dquat&) :
         {&Grid::addThruster, &Grid::addCockpit, &Grid::addReactionWheel}) {
        if (!readAnchors(reader, grid, placeBlock)) {
            return;
        }
    }
}

static std::size_t computeStructureHash(const Grid& grid) {
    std::size_t hash{0};

    hash = Hash::combineHashes(hash, std::hash<int>{}(1));
    for (const glm::ivec3& coord : sortedCoords(grid.getCells())) {
        const StructuralBlock& block{grid.getCells().at(coord)};
        hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coord));
        for (const glm::ivec3& vertex : block.m_localVertices) {
            hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(vertex));
        }
        for (int ii = 0; ii < 4; ii++) {
            hash = Hash::combineHashes(hash, std::hash<double>{}(block.m_color[ii]));
        }
    }

    hash = hashAnchors(hash, 2, grid.getThrusterCells());
    hash = hashAnchors(hash, 3, grid.getCockpitCells());
    hash = hashAnchors(hash, 4, grid.getReactionWheelCells());

    return hash;
}

namespace {

// One cached hash per grid id, valid as long as the grid's structure version
// hasn't moved on. Entries for despawned grids are never pruned — ids are never
// reused, so a stale entry is just a few bytes that will never be looked up
// again, not a growing cost.
struct CachedStructureHash {
    uint64_t m_version{0};
    std::size_t m_hash{0};
};
std::unordered_map<uint64_t, CachedStructureHash> s_structureHashCache{};

}  // namespace

std::size_t GridSerializer::structureHash(const Grid& grid) {
    auto cached{s_structureHashCache.find(grid.uniqueId)};
    if (cached != s_structureHashCache.end() &&
        cached->second.m_version == grid.getStructureVersion()) {
        return cached->second.m_hash;
    }

    std::size_t hash{computeStructureHash(grid)};
    s_structureHashCache[grid.uniqueId] = CachedStructureHash{grid.getStructureVersion(), hash};
    return hash;
}

void GridSerializer::serialize(const Grid& grid, ByteWriter& writer) {
    writer.write(grid.uniqueId);

    // Motion travels with the pose: the receiver rebuilds the body from this alone,
    // and a grid restored at rest would coast away from the world it belongs to.
    RigidBodyState state{};
    if (std::shared_ptr<RigidBody> body{grid.getRigidBody().lock()}) {
        state = RigidBodyState::capture(*body);
    }
    state.serialize(writer);

    writer.write(static_cast<std::uint32_t>(grid.getCells().size()));
    for (const glm::ivec3& coord : sortedCoords(grid.getCells())) {
        const StructuralBlock& block{grid.getCells().at(coord)};
        writer.write(coord.x);
        writer.write(coord.y);
        writer.write(coord.z);
        for (const glm::ivec3& vertex : block.m_localVertices) {
            writer.write(vertex.x);
            writer.write(vertex.y);
            writer.write(vertex.z);
        }
        for (int ii = 0; ii < 4; ii++) {
            writer.write(block.m_color[ii]);
        }
    }

    writeAnchors(writer, grid.getThrusterCells());
    writeAnchors(writer, grid.getCockpitCells());
    writeAnchors(writer, grid.getReactionWheelCells());
}

std::weak_ptr<Grid> GridSerializer::deserialize(ByteReader& reader,
                                                GridSubsystem& gridSubsystem) {
    std::uint64_t id{0};
    RigidBodyState state{};
    // Screened here, at the ingress: the rigid body asserts on non-finite and
    // off-unit state, so decoded garbage must never reach it.
    if (!reader.read(id) || !state.deserialize(reader) || !state.isValid()) {
        return {};
    }
    if (gridSubsystem.getGridById(id).lock()) {
        return {};  // occupied id; the caller decides whether to despawn first
    }

    std::shared_ptr<Grid> grid{
        gridSubsystem.createGrid(id, state.m_position, state.m_orientation).lock()};
    if (!grid) {
        return {};
    }
    readStructure(reader, *grid);

    // Motion last: angular velocity is momentum over inertia, and a grid whose cells
    // are not in place yet has none, so any momentum set before this reads as an
    // overflowing spin.
    if (std::shared_ptr<RigidBody> body{grid->getRigidBody().lock()}) {
        body->m_velocity = state.m_velocity;
        body->setAngularMomentumBody(state.m_angularMomentumBody);
    }
    return grid;
}

