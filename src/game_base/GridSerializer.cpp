#include "GridSerializer.h"

#include "Grid.h"
#include "GridSubsystem.h"
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

// Coordinate-sorted keys of a coord-keyed map: the canonical iteration order for
// hashing and serialization, independent of container history.
template <typename Map>
static std::vector<glm::ivec3> sortedKeys(const Map& map) {
    std::vector<glm::ivec3> keys{};
    keys.reserve(map.size());
    for (const auto& [coord, value] : map) {
        keys.push_back(coord);
    }
    std::sort(keys.begin(), keys.end(), [](const glm::ivec3& a, const glm::ivec3& b) {
        return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
    });
    return keys;
}

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

static std::size_t computeStructureHash(const Grid& grid) {
    std::size_t hash{0};

    // Distinct tags per section: a thruster and a cockpit at the same coord with
    // the same orientation must not hash alike.
    hash = Hash::combineHashes(hash, std::hash<int>{}(1));
    for (const glm::ivec3& coord : sortedKeys(grid.getCells())) {
        const StructuralBlock& block{grid.getCells().at(coord)};
        hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coord));
        for (const glm::ivec3& vertex : block.m_localVertices) {
            hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(vertex));
        }
        for (int ii = 0; ii < 4; ii++) {
            hash = Hash::combineHashes(hash, std::hash<double>{}(block.m_color[ii]));
        }
    }

    hash = Hash::combineHashes(hash, std::hash<int>{}(2));
    for (const glm::ivec3& coord : sortedKeys(grid.getThrusterCells())) {
        hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coord));
        hash = Hash::combineHashes(
            hash, Hash::DQuatHash{}(grid.getThrusterCells().at(coord).m_orientation));
    }

    hash = Hash::combineHashes(hash, std::hash<int>{}(3));
    for (const glm::ivec3& coord : sortedKeys(grid.getCockpitCells())) {
        hash = Hash::combineHashes(hash, Hash::IVec3Hash{}(coord));
        hash = Hash::combineHashes(
            hash, Hash::DQuatHash{}(grid.getCockpitCells().at(coord).m_orientation));
    }

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

    glm::dvec3 position{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    std::shared_ptr<RigidBody> body{grid.getRigidBody().lock()};
    if (body) {
        position = body->m_position;
        orientation = body->m_orientation;
    }
    writer.write(position.x);
    writer.write(position.y);
    writer.write(position.z);
    writeQuat(writer, orientation);

    writer.write(static_cast<std::uint32_t>(grid.getCells().size()));
    for (const glm::ivec3& coord : sortedKeys(grid.getCells())) {
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

    writer.write(static_cast<std::uint32_t>(grid.getThrusterCells().size()));
    for (const glm::ivec3& coord : sortedKeys(grid.getThrusterCells())) {
        writer.write(coord.x);
        writer.write(coord.y);
        writer.write(coord.z);
        writeQuat(writer, grid.getThrusterCells().at(coord).m_orientation);
    }

    writer.write(static_cast<std::uint32_t>(grid.getCockpitCells().size()));
    for (const glm::ivec3& coord : sortedKeys(grid.getCockpitCells())) {
        writer.write(coord.x);
        writer.write(coord.y);
        writer.write(coord.z);
        writeQuat(writer, grid.getCockpitCells().at(coord).m_orientation);
    }
}

std::weak_ptr<Grid> GridSerializer::deserialize(ByteReader& reader,
                                                GridSubsystem& gridSubsystem) {
    std::uint64_t id{0};
    glm::dvec3 position{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    if (!reader.read(id) || !reader.read(position.x) || !reader.read(position.y) ||
        !reader.read(position.z) || !readQuat(reader, orientation)) {
        return {};
    }
    if (gridSubsystem.getGridById(id).lock()) {
        return {};  // occupied id; the caller decides whether to despawn first
    }

    std::shared_ptr<Grid> grid{gridSubsystem.createGrid(id, position, orientation).lock()};
    if (!grid) {
        return {};
    }

    std::uint32_t cellCount{0};
    if (!reader.read(cellCount)) {
        return grid;
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
            return grid;
        }
        grid->addCell(coord);
        grid->modifyCell(coord, vertices);
        grid->setColor(coord, color);
    }

    std::uint32_t thrusterCount{0};
    if (!reader.read(thrusterCount)) {
        return grid;
    }
    for (std::uint32_t ii = 0; ii < thrusterCount; ii++) {
        glm::ivec3 coord{0};
        glm::dquat blockOrientation{1.0, 0.0, 0.0, 0.0};
        if (!(reader.read(coord.x) && reader.read(coord.y) && reader.read(coord.z) &&
              readQuat(reader, blockOrientation))) {
            return grid;
        }
        grid->addThruster(coord, blockOrientation);
    }

    std::uint32_t cockpitCount{0};
    if (!reader.read(cockpitCount)) {
        return grid;
    }
    for (std::uint32_t ii = 0; ii < cockpitCount; ii++) {
        glm::ivec3 coord{0};
        glm::dquat blockOrientation{1.0, 0.0, 0.0, 0.0};
        if (!(reader.read(coord.x) && reader.read(coord.y) && reader.read(coord.z) &&
              readQuat(reader, blockOrientation))) {
            return grid;
        }
        grid->addCockpit(coord, blockOrientation);
    }

    return grid;
}
