// StructuralCommand.h — a discrete edit to the world's structure as a value.
// Pure data plus serialization; GameBase::applyStructural realises one against the
// world. Useful wherever edits are represented as data rather than applied inline
// (e.g. save, replay, deferred application).
#pragma once

#include "CellType.h"
#include "GridSplitPiece.h"
#include "utils/ByteStream.h"
#include "utils/PolyhedronProcessor.h"

#include <array>
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

enum class StructuralOp : std::uint8_t {
    SetColor, AddCell, RemoveCell, ModifyCell, AddThruster, AddCockpit,
    AddReactionWheel, SplitGrid, SpawnGrid, DespawnGrid
};

struct StructuralCommand {
    StructuralOp m_op{StructuralOp::SetColor};
    std::uint64_t m_gridId{0};
    glm::ivec3 m_coord{0, 0, 0};
    glm::dvec4 m_color{1.0, 1.0, 1.0, 1.0};   // SetColor/AddCell/SpawnGrid
    int m_cornerIndex{0};                     // ModifyCell: which corner (0-7)
    glm::ivec3 m_direction{0, 0, 0};          // ModifyCell: how to nudge that corner
    // AddThruster/AddCockpit/AddReactionWheel/SpawnGrid seed
    glm::dquat m_orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 m_position{0.0, 0.0, 0.0};     // SpawnGrid: where the new grid appears
    CellType m_cellType{CellType::STRUCTURAL_BLOCK};  // SpawnGrid: block seeding the grid
    std::vector<GridSplitPiece> m_pieces{};   // SplitGrid: pieces breaking off m_gridId
    // AddCell/SpawnGrid: shape of the structural block being placed
    std::array<glm::ivec3, 8> m_vertices{PolyhedronProcessor::DEFAULT_VERTICES};

    void serialize(ByteWriter& writer) const;
    bool deserialize(ByteReader& reader);

    static StructuralCommand setColor(std::uint64_t gridId, const glm::ivec3& coord,
                                      const glm::dvec4& color);
    static StructuralCommand addCell(
        std::uint64_t gridId, const glm::ivec3& coord,
        const std::array<glm::ivec3, 8>& vertices = PolyhedronProcessor::DEFAULT_VERTICES,
        const glm::dvec4& color = glm::dvec4{1.0, 1.0, 1.0, 1.0});
    static StructuralCommand removeCell(std::uint64_t gridId, const glm::ivec3& coord);
    static StructuralCommand modifyCell(std::uint64_t gridId, const glm::ivec3& coord,
                                        int cornerIndex, const glm::ivec3& direction);
    static StructuralCommand addThruster(std::uint64_t gridId, const glm::ivec3& anchorCoord,
                                         const glm::dquat& orientation);
    static StructuralCommand addCockpit(std::uint64_t gridId, const glm::ivec3& anchorCoord,
                                        const glm::dquat& orientation);
    static StructuralCommand addReactionWheel(std::uint64_t gridId,
                                              const glm::ivec3& anchorCoord,
                                              const glm::dquat& orientation);
    static StructuralCommand splitGrid(std::uint64_t sourceGridId,
                                       std::vector<GridSplitPiece> pieces);
    // A new grid seeded with the given block at its origin. Sent with id 0; the
    // server allocates the real id before applying and broadcasting.
    static StructuralCommand spawnGrid(
        const glm::dvec3& position,
        CellType cellType = CellType::STRUCTURAL_BLOCK,
        const glm::dquat& orientation = glm::dquat{1.0, 0.0, 0.0, 0.0},
        const std::array<glm::ivec3, 8>& vertices = PolyhedronProcessor::DEFAULT_VERTICES,
        const glm::dvec4& color = glm::dvec4{1.0, 1.0, 1.0, 1.0});
    static StructuralCommand despawnGrid(std::uint64_t gridId);
};
