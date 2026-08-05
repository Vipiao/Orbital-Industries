#include "StructuralCommand.h"

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

static constexpr glm::dvec4 k_defaultColor{1.0, 1.0, 1.0, 1.0};

// An unshaped cube is by far the common case, so it costs a flag rather than a
// full vertex set on the wire.
static void writeVertices(ByteWriter& writer, const std::array<glm::ivec3, 8>& vertices) {
    const bool isShaped{vertices != PolyhedronProcessor::DEFAULT_VERTICES};
    writer.write(isShaped);
    if (isShaped) {
        writer.write(vertices);
    }
}

// Placed blocks are usually left the default colour, so that case costs a flag
// rather than four doubles.
static void writeColor(ByteWriter& writer, const glm::dvec4& color) {
    const bool isTinted{color != k_defaultColor};
    writer.write(isTinted);
    if (isTinted) {
        writer.write(color);
    }
}

static bool readColor(ByteReader& reader, glm::dvec4& color) {
    bool isTinted{false};
    if (!reader.read(isTinted)) {
        return false;
    }
    if (!isTinted) {
        color = k_defaultColor;
        return true;
    }
    return reader.read(color);
}

static bool readVertices(ByteReader& reader, std::array<glm::ivec3, 8>& vertices) {
    bool isShaped{false};
    if (!reader.read(isShaped)) {
        return false;
    }
    if (!isShaped) {
        vertices = PolyhedronProcessor::DEFAULT_VERTICES;
        return true;
    }
    return reader.read(vertices);
}

void StructuralCommand::serialize(ByteWriter& writer) const {
    writer.write(m_op);
    writer.write(m_gridId);
    writer.write(m_coord.x);
    writer.write(m_coord.y);
    writer.write(m_coord.z);
    switch (m_op) {
        case StructuralOp::SetColor:
            writer.write(m_color.x);
            writer.write(m_color.y);
            writer.write(m_color.z);
            writer.write(m_color.w);
            break;
        case StructuralOp::ModifyCell:
            writer.write(m_cornerIndex);
            writer.write(m_direction.x);
            writer.write(m_direction.y);
            writer.write(m_direction.z);
            break;
        case StructuralOp::AddThruster:
        case StructuralOp::AddCockpit:
        case StructuralOp::AddReactionWheel:
            writeQuat(writer, m_orientation);
            break;
        case StructuralOp::SpawnGrid:
            writer.write(m_position.x);
            writer.write(m_position.y);
            writer.write(m_position.z);
            writer.write(m_cellType);
            writeQuat(writer, m_orientation);
            writeVertices(writer, m_vertices);
            writeColor(writer, m_color);
            break;
        case StructuralOp::SplitGrid:
            writer.write(static_cast<std::uint32_t>(m_pieces.size()));
            for (const GridSplitPiece& piece : m_pieces) {
                writer.write(piece.m_newGridId);
                writer.write(static_cast<std::uint32_t>(piece.m_coords.size()));
                for (const glm::ivec3& coord : piece.m_coords) {
                    writer.write(coord.x);
                    writer.write(coord.y);
                    writer.write(coord.z);
                }
            }
            break;
        case StructuralOp::AddCell:
            writeVertices(writer, m_vertices);
            writeColor(writer, m_color);
            break;
        case StructuralOp::RemoveCell:
        case StructuralOp::DespawnGrid:
            break;
    }
}

bool StructuralCommand::deserialize(ByteReader& reader) {
    if (!(reader.read(m_op) && reader.read(m_gridId) && reader.read(m_coord.x) &&
          reader.read(m_coord.y) && reader.read(m_coord.z))) {
        return false;
    }
    switch (m_op) {
        case StructuralOp::SetColor:
            return reader.read(m_color.x) && reader.read(m_color.y) &&
                   reader.read(m_color.z) && reader.read(m_color.w);
        case StructuralOp::ModifyCell:
            return reader.read(m_cornerIndex) && reader.read(m_direction.x) &&
                   reader.read(m_direction.y) && reader.read(m_direction.z);
        case StructuralOp::AddThruster:
        case StructuralOp::AddCockpit:
        case StructuralOp::AddReactionWheel:
            return readQuat(reader, m_orientation);
        case StructuralOp::SpawnGrid:
            return reader.read(m_position.x) && reader.read(m_position.y) &&
                   reader.read(m_position.z) && reader.read(m_cellType) &&
                   readQuat(reader, m_orientation) && readVertices(reader, m_vertices) &&
                   readColor(reader, m_color);
        case StructuralOp::SplitGrid: {
            std::uint32_t pieceCount{0};
            if (!reader.read(pieceCount)) {
                return false;
            }
            m_pieces.clear();
            for (std::uint32_t ii = 0; ii < pieceCount; ii++) {
                GridSplitPiece piece{};
                std::uint32_t coordCount{0};
                if (!reader.read(piece.m_newGridId) || !reader.read(coordCount)) {
                    return false;
                }
                for (std::uint32_t jj = 0; jj < coordCount; jj++) {
                    glm::ivec3 coord{0};
                    if (!(reader.read(coord.x) && reader.read(coord.y) && reader.read(coord.z))) {
                        return false;
                    }
                    piece.m_coords.push_back(coord);
                }
                m_pieces.push_back(std::move(piece));
            }
            return true;
        }
        case StructuralOp::AddCell:
            return readVertices(reader, m_vertices) && readColor(reader, m_color);
        case StructuralOp::RemoveCell:
        case StructuralOp::DespawnGrid:
            return true;
    }
    return false;  // unknown op
}

StructuralCommand StructuralCommand::setColor(std::uint64_t gridId, const glm::ivec3& coord,
                                              const glm::dvec4& color) {
    return StructuralCommand{StructuralOp::SetColor, gridId, coord, color};
}

StructuralCommand StructuralCommand::addCell(std::uint64_t gridId, const glm::ivec3& coord,
                                             const std::array<glm::ivec3, 8>& vertices,
                                             const glm::dvec4& color) {
    StructuralCommand command{};
    command.m_op = StructuralOp::AddCell;
    command.m_gridId = gridId;
    command.m_coord = coord;
    command.m_vertices = vertices;
    command.m_color = color;
    return command;
}

StructuralCommand StructuralCommand::removeCell(std::uint64_t gridId, const glm::ivec3& coord) {
    return StructuralCommand{StructuralOp::RemoveCell, gridId, coord};
}

StructuralCommand StructuralCommand::modifyCell(std::uint64_t gridId, const glm::ivec3& coord,
                                                int cornerIndex, const glm::ivec3& direction) {
    StructuralCommand command{};
    command.m_op = StructuralOp::ModifyCell;
    command.m_gridId = gridId;
    command.m_coord = coord;
    command.m_cornerIndex = cornerIndex;
    command.m_direction = direction;
    return command;
}

StructuralCommand StructuralCommand::addThruster(std::uint64_t gridId,
                                                 const glm::ivec3& anchorCoord,
                                                 const glm::dquat& orientation) {
    StructuralCommand command{};
    command.m_op = StructuralOp::AddThruster;
    command.m_gridId = gridId;
    command.m_coord = anchorCoord;
    command.m_orientation = orientation;
    return command;
}

StructuralCommand StructuralCommand::addCockpit(std::uint64_t gridId,
                                                const glm::ivec3& anchorCoord,
                                                const glm::dquat& orientation) {
    StructuralCommand command{};
    command.m_op = StructuralOp::AddCockpit;
    command.m_gridId = gridId;
    command.m_coord = anchorCoord;
    command.m_orientation = orientation;
    return command;
}

StructuralCommand StructuralCommand::addReactionWheel(std::uint64_t gridId,
                                                      const glm::ivec3& anchorCoord,
                                                      const glm::dquat& orientation) {
    StructuralCommand command{};
    command.m_op = StructuralOp::AddReactionWheel;
    command.m_gridId = gridId;
    command.m_coord = anchorCoord;
    command.m_orientation = orientation;
    return command;
}

StructuralCommand StructuralCommand::splitGrid(std::uint64_t sourceGridId,
                                               std::vector<GridSplitPiece> pieces) {
    StructuralCommand command{};
    command.m_op = StructuralOp::SplitGrid;
    command.m_gridId = sourceGridId;
    command.m_pieces = std::move(pieces);
    return command;
}

StructuralCommand StructuralCommand::spawnGrid(const glm::dvec3& position, CellType cellType,
                                               const glm::dquat& orientation,
                                               const std::array<glm::ivec3, 8>& vertices,
                                               const glm::dvec4& color) {
    StructuralCommand command{};
    command.m_op = StructuralOp::SpawnGrid;
    command.m_position = position;
    command.m_cellType = cellType;
    command.m_orientation = orientation;
    command.m_vertices = vertices;
    command.m_color = color;
    return command;  // id stays 0: the server allocates it
}

StructuralCommand StructuralCommand::despawnGrid(std::uint64_t gridId) {
    StructuralCommand command{};
    command.m_op = StructuralOp::DespawnGrid;
    command.m_gridId = gridId;
    return command;
}
