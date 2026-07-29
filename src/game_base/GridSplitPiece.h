// GridSplitPiece.h — one piece produced by a grid split: the id it gets and the
// cells that move into it. The list of pieces fully describes a split's outcome, so
// it can be applied identically wherever it is needed (locally, or from a decoded
// command).
#pragma once

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

struct GridSplitPiece {
    std::uint64_t m_newGridId{0};
    std::vector<glm::ivec3> m_coords{};
};

// A completed split: the source grid and every piece that broke off it.
struct GridSplitResult {
    std::uint64_t m_sourceGridId{0};
    std::vector<GridSplitPiece> m_pieces{};
};
