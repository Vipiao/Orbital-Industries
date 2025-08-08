// StructuralBlock.h
#pragma once

#include "GridCell.h"
#include <array>
#include "../utils/PolyhedronProcessor.h"

/**
 * @brief Concrete implementation of GridCell for structural blocks
 * These are the standard building blocks that make up grid structures
 */
class StructuralBlock : public GridCell {
public:
    // Use PolyhedronProcessor's MeshData type
    using MeshData = PolyhedronProcessor::MeshData;
    // Type identifier for this block type
    static constexpr CellType TYPE = CellType::STRUCTURAL_BLOCK;
    
    StructuralBlock(const glm::ivec3& coords);
    virtual ~StructuralBlock() = default;

    // Shape definition - 8 corner vertices in local integer coordinates
    std::array<glm::ivec3, 8> m_localVertices;
    
    // Scaling factor for local coordinates
    int m_maxSize = 4;

    /**
     * @brief Get collision axes for current shape using PolyhedronProcessor
     * @return AxisResult containing face and edge axes
     */
    PolyhedronProcessor::AxisResult getAxes() const;
    
    /**
     * @brief Set new vertex configuration and validate the shape
     * @param newVertices New 8-vertex configuration
     * @return true if the new shape is valid and was set, false otherwise
     */
    bool setVertices(const std::array<glm::ivec3, 8>& newVertices);
    
    /**
     * @brief Validate if vertices would create a valid polyhedron shape
     * @param vertices 8-vertex configuration to validate
     * @param maxSize Maximum coordinate value for normalization
     * @return true if the vertices form a valid polyhedron, false otherwise
     */
    static bool validateVertices(const std::array<glm::ivec3, 8>& vertices, int maxSize = 4);
    
    /**
     * @brief Get normalized vertices in local coordinates
     * @return Vector of vertices in local coordinates (normalized by m_maxSize)
     */
    std::vector<glm::dvec3> getVertices() const;
    
    /**
     * @brief Generate complete triangle mesh data with normals, tangents, and UVs
     * @return MeshData containing all vertex attributes for rendering
     */
    MeshData generateTriangleMeshData() const;
};