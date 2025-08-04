// StructuralBlock.cpp
#include "StructuralBlock.h"

StructuralBlock::StructuralBlock(const glm::ivec3& coords)
    : GridCell(coords, TYPE) 
{
    // Initialize as standard cube in PolyhedronProcessor vertex order
    // Coordinates from (0,0,0) to (maxSize,maxSize,maxSize)
    m_localVertices = {{
        {0, 0, 0},           // 0: bottom-back-left
        {m_maxSize, 0, 0},   // 1: bottom-back-right  
        {m_maxSize, m_maxSize, 0}, // 2: bottom-front-right
        {0, m_maxSize, 0},   // 3: bottom-front-left
        {0, 0, m_maxSize},   // 4: top-back-left
        {m_maxSize, 0, m_maxSize},   // 5: top-back-right
        {m_maxSize, m_maxSize, m_maxSize}, // 6: top-front-right  
        {0, m_maxSize, m_maxSize}    // 7: top-front-left
    }};
}