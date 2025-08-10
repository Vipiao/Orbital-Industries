// SSBOManager.cpp
#include "SSBOManager.h"
#include <stdexcept>
#include <iostream>

SSBOManager::SSBOManager(size_t maxEntries) 
    : m_maxEntries(maxEntries), m_nextNewIndex(0) {
    
    // Create and initialize SSBO
    glGenBuffers(1, &m_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(MeshData) * m_maxEntries, nullptr, GL_DYNAMIC_DRAW);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, m_ssbo);
    
    std::cout << "SSBOManager: Created SSBO with capacity for " << m_maxEntries << " entries" << std::endl;
}

SSBOManager::~SSBOManager() {
    if (m_ssbo != 0) {
        glDeleteBuffers(1, &m_ssbo);
        std::cout << "SSBOManager: Deleted SSBO" << std::endl;
    }
}

int SSBOManager::allocateIndex() {
    // First try to reuse a deallocated index
    if (!m_availableIndices.empty()) {
        int index = m_availableIndices.back();
        m_availableIndices.pop_back();
        return index;
    }
    
    // No available indices, allocate a new one
    if (m_nextNewIndex >= m_maxEntries) {
        throw std::runtime_error("SSBOManager: SSBO is full, cannot allocate more indices");
    }
    
    int newIndex = static_cast<int>(m_nextNewIndex);
    m_nextNewIndex++;
    return newIndex;
}

void SSBOManager::deallocateIndex(int index) {
    if (index < 0 || static_cast<size_t>(index) >= m_maxEntries) {
        std::cerr << "SSBOManager: Warning - invalid index " << index << " in deallocateIndex" << std::endl;
        return;
    }
    
    // Check if already in available list to prevent duplicates
    for (int availableIndex : m_availableIndices) {
        if (availableIndex == index) {
            std::cerr << "SSBOManager: Warning - index " << index << " already deallocated" << std::endl;
            return;
        }
    }
    
    m_availableIndices.push_back(index);
}

void SSBOManager::updateData(int index, const MeshData& data) {
    if (index < 0 || static_cast<size_t>(index) >= m_maxEntries) {
        std::cerr << "SSBOManager: Warning - invalid index " << index << " in updateData" << std::endl;
        return;
    }
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_ssbo);
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, sizeof(MeshData) * index, sizeof(MeshData), &data);
}