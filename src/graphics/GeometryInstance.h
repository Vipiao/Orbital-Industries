// src/graphics/GeometryInstance.h
#pragma once

#include <glm/glm.hpp>
#include <cstddef>

// Forward declaration
class GeometryData;

struct InstanceTransform {
    glm::vec2 position;
    glm::vec2 scale;
    float orientation; // radians
    // Padding to align to 32 bytes for GPU.
    // Not nessesary but slight performance boost in the input assembler
    // on some hardware.
    float padding[3];
};

class GeometryInstance {
public:
    GeometryInstance(GeometryData* parent, size_t index);
    ~GeometryInstance() = default;
    
    // Property accessors with automatic GPU sync
    void setPosition(const glm::vec2& pos);
    void setScale(const glm::vec2& scale);
    void setOrientation(float radians);
    
    glm::vec2 getPosition() const { return m_transform.position; }
    glm::vec2 getScale() const { return m_transform.scale; }
    float getOrientation() const { return m_transform.orientation; }
    
    // Direct access to transform for batch updates
    InstanceTransform& getTransform() { return m_transform; }
    const InstanceTransform& getTransform() const { return m_transform; }
    
    // Internal management
    size_t getIndex() const { return m_index; }
    void setIndex(size_t index) { m_index = index; }
    GeometryData* getParent() const { return m_parent; }
    
    // Batch update - call this after modifying transform directly
    void syncToGPU();

private:
    InstanceTransform m_transform;
    size_t m_index;
    GeometryData* m_parent;
    
    void updateParent();
};