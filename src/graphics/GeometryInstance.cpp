// src/graphics/GeometryInstance.cpp
#include "GeometryInstance.h"
#include "GeometryData.h"

GeometryInstance::GeometryInstance(GeometryData* parent, size_t index)
    : m_parent(parent), m_index(index) {
    // Initialize with default transform
    m_transform.position = glm::vec2(0.0f);
    m_transform.scale = glm::vec2(1.0f);
    m_transform.orientation = 0.0f;
    m_transform.padding[0] = 0.0f;
    m_transform.padding[1] = 0.0f;
    m_transform.padding[2] = 0.0f;
}

void GeometryInstance::setPosition(const glm::vec2& pos) {
    m_transform.position = pos;
    updateParent();
}

void GeometryInstance::setScale(const glm::vec2& scale) {
    m_transform.scale = scale;
    updateParent();
}

void GeometryInstance::setOrientation(float radians) {
    m_transform.orientation = radians;
    updateParent();
}

void GeometryInstance::syncToGPU() {
    updateParent();
}

void GeometryInstance::updateParent() {
    if (m_parent) {
        m_parent->updateInstanceTransform(m_index, m_transform);
    }
}