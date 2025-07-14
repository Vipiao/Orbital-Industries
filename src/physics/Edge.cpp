// Edge.cpp
#include "Edge.h"
#include "Collider.h"

Edge::Edge(Collider* collider, EdgeType type, EdgeAxis axis)
    : m_collider(collider)
    , m_type(type)
    , m_axis(axis)
    , m_value(0.0)
{
}

void Edge::updateValue() {
    if (!m_collider) return;
    
    switch (m_axis) {
        case EdgeAxis::X:
            m_value = (m_type == EdgeType::MIN) ? m_collider->m_AABBMin.x : m_collider->m_AABBMax.x;
            break;
        case EdgeAxis::Y:
            m_value = (m_type == EdgeType::MIN) ? m_collider->m_AABBMin.y : m_collider->m_AABBMax.y;
            break;
        case EdgeAxis::Z:
            m_value = (m_type == EdgeType::MIN) ? m_collider->m_AABBMin.z : m_collider->m_AABBMax.z;
            break;
    }
}