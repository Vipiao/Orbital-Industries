// Edge.cpp
#include "Edge.h"
#include "Collider.h"

Edge::Edge(Collider* collider, EdgeType type, EdgeAxis axis)
    : collider(collider)
    , type(type)
    , axis(axis)
    , value(0.0)
{
}

void Edge::updateValue() {
    if (!collider) return;
    
    switch (axis) {
        case EdgeAxis::X:
            value = (type == EdgeType::MIN) ? collider->m_AABBMin.x : collider->m_AABBMax.x;
            break;
        case EdgeAxis::Y:
            value = (type == EdgeType::MIN) ? collider->m_AABBMin.y : collider->m_AABBMax.y;
            break;
        case EdgeAxis::Z:
            value = (type == EdgeType::MIN) ? collider->m_AABBMin.z : collider->m_AABBMax.z;
            break;
    }
}