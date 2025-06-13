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
            value = (type == EdgeType::MIN) ? collider->collisionBoxMin.x : collider->collisionBoxMax.x;
            break;
        case EdgeAxis::Y:
            value = (type == EdgeType::MIN) ? collider->collisionBoxMin.y : collider->collisionBoxMax.y;
            break;
        case EdgeAxis::Z:
            value = (type == EdgeType::MIN) ? collider->collisionBoxMin.z : collider->collisionBoxMax.z;
            break;
    }
}