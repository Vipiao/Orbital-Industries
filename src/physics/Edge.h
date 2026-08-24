// Edge.h
#pragma once

class Collider;

enum class EdgeType {
    MIN,
    MAX
};

enum class EdgeAxis {
    X,
    Y,
    Z
};

class Edge {
public:
    Edge(Collider* collider, EdgeType type, EdgeAxis axis);
    
    Collider* m_collider;
    EdgeType m_type;
    EdgeAxis m_axis;
    double m_value;  // The position value along the axis
    
    // Update the edge value based on collider's collision box
    void updateValue();
};