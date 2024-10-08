#pragma once

#include <string>

#include "entity.hpp"

class MapNodeVisualizer {
public:
    virtual void push_node(Entity entity) = 0;

    virtual void push_edge(Entity a, Entity b, std::string label, double (*calc_edge_val)(Entity, Entity)) = 0;

    virtual void init() = 0;

    virtual void update() = 0;
};
