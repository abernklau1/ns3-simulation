#pragma once

#include "entity.hpp"
#include "networking/network_metrics.hpp"

class NetworkSim {
public:
    NetworkSim();

    void add_node(Entity entity);

    double get_metric(Metric metric, Entity& a, Entity& b);
};