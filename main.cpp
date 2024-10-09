#include <vector>

#include "networking/network_sim.hpp"
#include "src/entity.hpp"

int main() {
    std::vector<Entity> entities;
    entities.push_back(Entity(39.75278539999197, -105.227871868156));
    entities.push_back(Entity(39.7529109775134, -105.2273650800339));
    entities.push_back(Entity(39.75278539999197, -105.227871868156));

    NetworkSim networkSim;

    for (auto entity : entities) {
        networkSim.add_node(entity);
    }

    networkSim.start_sim();

    return 0;
}