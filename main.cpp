#include "src/entity.hpp"
#include "src/visualizations/map_node_visualizer.hpp"
#include "src/visualizations/sfml_map_node_visualizer.hpp"

#include <vector>
#include <iostream>

int main() {
    std::vector<Entity> entities;

    entities.push_back(Entity(39.75278539999197, -105.227871868156));
    entities.push_back(Entity(39.7529109775134, -105.2273650800339));
    entities.push_back(Entity(39.75278539999197, -105.227871868156));

    MapNodeVisualizer* visualizer = new SFMLMapNodeVisualizer();
    visualizer->init();

    for (auto entity : entities) {
        visualizer->push_node(entity);
    }

    while (true) {
        visualizer->update();
    }

    return 0;
}