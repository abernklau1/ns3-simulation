#pragma once

#include "map_node_visualizer.hpp"
#include "SFML/Graphics.hpp"
#include "utils/matrix.hpp"

#include <thread>

class SFMLMapNodeVisualizer : public MapNodeVisualizer {
private:
    sf::RenderWindow window;

    std::vector<Entity> entities;
    std::vector<std::tuple<Entity, Entity, std::string, double (*)(Entity, Entity)>> edges;

    std::pair<double, double> center_long_lat;

    // Zoom/pan matrix
    Matrix3d zoom_pan_matrix;

public:
    SFMLMapNodeVisualizer();

    void push_node(Entity entity) override;

    void push_edge(Entity a, Entity b, std::string label, double (*calc_edge_val)(Entity, Entity)) override;

    void init() override;

    void update() override;

    ~SFMLMapNodeVisualizer();
};