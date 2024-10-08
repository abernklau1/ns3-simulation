#include <SFML/System/Thread.hpp>
#include <SFML/Window/ContextSettings.hpp>
#include <iostream>
#include <thread>
#include <utility>
#include <memory>

#include "sfml_map_node_visualizer.hpp"

SFMLMapNodeVisualizer::SFMLMapNodeVisualizer() {
    center_long_lat = std::make_pair(39.75278539999197, -105.227871868156);

    // Initialize the zoom/pan matrix
    zoom_pan_matrix = Matrix3d(
        1, 0, 0,
        0, 1, 0,
        0, 0, 1
    );
}

void SFMLMapNodeVisualizer::push_node(Entity entity) {
    this->entities.push_back(entity);
}

void SFMLMapNodeVisualizer::push_edge(Entity a, Entity b, std::string label, double (*calc_edge_val)(Entity, Entity)) {
    this->edges.push_back({a, b, label, calc_edge_val});
}

void SFMLMapNodeVisualizer::init() {
    sf::ContextSettings settings(0, 0, 16);

    window.create(sf::VideoMode(1920, 1080), "SFML Map Node Visualizer", sf::Style::HighDPI, settings);
    window.setVerticalSyncEnabled(true);

    window.setActive(false);
}

void SFMLMapNodeVisualizer::update() {
    if (!window.isOpen()) {
        return;
    }

    sf::Event event{};
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
            return;
        }
    }

    // Clear the window
    window.clear(sf::Color(240, 240, 240));

    // Draw the nodes and edges
    for (auto node : entities) {
        sf::CircleShape circle(5);
        
        circle.setFillColor(sf::Color::Black);
        auto xy_dist_from_center = node.distanceVectorMeters(
            center_long_lat.first, center_long_lat.second
        );

        // Apply the zoom/pan matrix
        Vector3d vector_from_center = zoom_pan_matrix * Vector3d(
            xy_dist_from_center.first,
            xy_dist_from_center.second,
            1
        );
        
        circle.setPosition(
            vector_from_center[0],
            vector_from_center[1]
        );

        window.draw(circle);
    }

    // Display the window
    window.display();
}

SFMLMapNodeVisualizer::~SFMLMapNodeVisualizer() {
    window.close();
}