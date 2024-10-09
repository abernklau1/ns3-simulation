#include "entity.hpp"
#include "utils/geo_utils.hpp"
#include <utility>

Entity::Entity(double longitude, double latitude) {
    this->latitude = latitude;
    this->longitude = longitude;
}

double Entity::distanceMeters(Entity &other) {
    return distance_meters_between_coords(this->latitude, this->longitude, other.latitude, other.longitude);
}

std::pair<double, double> Entity::distanceVectorMeters(double lon, double lat) {
    // Return the distance in meters between the two points in the x and y directions
    return std::make_pair(
        distance_meters_between_coords(this->longitude, this->latitude, this->longitude, lat),
        distance_meters_between_coords(this->longitude, this->latitude, lon, this->latitude)
    );
}