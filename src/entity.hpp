#pragma once

#include <cmath>

class Entity {
private:
    double latitude;
    double longitude;

public:
    Entity(double latitude, double longitude);

    double distanceMeters(Entity& other);

    std::pair<double, double> distanceVectorMeters(double lat, double lon);
};
