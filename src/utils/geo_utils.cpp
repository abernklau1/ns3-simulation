#include "geo_utils.hpp"

double distance_meters_between_coords(double lon1, double lat1, double lon2, double lat2) {
    double earthRadius = 6371000; // meters

    // Convert to radians and apply the haversine formula
    double dLat = (lat2 - lat1) * M_PI / 180;
    double dLon = (lon2 - lon1) * M_PI / 180;

    double a = sin(dLat / 2) * sin(dLat / 2) +
                cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) *
                sin(dLon / 2) * sin(dLon / 2);
    
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return earthRadius * c;
}
