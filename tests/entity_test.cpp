#include <catch2/catch_test_macros.hpp>

#include "src/entity.hpp"
#include <iostream>

const double LAT1 = 39.75278539999197;
const double LON1 = -105.227871868156;

const double LAT2 = 39.7529109775134;
const double LON2 = -105.2273650800339;

const double ACTUAL_DIST_METERS = 45.62;

const double DIST_EPSILON = 0.25;

TEST_CASE("Distance between two points is calculated correctly", "[single-file]") {
    Entity a = Entity(LAT1, LON1);
    Entity b = Entity(LAT2, LON2);

    REQUIRE(abs(a.distanceMeters(b) - ACTUAL_DIST_METERS) < DIST_EPSILON);
}