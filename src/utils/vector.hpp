#pragma once

class Vector3d {
private:
    double data[3];
public:
    Vector3d() {
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
    }

    Vector3d(double x, double y, double z) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }

    double operator[](int i) {
        return data[i];
    }

    Vector3d operator+(Vector3d&& other) {
        return Vector3d(
            data[0] + other.data[0],
            data[1] + other.data[1],
            data[2] + other.data[2]
        );
    }
};
