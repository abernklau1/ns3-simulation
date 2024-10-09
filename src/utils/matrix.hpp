#pragma once

#include <cmath>

#include "vector.hpp"

class Matrix3d {
private:
    double data[3][3];

public:
    Matrix3d() {
        data[0][0] = 0;
        data[0][1] = 0;
        data[0][2] = 0;

        data[1][0] = 0;
        data[1][1] = 0;
        data[1][2] = 0;

        data[2][0] = 0;
        data[2][1] = 0;
        data[2][2] = 0;
    }

    Matrix3d(double xx, double xy, double xz,
             double yx, double yy, double yz,
             double zx, double zy, double zz) {
        data[0][0] = xx;
        data[0][1] = xy;
        data[0][2] = xz;

        data[1][0] = yx;
        data[1][1] = yy;
        data[1][2] = yz;

        data[2][0] = zx;
        data[2][1] = zy;
        data[2][2] = zz;
    }

    double operator()(int i, int j) {
        return data[i][j];
    }

    Matrix3d operator+(Matrix3d& matrix) {
        return Matrix3d(
            data[0][0] + matrix.data[0][0],
            data[0][1] + matrix.data[0][1],
            data[0][2] + matrix.data[0][2],

            data[1][0] + matrix.data[1][0],
            data[1][1] + matrix.data[1][1],
            data[1][2] + matrix.data[1][2],

            data[2][0] + matrix.data[2][0],
            data[2][1] + matrix.data[2][1],
            data[2][2] + matrix.data[2][2]
        );
    }

    Matrix3d operator*(Matrix3d&& other) {
        // Fun matrix multiplication
        return Matrix3d(
            data[0][0] * other.data[0][0] + data[0][1] * other.data[1][0] + data[0][2] * other.data[2][0],
            data[0][0] * other.data[0][1] + data[0][1] * other.data[1][1] + data[0][2] * other.data[2][1],
            data[0][0] * other.data[0][2] + data[0][1] * other.data[1][2] + data[0][2] * other.data[2][2],

            data[1][0] * other.data[0][0] + data[1][1] * other.data[1][0] + data[1][2] * other.data[2][0],
            data[1][0] * other.data[0][1] + data[1][1] * other.data[1][1] + data[1][2] * other.data[2][1],
            data[1][0] * other.data[0][2] + data[1][1] * other.data[1][2] + data[1][2] * other.data[2][2],

            data[2][0] * other.data[0][0] + data[2][1] * other.data[1][0] + data[2][2] * other.data[2][0],
            data[2][0] * other.data[0][1] + data[2][1] * other.data[1][1] + data[2][2] * other.data[2][1],
            data[2][0] * other.data[0][2] + data[2][1] * other.data[1][2] + data[2][2] * other.data[2][2]
        );
    }

    Matrix3d operator*(double scalar) {
        return Matrix3d(
            data[0][0] * scalar,
            data[0][1] * scalar,
            data[0][2] * scalar,

            data[1][0] * scalar,
            data[1][1] * scalar,
            data[1][2] * scalar,

            data[2][0] * scalar,
            data[2][1] * scalar,
            data[2][2] * scalar
        );
    }

    Vector3d operator*(Vector3d&& vector) {
        return Vector3d(
            data[0][0] * vector[0] + data[0][1] * vector[1] + data[0][2] * vector[2],
            data[1][0] * vector[0] + data[1][1] * vector[1] + data[1][2] * vector[2],
            data[2][0] * vector[0] + data[2][1] * vector[1] + data[2][2] * vector[2]
        );
    }
};