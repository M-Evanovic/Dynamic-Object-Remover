#include "remover/utility.h"

std::vector<double> split_line(std::string line, char delimiter) {
    std::vector<double> output;
    std::stringstream ss(line);
    std::string temp;

    while (getline(ss, temp, delimiter)) {
        output.emplace_back(stof(temp));
    }
    return output;
}

SphericalPoint Cartesian2Spherical(PointType point) {
    SphericalPoint sphericalPoint {
        std::atan2(point.y, point.x),
        std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y)),
        std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z)
    };
    return sphericalPoint;
}

float radian2degree(float radian) {
    return radian * 180.0 / M_PI;
}

float degree2radian(float degree) {
    return degree * M_PI / 180.0;
}

