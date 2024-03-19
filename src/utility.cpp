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

void seperateVector(std::vector<int> & vector_target, std::vector<int> vector_seperate) {
    int index_seperate = 0;
    for (int index = 0; index < vector_target.size(); index++) {
        if (vector_target[index] == vector_seperate[index_seperate]) {
            vector_target.erase(vector_target.begin() + index);
            index--;
            index_seperate++;
        }
    }
}

void seperateVector(std::vector<int> vector_src, std::vector<int> vector_seperate, std::vector<int> & vector_target) {
    int index_seperate = 0;
    for (int index = 0; index < vector_src.size(); index++) {
        if (index != vector_seperate[index_seperate]) {
            vector_target.emplace_back(index);
        }
        else {
            index_seperate++;
        }
    }
}

void seperateVector(int size, std::vector<int> vector_seperate, std::vector<int> & vector_target) {
    int index_seperate = 0;
    for (int index = 0; index < size; index++) {
        if (index != vector_seperate[index_seperate]) {
            vector_target.emplace_back(index);
        }
        else {
            index_seperate++;
        }
    }
}
