#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "Eigen/Dense"

class MathUtils
{
public:
    MathUtils();

    /** Generates a uniform random number between [0, 1) */
    static float random();

    static Eigen::Vector3f reflect(Eigen::Vector3f in, Eigen::Vector3f normal);

    static Eigen::Vector3f refract(Eigen::Vector3f in, Eigen::Vector3f normal, float eta);

    static bool isSameDirection(Eigen::Vector3f target, Eigen::Vector3f input);
};

#endif // MATHUTILS_H
