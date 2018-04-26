#include "MathUtils.h"

#include <stdlib.h>

using namespace Eigen;

MathUtils::MathUtils()
{
}

float MathUtils::random() {
    return rand() / static_cast<float>(RAND_MAX);
}

Vector3f MathUtils::reflect(Vector3f in, Vector3f normal) {
    return (in - 2.f * normal.dot(in) * normal).normalized();
}

Vector3f MathUtils::refract(Vector3f in, Vector3f normal, float eta) {
    Vector3f bounce = in;
    float k = 1.f - powf(eta, 2) * (1.f - powf(normal.dot(bounce), 2)); //k is the square of the cosine of theta transmitted
    if (k < 0.f) {
        return Vector3f(0, 0, 0);
    }
    float dot = normal.dot(bounce);
    Vector3f refracted = (eta * bounce - (eta * dot + std::sqrt(k)) * normal).normalized(); //minus or add
    return refracted;
}

bool MathUtils::isSameDirection(Vector3f target, Vector3f input) {
    const float epsilon = 0.01;
    return (target - input).norm() < epsilon;
}
