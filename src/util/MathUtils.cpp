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

bool MathUtils::isSameDirection(Vector3f target, Vector3f input) {
    const float epsilon = 0.0001;
    return (target - input).norm() < epsilon;
}
