#include "bsdf.h"

#include <iostream>
#include "util/MathUtils.h"
#include "SampleRay.h"

using namespace Eigen;

BSDF::BSDF()
{
}

Vector3f BSDF::getBsdfFromType(Ray incoming_ray, Vector3f out, Vector3f normal,
                               const tinyobj::material_t& mat, MaterialType type) {
    switch (type) {
    case IDEAL_DIFFUSE:
        return idealDiffuseBsdf(mat);
    case IDEAL_SPECULAR:
        return idealSpecularBsdf(incoming_ray, out, normal);
    case GLOSSY_SPECULAR:
        return glossySpecularBsdf(incoming_ray, out, normal, mat);
    case REFRACTION:
        return refractionBsdf(incoming_ray, out, normal, mat);
    default:
        std::cout << type << std::endl;
        std::cerr << "(BRDF) Unsupported Material Type"  << std::endl;
        exit(1);
    }
}

MaterialType BSDF::getType(const tinyobj::material_t& mat) {
    Vector3f specular(mat.specular[0], mat.specular[1], mat.specular[2]);
    Vector3f transmittance(mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]);
    Vector3f emission(mat.emission[0], mat.emission[1], mat.emission[2]);
    if (emission.norm() > .1) {
        return LIGHT;
    } else if (transmittance.norm() > 0.1) {
        return REFRACTION;
    } else if (specular.norm() < DIFFUSE_CUTOFF) {
        return IDEAL_DIFFUSE;
    } else if (mat.shininess < SPECULAR_CUTOFF) {
        return GLOSSY_SPECULAR;
    } else {
        return IDEAL_SPECULAR;
    }
}

Vector3f BSDF::glossySpecularBsdf(Ray incoming_ray, Vector3f out, Vector3f normal, const tinyobj::material_t& mat) {
    Vector3f specular(mat.specular[0], mat.specular[1], mat.specular[2]);
    float shininess = mat.shininess;

    Vector3f ideal_reflection = MathUtils::reflect(incoming_ray.d, normal);
    return specular * (shininess + 2) / (2 * M_PI) * pow(ideal_reflection.dot(out), shininess);
}

Vector3f BSDF::idealDiffuseBsdf(const tinyobj::material_t& mat) {
    Vector3f diffuse(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
    return diffuse / M_PI;
}

Vector3f BSDF::idealSpecularBsdf(Ray incoming_ray, Vector3f out, Vector3f normal) {
    Vector3f N = incoming_ray.is_in_air ? normal : -normal;

    Vector3f target = MathUtils::reflect(incoming_ray.d, N);
    if (MathUtils::isSameDirection(target, out) == false) {
        return Vector3f(0, 0, 0); // Incorrect out vector has 0 weight
    }

    Vector3f full_reflection(1.f, 1.f, 1.f); // Assume perfect reflectance and no absorbtion.
    return full_reflection / out.dot(N);
}

Vector3f BSDF::refractionBsdf(Ray incoming_ray, Vector3f out, Vector3f normal, const tinyobj::material_t& mat) {
    Vector3f target = SampleRay::refractionDirection(incoming_ray, normal, mat);
    if (MathUtils::isSameDirection(target, out) == false) {
        return Vector3f(0, 0, 0); // Incorrect out vector has 0 weight
    }

    Vector3f N = incoming_ray.is_in_air ? normal : -normal;

    Vector3f full_refraction(1.f, 1.f, 1.f); // Assume perfect refraction and no absorbtion.
    return full_refraction / out.dot(N);
}

float BSDF::getBsdfDirectionalProb(const Vector3f &incoming, const Vector3f &outgoing, Vector3f normal, const tinyobj::material_t& mat, MaterialType type, float eta) {
    switch (type) {
    case IDEAL_DIFFUSE:
        return cosineWeightedProb(outgoing, normal);
    case GLOSSY_SPECULAR:
        return glossyWeightedProb(incoming, outgoing, normal, mat);
    case IDEAL_SPECULAR:
        return idealSpecularProb(incoming, outgoing, normal);
    case REFRACTION:
        return idealRefractionProb(incoming, outgoing, normal, eta);
    default:
        std::cerr << "(BRDF) Unsupported Material Type"  << std::endl;
        exit(1);
    }
}

//check this for probability
float BSDF::cosineWeightedProb(const Vector3f &outgoing, Vector3f normal) {
    return fabsf(outgoing.dot(normal) / M_PI);
}

float BSDF::glossyWeightedProb(const Vector3f &incoming, const Vector3f &outgoing, Vector3f normal, const tinyobj::material_t &mat) {
    Vector3f ideal_reflection = MathUtils::reflect(incoming, normal);
    float cos_alpha = ideal_reflection.dot(outgoing);
    return fmax(0, (mat.shininess + 1) * powf(cos_alpha, mat.shininess) / (2 * M_PI));
}

float BSDF::idealSpecularProb(const Vector3f &incoming, const Vector3f &outgoing, Vector3f normal) {
    Vector3f ideal_reflection = MathUtils::reflect(incoming, normal);
    if (MathUtils::isSameDirection(ideal_reflection, outgoing)) {
        return 1.f;
    }
    return 0.f;
}

float BSDF::idealRefractionProb(const Vector3f &incoming, const Vector3f &outgoing, Vector3f normal, float eta) {
    Vector3f ideal_refraction = MathUtils::refract(incoming, normal, eta);
    if (MathUtils::isSameDirection(ideal_refraction, outgoing)) {
        return 1.f;
    }
    return 0.f;
}
