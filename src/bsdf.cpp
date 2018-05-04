#include "bsdf.h"

#include <iostream>
#include "util/MathUtils.h"
#include "SampleRay.h"

using namespace Eigen;

BSDF::BSDF()
{
}

Vector3f BSDF::getBsdfFromType(Ray incoming_ray, Vector3f &position, Ray outgoing_ray, Vector3f normal,
                               const tinyobj::material_t& mat, MaterialType type) {
    switch (type) {
    case IDEAL_DIFFUSE:
        return idealDiffuseBsdf(mat);
    case IDEAL_SPECULAR:
        return idealSpecularBsdf(incoming_ray, outgoing_ray.d, normal);
    case GLOSSY_SPECULAR:
        return glossySpecularBsdf(incoming_ray, outgoing_ray.d, normal, mat);
    case REFRACTION:
        return refractionBsdf(incoming_ray, outgoing_ray.d, normal, mat);
    case DIFFUSE_SCATTERING:
        return diffuseScatteringBssrdf(incoming_ray, position, outgoing_ray, normal, mat);
    case SINGLE_SCATTERING:
        return singleScatteringBssrdf(incoming_ray, position, outgoing_ray, normal, mat);
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
    Vector3f scattering(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
    if (emission.norm() > 0.1) {
        return LIGHT;
    } else if (transmittance.norm() > 0.01) {
        return DIFFUSE_SCATTERING;
        if (MathUtils::random() < 0.98) {
            return DIFFUSE_SCATTERING;
        } else {
            return SINGLE_SCATTERING;
        }
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

float BSDF::fdr(float n) {
    if (n < 1) {
        return -0.4399 + (0.7099/n) - (0.3319/pow(n, 2)) + (0.0636/pow(n, 3));
    } else {
        return -(1.4399/pow(n, 2)) + (0.7099/n) + 0.6681 + 0.0636 * n;
    }
}

Vector3f BSDF::singleScatteringBssrdf(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray,
                      Vector3f normal, const tinyobj::material_t& mat) {
    outgoing_ray.d = -1.f * outgoing_ray.d;
    float n = 1.3;

    Vector3f wp_o = SampleRay::refractionDirection(outgoing_ray, normal, mat);
    Vector3f wp_i = SampleRay::refractionDirection(incoming_ray, normal, mat);

    Vector3f single_scat;
    int num_samples = 10;
    for (int i = 0; i < 3; i++) {
        float total = 0.f;
        for (int j = 0; j < num_samples; j++) {
            float sig_t = mat.transmittance[i] + mat.ambient[i];
            float s_o = 5.f * -std::log(MathUtils::random())/sig_t;

            Vector3f p = outgoing_ray.o + (s_o * wp_o);
            float s_i = (p - position).norm();

            float dot_wn = (-incoming_ray.d).dot(normal);
            float s_i_prime = (s_i * dot_wn)/std::sqrt(1.f - pow(1.f/n, 2) * (1.f - pow(dot_wn, 2)));

            float ft_i = fresnel(AIR_IOR, 1.3, normal.dot(-incoming_ray.d));
            float ft_o = fresnel(AIR_IOR, 1.3, normal.dot(outgoing_ray.d));
            float G = std::abs(normal.dot(wp_o)/normal.dot(wp_i));
            float sig_tc = sig_t + G * sig_t;

            float res = (mat.transmittance[i] * (1.f/(4.f*M_PI)))/sig_tc;
            res *= pow(M_E, -std::abs(s_i_prime) * sig_t);
            res *= pow(M_E, -s_o * sig_t) * ft_i * ft_o;

            total += res;
        }

        single_scat[i] = total/float(num_samples);
    }

    outgoing_ray.d = -1.f * outgoing_ray.d;
    return single_scat;
}

float BSDF::fresnel(float n1, float n2, float cosi) {
    if (cosi < -1.f) {
        cosi = -1.f;
    } else if (cosi > 1.f) {
        cosi = 1.f;
    }

    float sint = (n1/n2) * std::sqrt(std::max(0.0, 1.0 - pow(cosi, 2)));

    if (sint > 1.f) {
        return 0.f;
    }

    float cost = std::sqrt(std::max(0.0, 1.0 - pow(sint, 2)));
    cosi = std::abs(cosi);

    float F1_R = pow((n2 * cosi - n1 * cost)/(n2 * cosi + n1 * cost), 2);
    float F2_R = pow((n1 * cosi - n2 * cost)/(n1 * cosi + n2 * cost), 2);

    float F_R = (F1_R + F2_R)/2.f;

    return 1.f - F_R;
}

Vector3f BSDF::diffuseScatteringBssrdf(Ray incoming_ray, Vector3f &position, Ray outgoing_ray,
                                       Vector3f normal, const tinyobj::material_t &mat) {
    float r = (position - outgoing_ray.o).norm();
    Vector3f R = reflectance(r, mat);

    float ft_i = fresnel(AIR_IOR, 1.3, normal.dot(-incoming_ray.d));
    float ft_o = fresnel(AIR_IOR, 1.3, normal.dot(outgoing_ray.d));

    for (int i = 0; i < 3; i++) {
        R[i] *= mat.diff[i];
    }

    return ft_i * R * ft_o;
}

Vector3f BSDF::reflectance(float r, const tinyobj::material_t &mat) {
    Vector3f R;

    for (int i = 0; i < 3; i++) {
        float sig_s = mat.transmittance[i];
        float sig_a = mat.ambient[i];
        float sig_t = sig_s + sig_a;
        float sig_tr = std::sqrt(3.f * sig_a * sig_t);

        float alpha = sig_s/sig_t;

        float F_dr = fdr(1.3);
        float A = (1.f + F_dr)/(1.f - F_dr);
        float D = 1.f/(3.f * sig_t);

        float z_v = 1.f/sig_t;
        float z_r = z_v + 4 * A * D;

        float d_r = std::sqrt(pow(z_r, 2) + pow(r, 2));
        float d_v = std::sqrt(pow(z_v, 2) + pow(r, 2));

        float r_r = z_r * (sig_tr * d_r + 1.f) * (pow(M_E, -sig_tr * d_r)/pow(d_r, 3));
        float r_v = z_v * (sig_tr * d_v + 1.f) * (pow(M_E, -sig_tr * d_v)/pow(d_v, 3));

        R[i] = alpha/(4.f * M_PI) * (r_r + r_v);
    }

    return R;
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

Vector3f BSDF::refractionBsdf(Ray incoming_ray, Vector3f out, Vector3f normal,
                              const tinyobj::material_t& mat) {
    Vector3f target = SampleRay::refractionDirection(incoming_ray, normal, mat);
    if (MathUtils::isSameDirection(target, out) == false) {
        return Vector3f(0, 0, 0); // Incorrect out vector has 0 weight
    }

    Vector3f N = incoming_ray.is_in_air ? normal : -normal;

    Vector3f full_refraction(1.f, 1.f, 1.f); // Assume perfect refraction and no absorbtion.
    return full_refraction / out.dot(N);
}

float BSDF::getBsdfDirectionalProb(const Vector3f &incoming, const Vector3f &outgoing,
                                   Vector3f normal, float radius, const tinyobj::material_t& mat,
                                   MaterialType type, float eta) {
    switch (type) {
    case DIFFUSE_SCATTERING:
    case SINGLE_SCATTERING:
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

float BSDF::cosineWeightedProb(const Vector3f &outgoing, Vector3f normal) {
    return fabsf(outgoing.dot(normal) / M_PI);
}

float BSDF::glossyWeightedProb(const Vector3f &incoming, const Vector3f &outgoing,
                               Vector3f normal, const tinyobj::material_t &mat) {
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
