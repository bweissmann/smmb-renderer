#include "bsdf.h"

#include <algorithm>
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
    case SCATTERING:
//        std::cout << "rays: " << std::endl;
//        std::cout << position << std::endl;
//        std::cout << outgoing_ray.o << std::endl;
//        std::cout << (outgoing_ray.o - position).norm() << std::endl;
        return 0.5 * bssrdf(incoming_ray, position, outgoing_ray, normal, mat)
                + 0.5 * bssrdf1(incoming_ray, position, outgoing_ray, normal, mat);
//        return bssrdf(incoming_ray, position, outgoing_ray, normal, mat);
//        return idealDiffuseBsdf(mat);
    case IDEAL_DIFFUSE:
        return idealDiffuseBsdf(mat);
    case IDEAL_SPECULAR:
        return idealSpecularBsdf(incoming_ray, outgoing_ray, normal);
    case GLOSSY_SPECULAR:
        return glossySpecularBsdf(incoming_ray, outgoing_ray, normal, mat);
    case REFRACTION:
        return refractionBsdf(incoming_ray, outgoing_ray, normal, mat);
    default:
        std::cerr << "(BRDF) Unsupported Material Type"  << std::endl;
        exit(1);
    }
}

MaterialType BSDF::getType(const tinyobj::material_t& mat) {
    Vector3f specular(mat.specular[0], mat.specular[1], mat.specular[2]);
    Vector3f transmittance(mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]);

//    return REFRACTION;
//    return IDEAL_DIFFUSE;
    return SCATTERING;
//    return IDEAL_SPECULAR;
    if (transmittance.norm() > 0.1) {
        return REFRACTION;
//        return SCATTERING;
    } else if (specular.norm() < DIFFUSE_CUTOFF) {
        return IDEAL_DIFFUSE;
//        return SCATTERING;
    } else if (mat.shininess < SPECULAR_CUTOFF) {
        return GLOSSY_SPECULAR;
//        return SCATTERING;
    } else {
        return IDEAL_SPECULAR;
//        return SCATTERING;
    }
}

float BSDF::fdr(float n) {
    if (n < 1) {
        return -0.4399 + (0.7099/n) - (0.3319/pow(n, 2)) + (0.0636/pow(n, 3));
    } else {
        return -(1.4399/pow(n, 2)) + (0.7099/n) + 0.6681 + 0.0636 * n;
    }
}

Vector3f BSDF::bssrdf1(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray,
                      Vector3f normal, const tinyobj::material_t& mat) {
    outgoing_ray.d = -1.f * outgoing_ray.d;
//    Vector3f sr = SampleRay::refractionDirection(outgoing_ray, normal, mat);

    float n = 1.3;

//    Vector3f sig_t = mat.sig_s + mat.sig_a;
//    std::cout << "hi" << std::endl;
    Vector3f wp_o = SampleRay::refractionDirection(outgoing_ray, normal, mat);
    Vector3f wp_i = SampleRay::refractionDirection(incoming_ray, normal, mat);

    Vector3f single_scat;
    for (int i = 0; i < 3; i++) {
        float total = 0.f;
        for (int j = 0; j < 10; j++) {
            float sig_t = mat.sig_s[i] + mat.sig_a[i];
            float s_o = -std::log(MathUtils::random())/sig_t;
//            float s_o = std::sqrt(-std::log(MathUtils::random())/sig_t);

            Vector3f p = outgoing_ray.o + (s_o * wp_o);
            float s_i = pow((p - position).norm(), 2);
//            std::cout << "s_o: " << s_o << ", s_i: " << s_i << std::endl;

            float dot_wn = (-incoming_ray.d).dot(normal);
            float s_i_prime = (s_i * dot_wn)/std::sqrt(1.f - pow(1.f/n, 2) * (1.f - pow(dot_wn, 2)));

//            std::cout << "s_i: " << s_i << ", s_i_prime: " << s_i_prime << std::endl;
//            std::cout << "s_i_prime: " << s_i_prime << std::endl;
//            float cos_i = normal.dot(-incoming_ray.d);
//            if (cos_i < -1.f) {
//                cos_i = -1.f;
//            } else if (cos_i > 1.f) {
//                cos_i = 1.f;
//            }
//        //                std::cout << "cosi: " << cos_i << std::endl;

            float ft_i = fresnel(AIR_IOR, 1.3, normal.dot(-incoming_ray.d));
//            if (cos_i > 0) {
//                float sin_t = AIR_IOR/1.3 * sqrt(std::max(0.f, 1.f - powf(cos_i, 2)));
//                if (sin_t > 1) {
//                    ft_i = 0.f;
//                } else {
//                    float cos_t = sqrtf(std::max(0.f, 1 - sin_t * sin_t));
//                    ft_i = fresnel(AIR_IOR, 1.3, cos_i, cos_t);
//                }
//            } else {
//                float sin_t = 1.3/AIR_IOR * sqrt(std::max(0.f, 1.f - powf(cos_i, 2)));
//                if (sin_t > 1) {
//                    ft_i = 0.f;
//                } else {
//                    float cos_t = sqrtf(std::max(0.f, 1 - sin_t * sin_t));
//                    ft_i = fresnel(1.3, AIR_IOR, cos_i, cos_t);
//                }
//            }
    //        ft_i *= 10.f;
    //        float theta_t_sqrd = std::abs(SampleRay::refractionGetAngleSquared(incoming_ray, normal, mat));
    //        float cos_theta_t = std::sqrt(theta_t_sqrd);
    //        float theta_i = normal.dot(-incoming_ray.d);

    //        float n_i = AIR_IOR;
    //        float n_t = 1.3;

    //        float ft_i = (2.f * n_i * std::cos(theta_i))/(n_i * cos_theta_t + n_t * std::cos(theta_i));
            float G = std::abs(normal.dot(wp_o)/normal.dot(wp_i));
//            std::cout << G << std::endl;
            float sig_tc = sig_t + G * sig_t;
//            std::cout << sig_tc << std::endl;

    //        std
            float res = (mat.sig_s[i] * (1.f/(4.f*M_PI)))/sig_tc;
//            float res = (mat.sig_s[i] * ft_i * (1.f/(4.f*M_PI)))/sig_tc;
            res *= pow(M_E, -std::abs(s_i_prime) * sig_t);
            res *= pow(M_E, -s_o * sig_t) * ft_i;

//            std::cout << -std::abs(s_i_prime) * sig_t << std::endl;
            total += res;
        }
        single_scat[i] = total/20.f;
    }

//    std::cout << single_scat << std::endl;
    outgoing_ray.d = -1.f * outgoing_ray.d;
    return single_scat;
//    re
//    float av_sig_t = (sig_t[0] + sig_t[1] + sig_t[2])/3.f;
//    float dist = std::log(MathUtils::random())/av_sig_t;
}

float BSDF::fresnel(float n1, float n2, float cosi) {
    if (cosi < -1.f) {
        cosi = -1.f;
    } else if (cosi > 1.f) {
        cosi = 1.f;
    }

    float sint = (n1/n2) * std::sqrt(std::max(0.0, 1.0 - pow(cosi, 2)));

    if (sint > 1.f) {
//        return 1.f;
        return 0.f;
    }

    float cost = std::sqrt(std::max(0.0, 1.0 - pow(sint, 2)));
    cosi = std::abs(cosi);

    float F1_R = pow((n2 * cosi - n1 * cost)/(n2 * cosi + n1 * cost), 2);
    float F2_R = pow((n1 * cosi - n2 * cost)/(n1 * cosi + n2 * cost), 2); // ???

    float F_R = (F1_R + F2_R)/2.f;

    return 1.f - F_R;
}

Vector3f BSDF::bssrdf(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray,
                      Vector3f normal, const tinyobj::material_t& mat) {
//    if (std::abs(-incoming_ray.d.dot(normal) - 1.f) <= 0.0001f) {
////        std::cout << -incoming_ray.d.dot(normal) << std::endl;
//        return Vector3f(.2, 0.03, 0.03);
//    } else {
////        std::cout << "loljk" << std::endl;
//        return Vector3f(.002, 0.003, 0.03);
//    }
//    Vector3f single_scat = bssrdf1(incoming_ray, position, outgoing_ray, normal, mat);
//    mat.dif
//    return idealDiffuseBsdf(mat);
//    return Vector3f(0.9, 0.44, 0.4);
//    incoming_ray.d = -incoming_ray.d;
    Vector3f sig_s = mat.sig_s;
//    Vector3f sig_s = mat.sig_s.cwiseQuotient(Vector3f(500, 500, 500));
    Vector3f sig_a = mat.sig_a;
//    Vector3f sig_a = mat.sig_a.cwiseQuotient(Vector3f(500, 500, 500));

    Vector3f sig_t = sig_s + sig_a;
    Vector3f sig_tr_squared = 3.f * sig_a.cwiseProduct(sig_t);
    Vector3f sig_tr = sig_tr_squared.cwiseSqrt();

    Vector3f f_dr = Vector3f(fdr(1.3), fdr(1.3), fdr(1.3));
    Vector3f A = (Vector3f(1.f, 1.f, 1.f) + f_dr).cwiseQuotient(Vector3f(1.f, 1.f, 1.f) - f_dr);

    Vector3f z_r = sig_t.cwiseInverse();
    Vector3f z_v = (Vector3f(1, 1, 1) + (4.f/3.f) * A).cwiseQuotient(sig_t);
    Vector3f alpha = sig_s.cwiseQuotient(sig_t);
//    std::cout << z_r << std::endl;

    float dist = (outgoing_ray.o - position).norm();
//    if (dist < 0.0001) {
//        return Vector3f(1, 0.01, 0.01);
//    }
//    std::cout << dist << std::endl;
//    if (dist != 0 || dist != 1) {
//        std::cout << "rays: " << std::endl;
//        std::cout << position << std::endl;
//        std::cout << outgoing_ray.o << std::endl;
//    }
//    dist = 0;
//    if (dist < 1.f/((sig_t[0] + sig_t[1] + sig_t[2])/3.f)) {
//        dist = 1.f/((sig_t[0] + sig_t[1] + sig_t[2])/3.f);
//    }

    Vector3f r = Vector3f(dist, dist, dist);

    for (int i = 0; i < 3; i++) {
//        std::cout << r[i] << ", " << 1.f/sig_t[i] << std::endl;
        if (r[i] < 1.f/sig_t[i]) {
            r[i] = 1.f/sig_t[i];
        }
    }

    Vector3f d_r = (r.cwiseProduct(r) + z_r.cwiseProduct(z_r)).cwiseSqrt();
    Vector3f d_v = (r.cwiseProduct(r) + z_v.cwiseProduct(z_v)).cwiseSqrt();
//    std::cout << d_v << std::endl;

    Vector3f R;
    for (int i = 0; i < 3; i++) {
        float sdr = sig_tr[i] * d_r[i];
        float sdv = sig_tr[i] * d_v[i];

        float r_r = (alpha[i] * z_r[i] * (1.f + sdr) * pow(M_E, -sdr))/(4.f * M_PI * pow(d_r[i], 3));
        float r_v = (alpha[i] * z_v[i] * (1.f + sdv) * pow(M_E, -sdv))/(4.f * M_PI * pow(d_v[i], 3));

//        std::cout << (4.f * M_PI * pow(d_r[i], 3)) << std::endl;
//        std::cout << sdr << std::endl;
        R[i] = r_r - r_v;
    }

    /////////////////////////////////////////////////////////////////////
    float ft_i = fresnel(AIR_IOR, 1.3, normal.dot(-incoming_ray.d));
    float ft_o = fresnel(AIR_IOR, 1.3, normal.dot(outgoing_ray.d));

    Vector3f res = (1.f/M_PI) * std::abs(ft_i) * R * std::abs(ft_o);
//    std::cout << ft_i << std::endl;
//    Vector3f res = 100.f * R;
//    Vector3f res = (1.f/M_PI) * R;

//    std::cout << mat.diff[0] << std::endl;
    res = Vector3f(mat.diff[0] * std::abs(res[0]),
            mat.diff[1] * std::abs(res[1]),
            mat.diff[2] * std::abs(res[2]));
//    res = std::abs(ft_i) * single_scat * std::abs(ft_o);
//    res = Vector3f(std::abs(res[0]), std::abs(res[1]), std::abs(res[2]));
//    res = Vector3f(0.5 * mat.diff[0] + std::abs(res[0]),
//            0.5 * mat.diff[1] + std::abs(res[1]),
//            0.5 * mat.diff[2] + std::abs(res[2]));
//    std::cout << res << std::endl;
//    res = Vector3f(0.3, 0.3, 0.3);

//    return Vec;
//    std::cout << "r: " << dist << std::endl;
//    std::cout << "Rd: " << R << std::endl;
//    std::cout << R << std::endl;
    return res;
}

Vector3f BSDF::glossySpecularBsdf(Ray incoming_ray, Ray outgoing_ray, Vector3f normal, const tinyobj::material_t& mat) {
    Vector3f out = outgoing_ray.d;

    Vector3f specular(mat.specular[0], mat.specular[1], mat.specular[2]);
    float shininess = mat.shininess;

    Vector3f ideal_reflection = MathUtils::reflect(incoming_ray.d, normal);
    return specular * (shininess + 2.f) / (2.f * M_PI) * pow(ideal_reflection.dot(out), shininess);
}

Vector3f BSDF::idealDiffuseBsdf(const tinyobj::material_t& mat) {
    Vector3f diffuse(mat.diffuse[0], mat.diffuse[1], mat.diffuse[2]);
//    std::cout << diffuse/M_PI << std::endl;
    return diffuse / M_PI;
//    return Vector3f(.002, 0.003, 0.03);
}

Vector3f BSDF::idealSpecularBsdf(Ray incoming_ray, Ray outgoing_ray, Vector3f normal) {
    Vector3f out = outgoing_ray.d;

    Vector3f N = incoming_ray.is_in_air ? normal : -normal;

    Vector3f target = MathUtils::reflect(incoming_ray.d, N);
    if (MathUtils::isSameDirection(target, out) == false) {
        return Vector3f(0, 0, 0); // Incorrect out vector has 0 weight
    }

    Vector3f full_reflection(1.f, 1.f, 1.f); // Assume perfect reflectance and no absorbtion.
    return full_reflection / out.dot(N);
}

Vector3f BSDF::refractionBsdf(Ray incoming_ray, Ray outgoing_ray, Vector3f normal, const tinyobj::material_t& mat) {
    Vector3f out = outgoing_ray.d;

    Vector3f target = SampleRay::refractionDirection(incoming_ray, normal, mat);
    if (MathUtils::isSameDirection(target, out) == false) {
        return Vector3f(0, 0, 0); // Incorrect out vector has 0 weight
    }

    Vector3f N = incoming_ray.is_in_air ? normal : -normal;

    Vector3f full_refraction(1.f, 1.f, 1.f); // Assume perfect refraction and no absorbtion.
    return full_refraction / out.dot(N);
}

