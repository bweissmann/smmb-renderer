#ifndef BSDF_H
#define BSDF_H

#include "Eigen/Dense"

#include "util/tiny_obj_loader.h"

struct Ray;

enum MaterialType {
    IDEAL_DIFFUSE,
    IDEAL_SPECULAR,
    GLOSSY_SPECULAR,
    REFRACTION
};

class BSDF
{
public:
    BSDF();

    static MaterialType getType(const tinyobj::material_t& mat);

    static Eigen::Vector3f getBsdfFromType(Ray incoming_ray, Eigen::Vector3f out, Eigen::Vector3f normal,
                                   const tinyobj::material_t& mat, MaterialType type);

private:
    static Eigen::Vector3f glossySpecularBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                              Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f idealSpecularBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                             Eigen::Vector3f normal);

    static Eigen::Vector3f refractionBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                             Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f idealDiffuseBsdf(const tinyobj::material_t& mat);

    static constexpr float DIFFUSE_CUTOFF = 0.001f;
    static constexpr float SPECULAR_CUTOFF = 100.0f;
};

#endif // BSDF_H
