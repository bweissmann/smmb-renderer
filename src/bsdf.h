#ifndef BSDF_H
#define BSDF_H

#include "Eigen/Dense"

#include "util/tiny_obj_loader.h"

struct Ray;

enum MaterialType {
    IDEAL_DIFFUSE,
    IDEAL_SPECULAR,
    GLOSSY_SPECULAR,
    REFRACTION,
    DIFFUSE_SCATTERING,
    SINGLE_SCATTERING,
    LIGHT,
    EYE
};

class BSDF
{
public:
    BSDF();

    static MaterialType getType(const tinyobj::material_t& mat);

    static Eigen::Vector3f getBsdfFromType(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray, Eigen::Vector3f normal,
                                   const tinyobj::material_t& mat, MaterialType type);



    //computes directional probability based given incoming and outgoing
    static float getBsdfDirectionalProb(const Eigen::Vector3f &incoming, const Eigen::Vector3f &outgoing, Eigen::Vector3f normal, float radius,
                                        const tinyobj::material_t& mat, MaterialType type, float eta);

private:

    static float fdr(float n);

    static float fresnel(float n1, float n2, float cosi);

    static Eigen::Vector3f reflectance(float r, const tinyobj::material_t &mat);

    static Eigen::Vector3f diffuseScatteringBssrdf(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray,
                                      Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f singleScatteringBssrdf(Ray incoming_ray, Eigen::Vector3f &position, Ray outgoing_ray,
                                  Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f glossySpecularBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                              Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f idealSpecularBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                             Eigen::Vector3f normal);

    static Eigen::Vector3f refractionBsdf(Ray incoming_ray, Eigen::Vector3f out,
                                             Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f idealDiffuseBsdf(const tinyobj::material_t& mat);

    static float cosineWeightedProb(const Eigen::Vector3f &outgoing, Eigen::Vector3f normal);

    static float glossyWeightedProb(const Eigen::Vector3f &incoming, const Eigen::Vector3f &outgoing, Eigen::Vector3f normal, const tinyobj::material_t& mat);

    static float idealSpecularProb(const Eigen::Vector3f &incoming, const Eigen::Vector3f &outgoing, Eigen::Vector3f normal);
    static float idealRefractionProb(const Eigen::Vector3f &incoming, const Eigen::Vector3f &outgoing, Eigen::Vector3f normal, float eta);


    static constexpr float DIFFUSE_CUTOFF = 0.001f;
    static constexpr float SPECULAR_CUTOFF = 100.0f;
};

#endif // BSDF_H
