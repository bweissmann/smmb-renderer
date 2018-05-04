#ifndef SAMPLERAY_H
#define SAMPLERAY_H

#include <BVH/Ray.h>
#include "bsdf.h"
#include "scene/scene.h"

/** an object to return a ray and the probability of choosing that ray */
struct SampledRayInfo {
    Ray ray;
    float prob;

    SampledRayInfo(Ray r, const float p) :
        ray(r), prob(p) {}
};

class SampleRay
{
public:
    SampleRay();

    static SampledRayInfo sampleRay(const MaterialType type, const Eigen::Vector3f &position, const Ray &incoming_ray,
                                        const Eigen::Vector3f &surface_normal, const tinyobj::material_t& mat, const Scene &scene);

    static SampledRayInfo singleScattering(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                     const Eigen::Vector3f &surface_normal, const tinyobj::material_t &mat);

    static SampledRayInfo diffuseScattering(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                     const Eigen::Vector3f &surface_normal, const tinyobj::material_t &mat, const Scene &scene);

    static SampledRayInfo uniformSampleHemisphere(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                                  const Eigen::Vector3f &surface_normal);

    static SampledRayInfo sampleIdealDiffuseImportance(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                                  const Eigen::Vector3f &surface_normal);

    static SampledRayInfo sampleGlossySpecularImportance(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                                  const Eigen::Vector3f &surface_normal, const tinyobj::material_t& mat);

    static SampledRayInfo idealSpecularReflection(const Eigen::Vector3f &position, const Ray &incoming_ray, const Eigen::Vector3f &surface_normal);

    static SampledRayInfo refraction(const Eigen::Vector3f &position, const Ray &incoming_ray,
                                     const Eigen::Vector3f &surface_normal, const tinyobj::material_t& mat);

    static Eigen::Vector3f refractionDirection(const Ray &incoming_ray, const Eigen::Vector3f &surface_normal,
                                               const tinyobj::material_t& mat);

    static float refractionGetAngleSquared(const Ray &incoming_ray, const Eigen::Vector3f &surface_normal,
                                    const tinyobj::material_t& mat);
    static Eigen::Vector3f tangentToWorldSpaceNotNormalized(const Eigen::Vector3f &surface_normal, const Eigen::Vector3f &tangentspace_direction);

private:
    static Eigen::Vector3f tangentToWorldSpace(const Eigen::Vector3f &surface_normal, const Eigen::Vector3f &tangentspace_direction);


};

#endif // SAMPLERAY_H
