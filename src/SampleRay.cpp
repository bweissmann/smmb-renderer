#include "SampleRay.h"

#include <util/MathUtils.h>
#include <BVH/Ray.h>
#include "bsdf.h"

#include <cmath>
#include <iostream>

using namespace Eigen;

SampleRay::SampleRay()
{

}

SampledRayInfo SampleRay::sampleRay(const MaterialType type, const Vector3f &position, const Ray &incoming_ray,
                                    const Vector3f &surface_normal, const tinyobj::material_t& mat) {

    switch (type) {
    case IDEAL_DIFFUSE:
        return sampleIdealDiffuseImportance(position, incoming_ray, surface_normal);
//        return uniformSampleHemisphere(position, incoming_ray, surface_normal);
    case GLOSSY_SPECULAR:
//        return uniformSampleHemisphere(position, incoming_ray, surface_normal);
        return sampleGlossySpecularImportance(position, incoming_ray, surface_normal, mat);
    case IDEAL_SPECULAR:
        return idealSpecularReflection(position, incoming_ray, surface_normal);
    case REFRACTION:
        return refraction(position, incoming_ray, surface_normal, mat);
    default:
        std::cerr << "(SampleRay) Unsupported Material Type" << std::endl;
        exit(1);
    }
}

SampledRayInfo SampleRay::uniformSampleHemisphere(const Vector3f &position, const Ray &incoming_ray,
                                                  const Vector3f &surface_normal) {
    float phi = acosf(MathUtils::random());
    float theta = 2.f * M_PI * MathUtils::random();

    const Vector3f tangentspace_direction = Vector3f(sinf(phi) * cosf(theta), sinf(phi) * sinf(theta), cosf(phi));
    const Vector3f worldspace_direction = tangentToWorldSpace(surface_normal, tangentspace_direction);

    Ray ray(position, worldspace_direction, incoming_ray.index_of_refraction, incoming_ray.is_in_air);

    return SampledRayInfo(ray, 1 / (2.f * M_PI));
}


SampledRayInfo SampleRay::sampleIdealDiffuseImportance(const Vector3f &position, const Ray &incoming_ray,
                                                      const Vector3f &surface_normal) {

    //TODO::DETERMINE WHICH OF THREE METHODS ARE RIGHT

        float phi = acosf(sqrt(MathUtils::random()));
    float theta = 2.f * M_PI * MathUtils::random();

    const Vector3f tangentspace_direction = Vector3f(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    const Vector3f worldspace_direction = tangentToWorldSpace(surface_normal, tangentspace_direction);

    Ray ray(position, worldspace_direction, incoming_ray.index_of_refraction, incoming_ray.is_in_air);
    float cos_phi = worldspace_direction.dot(surface_normal);
    return SampledRayInfo(ray, cos_phi / M_PI);


    //other solution
//    float r = sqrtf(MathUtils::random());
//    float theta = 2.0f * M_PI * MathUtils::random();
//    float x = r * cosf(theta);
//    float y = r * sinf(theta);
//    const Vector3f tangentspace_direction(x, y, fmaxf(0, 1.f - x * x - y * y));
//    const Vector3f worldspace_direction = tangentToWorldSpace(surface_normal, tangentspace_direction);
//    Ray ray(position, worldspace_direction, incoming_ray.index_of_refraction, incoming_ray.is_in_air);
//    float cos_phi = worldspace_direction.dot(surface_normal);

//    return SampledRayInfo(ray, cosf(theta) * sinf(theta) / M_PI);

//    return SampledRayInfo(ray, cos_phi / M_PI);
}

SampledRayInfo SampleRay::sampleGlossySpecularImportance(const Vector3f &position, const Ray &incoming_ray,
                                                      const Vector3f &surface_normal, const tinyobj::material_t& mat) {
    Vector3f reflected_direction = MathUtils::reflect(incoming_ray.d, surface_normal);
    float n = mat.shininess;
    float phi = acosf(pow(MathUtils::random(), 1.f /(n + 1.f)));
    float theta = 2.f * M_PI * MathUtils::random();

    const Vector3f tangentspace_direction = Vector3f(sin(phi) * cos(theta), sin(phi) * sin(theta), cos(phi));
    Vector3f worldspace_direction = tangentToWorldSpace(reflected_direction, tangentspace_direction);
    if (worldspace_direction.dot(surface_normal) < 0.f) {
//        worldspace_direction = MathUtils::reflect(-1.f * worldspace_direction, reflected_direction);
//        worldspace_direction = MathUtils::reflect(-1.f * worldspace_direction, surface_normal);
        worldspace_direction *= -1.f;
    }

    Ray ray(position, worldspace_direction, incoming_ray.index_of_refraction, incoming_ray.is_in_air);
    float cos_phi = reflected_direction.dot(worldspace_direction);

    //determine probability and check refract
    return SampledRayInfo(ray, (n + 1.f) * pow(cos_phi, n) / (2.f * M_PI));
}


SampledRayInfo SampleRay::idealSpecularReflection(const Vector3f &position, const Ray &incoming_ray, const Vector3f &surface_normal) {
    Vector3f reflected_direction = MathUtils::reflect(incoming_ray.d, surface_normal);

    Ray ray(position, reflected_direction, incoming_ray.index_of_refraction, incoming_ray.is_in_air);

    return SampledRayInfo(ray, 1.f);
}

SampledRayInfo SampleRay::refraction(const Vector3f &position, const Ray &incoming_ray,
                                 const Vector3f &surface_normal, const tinyobj::material_t& mat) {

    float outgoing_ior = incoming_ray.is_in_air ? mat.ior : AIR_IOR;
    const Vector3f outgoing_direction = refractionDirection(incoming_ray, surface_normal, mat);

    return SampledRayInfo(Ray(position, outgoing_direction, outgoing_ior, !incoming_ray.is_in_air), 1.f);
}

Vector3f SampleRay::refractionDirection(const Ray &incoming_ray, const Vector3f &surface_normal,
                                        const tinyobj::material_t& mat) {
    float n_i = incoming_ray.index_of_refraction;
    Vector3f I = -incoming_ray.d; // Flip the incoming ray for the incident vector

    // This is the case of Air -> Material or Material -> Air
    float n_t = incoming_ray.is_in_air ? mat.ior : AIR_IOR;
    Vector3f N = incoming_ray.is_in_air ? surface_normal : -surface_normal;

    float eta = n_i / n_t;
    float cos_t_theta_squared = refractionGetAngleSquared(incoming_ray, surface_normal, mat);
    if (cos_t_theta_squared < 0.0) {
        std::cerr << "(SampleRay refractionDirection): Negative Cosine in Law of Refraction" << std::endl;
        exit(1);
    }

    return (eta * -I + (eta * N.dot(I) - sqrt(cos_t_theta_squared)) * N).normalized();
}

float SampleRay::refractionGetAngleSquared(const Ray &incoming_ray, const Vector3f &surface_normal,
                                           const tinyobj::material_t& mat) {
    float n_i = incoming_ray.index_of_refraction;
    Vector3f I = -incoming_ray.d; // Flip the incoming ray for the incident vector

    // This is the case of Air -> Material or Material -> Air
    float n_t = incoming_ray.is_in_air ? mat.ior : AIR_IOR;
    Vector3f N = incoming_ray.is_in_air ? surface_normal : -surface_normal;

    float eta = n_i / n_t;
    return 1.0 - eta * eta * (1.0 - N.dot(I) * N.dot(I));
}


Vector3f SampleRay::tangentToWorldSpace(const Vector3f &surface_normal, const Vector3f &tangentspace_direction) {

    // Create a ray to cross with the normal to get *a* tangent vector. Make sure its not equal to the normal
    Vector3f not_the_normal = (surface_normal.x() > 0.1 || surface_normal.x() < -0.1)
            ? Vector3f(0, 1.f, 0) : Vector3f(1.f, 0, 0);

    const Vector3f surface_tangent = surface_normal.cross(not_the_normal);
    const Vector3f surface_bitangent = surface_normal.cross(surface_tangent);

    return (surface_tangent * tangentspace_direction.x() +
            surface_bitangent * tangentspace_direction.y() +
            surface_normal * tangentspace_direction.z()).normalized();
}
