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
                                    const Vector3f &surface_normal, const tinyobj::material_t& mat, const Scene &scene) {
    switch (type) {
    case IDEAL_DIFFUSE:
        return sampleIdealDiffuseImportance(position, incoming_ray, surface_normal);
    case GLOSSY_SPECULAR:
        return sampleGlossySpecularImportance(position, incoming_ray, surface_normal, mat);
    case IDEAL_SPECULAR:
        return idealSpecularReflection(position, incoming_ray, surface_normal);
    case REFRACTION:
        return refraction(position, incoming_ray, surface_normal, mat);
    case SINGLE_SCATTERING:
//        return singleScattering(position, incoming_ray, surface_normal, mat, scene);
        return scattering(position, incoming_ray, surface_normal, mat, scene);
    case DIFFUSE_SCATTERING:
        return scattering(position, incoming_ray, surface_normal, mat, scene);
    default:
        std::cerr << "(SampleRay) Unsupported Material Type" << std::endl;
        exit(1);
    }
}


SampledRayInfo SampleRay::singleScattering(const Vector3f &position, const Ray &incoming_ray,
                                     const Vector3f &surface_normal, const tinyobj::material_t& mat,
                                     const Scene &scene) {
    SampledRayInfo sampled_ray = uniformSampleHemisphere(position, incoming_ray, surface_normal);

    float eps_1 = MathUtils::random();
    float eps_2 = MathUtils::random();

    Vector3f sig_a = Vector3f(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
    Vector3f sig_s = Vector3f(mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]);
    Vector3f sig_t = sig_a + sig_s;
    float sig_tr = (3.f * mat.sig_a.cwiseProduct(sig_t)).cwiseSqrt().norm();

    float r = -std::log(eps_1)/sig_tr;
    float theta = 2.f * M_PI * eps_2;

    const Vector3f tangentspace_pos = Vector3f(r * cos(theta), 0.0f, r * sin(theta));
    const Vector3f worldspace_pos = tangentToWorldSpaceNotNormalized(surface_normal, tangentspace_pos);
//    sampled_ray.ray.o = position + worldspace_pos;

    sampled_ray.ray.o = position + worldspace_pos;
    sampled_ray.ray.d = -sampled_ray.ray.d;

    IntersectionInfo i;
    float d = r;
    Vector3f normal = surface_normal;
    Ray back_normal(sampled_ray.ray.o, -surface_normal, AIR_IOR, true);
    if(scene.getBVH().getIntersection(back_normal, &i, false)) {
        const Mesh * m = static_cast<const Mesh *>(i.object);
        const Triangle *t = static_cast<const Triangle *>(i.data);
//        normal = t->getNormal(i);
        sampled_ray.ray.o = i.hit;
//        d = (sampled_ray.ray.o - position).norm();
    }

    sampled_ray.ray.d = -sampled_ray.ray.d;
    float pdf = sig_tr * pow(M_E, -sig_tr * r);

//    std::cout << "RAD: " << r << std::endl;
    return SampledRayInfo(sampled_ray.ray, sampled_ray.prob * std::abs(pdf));
}


SampledRayInfo SampleRay::scattering(const Vector3f &position, const Ray &incoming_ray,
                                     const Vector3f &surface_normal, const tinyobj::material_t& mat,
                                     const Scene &scene) {
    SampledRayInfo sampled_ray = uniformSampleHemisphere(position, incoming_ray, surface_normal);

    float eps_1 = MathUtils::random();
    float eps_2 = MathUtils::random();

    Vector3f sig_a = Vector3f(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
    Vector3f sig_s = Vector3f(mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]);
    Vector3f sig_t = sig_a + sig_s;

    float sig_tr = (3.f * mat.sig_a.cwiseProduct(sig_t)).cwiseSqrt().norm();
    float v = 1.f/(2.f * sig_tr);
    float R_m = std::sqrt(v/12.46);

    float r = std::sqrt(std::log(1.f - eps_1 * (1.f - pow(M_E, -sig_tr * pow(R_m, 2))))/-sig_tr);
    float theta = 2.f * M_PI * eps_2;

    const Vector3f tangentspace_pos = Vector3f(r * cos(theta), 0.0f, r * sin(theta));
    const Vector3f worldspace_pos = tangentToWorldSpaceNotNormalized(surface_normal, tangentspace_pos);
//    sampled_ray.ray.o = position + worldspace_pos;

    sampled_ray.ray.o = position + worldspace_pos;
    sampled_ray.ray.d = -sampled_ray.ray.d;

    IntersectionInfo i;
    float d = r;
    Vector3f normal = surface_normal;
    Ray back_normal(sampled_ray.ray.o, -surface_normal, AIR_IOR, true);
    if(scene.getBVH().getIntersection(back_normal, &i, false)) {
        const Mesh * m = static_cast<const Mesh *>(i.object);
        const Triangle *t = static_cast<const Triangle *>(i.data);
        normal = t->getNormal(i);
        sampled_ray.ray.o = i.hit;
        d = (sampled_ray.ray.o - position).norm();
    }

    sampled_ray.ray.d = -sampled_ray.ray.d;

//    float d = (sampled_ray.ray.o - position).norm();
//    std::cout << "r: " << r << " d: " << d << std::endl;
    float R_dr =  1.f/(2.f * M_PI * v) * pow(M_E, -pow(r, 2)/(2.f * v));
    float R_dr2 =  1.f/(2.f * M_PI * v) * pow(M_E, -pow(d, 2)/(2.f * v));
//    float R_dr = 1.f/M_PI * sig_tr * pow(M_E, -sig_tr * pow(r, 2));
//    float pdf = R_dr/(1.f - pow(M_E, -sig_tr * pow(R_m, 2)));
    float pdf = R_dr/(1.f - pow(M_E, -pow(R_m, 2)/(2.f * v)))/100.f;
//    pdf = (R_dr2/pdf)/(normal.dot(-sampled_ray.ray.d));

//    if (pdf < 0.0001) {
//        pdf = 0.0001;
//    }
//    std::cout << pdf << std::endl;
//    std::cout << r << std::endl;
//    std::cout << "denomn: " << (1.f - pow(M_E, -pow(R_m, 2)/(2.f * v))) << std::endl;
    return SampledRayInfo(sampled_ray.ray, sampled_ray.prob * std::abs(pdf));
}


SampledRayInfo SampleRay::scattering2(const Vector3f &position, const Ray &incoming_ray,
                                     const Vector3f &surface_normal, const tinyobj::material_t& mat,
                                     const Scene &scene) {
    SampledRayInfo r = uniformSampleHemisphere(position, incoming_ray, surface_normal);

    Vector3f sig_t = Vector3f(mat.transmittance[0], mat.transmittance[1], mat.transmittance[2]) +
            Vector3f(mat.ambient[0], mat.ambient[1], mat.ambient[2]);
    Vector3f sig_tr = (3.f * mat.sig_a.cwiseProduct(sig_t)).cwiseSqrt();

    float rad = std::sqrt(-std::log(MathUtils::random())/((sig_tr[0] + sig_tr[1] + sig_tr[2])/3.f)); // HARD CODED IN !!!!! // sqrt ??
//    rad = -std::log(MathUtils::random())/((sig_tr[0] + sig_tr[1] + sig_tr[2])/3.f);
//    std::cout << "rad: " << rad << std::endl;
    float theta = 2.f * M_PI * MathUtils::random();

    const Vector3f tangentspace_pos = Vector3f(rad * cos(theta), 0.f, rad * sin(theta));
    const Vector3f worldspace_pos = tangentToWorldSpaceNotNormalized(surface_normal, tangentspace_pos);

    r.ray.o = position + worldspace_pos;
    r.ray.d = -r.ray.d;

    IntersectionInfo i;
    if(scene.getBVH().getIntersection(r.ray, &i, false)) {
//        std::cout << "yess" << std::endl;
        r.ray.o = i.hit;
    } else {
//        std::cout << "no" << std::endl;
    }

    r.ray.d = -r.ray.d;

//    std::cout << "rays: " << std::endl;
//        std::cout << sig_tr << std::endl;
//    std::cout << position << std::endl;
//    std::cout << r.ray.o << std::endl;
//    std::cout << (r.ray.o - position).norm() << std::endl;

    float av_sig_tr = (sig_tr[0] + sig_tr[1] + sig_tr[2])/3.f;
//    float prob = av_sig_tr * std::pow(M_E, -av_sig_tr * rad);
    float prob = 2.f * av_sig_tr * rad * pow(M_E, -av_sig_tr * pow(rad, 2));

//    std::cout << "prob: " << prob << std::endl;

//    return SampledRayInfo(r.ray, 0.16);
    return SampledRayInfo(r.ray, r.prob * prob);
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
    float cos_phi = fabs(worldspace_direction.dot(surface_normal));
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

Vector3f SampleRay::tangentToWorldSpaceNotNormalized(
        const Vector3f &surface_normal, const Vector3f &tangentspace_direction) {

    // Create a ray to cross with the normal to get *a* tangent vector. Make sure its not equal to the normal
    Vector3f not_the_normal = (surface_normal.x() > 0.1 || surface_normal.x() < -0.1)
            ? Vector3f(0, 1.f, 0) : Vector3f(1.f, 0, 0);

    const Vector3f surface_tangent = surface_normal.cross(not_the_normal);
    const Vector3f surface_bitangent = surface_normal.cross(surface_tangent);

    return (surface_tangent * tangentspace_direction.x() +
            surface_bitangent * tangentspace_direction.y() +
            surface_normal * tangentspace_direction.z());
}
