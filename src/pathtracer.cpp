#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <util/MathUtils.h>
#include "SampleRay.h"
#include "renderthread.h"
#include "bsdf.h"
#include "util/statuslogger.h"

#include <QThreadPool>

using namespace Eigen;

PathTracer::PathTracer(int width, int image_height, int output_height, int section_id)
    : m_width(width), m_image_height(image_height), m_output_height(output_height), m_section_id(section_id)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    // Initialize the tread pool
    QThreadPool *threadPool = QThreadPool::globalInstance();
    std::vector<RenderThread *> threads;
    threads.resize(m_width * m_output_height);

    // Setup intensity values and logging
    Vector3f intensityValues[m_width * m_output_height]; // Init intensity values
    StatusLogger::initInstance(PARALLEL_RANGE, m_width, m_output_height); // Init status logger
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

    // start each thread
    for(int y = 0; y < m_output_height; y += PARALLEL_RANGE) {
        for(int x = 0; x < m_width; x += PARALLEL_RANGE) {
            int thread_index = x + (y * m_width);
            threads[thread_index] = new RenderThread;
            threads[thread_index]->setData(this, intensityValues, scene, x, y, PARALLEL_RANGE, &invViewMat);
            threads[thread_index]->setAutoDelete(false);
            threadPool->start(threads[thread_index]);
        }
    }

    // Wait for rendering to finish
    threadPool->waitForDone();

    toneMap(imageData, intensityValues);
}

void PathTracer::tracePixel(int output_x, int output_y, const Scene& scene,
                            Vector3f *intensityValues, const Eigen::Matrix4f &invViewMatrix)
{
    int pixel_x = output_x;
    int pixel_y = output_y + m_section_id * m_output_height;
    int output_index = output_x + output_y * m_width;

    Vector3f output_radience = Vector3f::Zero();
    Vector3f eye_center(0, 0, 0);

    for (int i = 0; i < M_NUM_SAMPLES; i++) {
        /* Sample an x and y randomly within the sensor square */
        float x = pixel_x + MathUtils::random() - 0.5f;
        float y = pixel_y + MathUtils::random() - 0.5f;

        Vector3f screen_plane_pos(((2.f * x / m_width) - 1), (1 - (2.f * y / m_image_height)), -1);

        Vector3f d = (screen_plane_pos - eye_center).normalized();

        const Ray camera_space_ray(eye_center, d, AIR_IOR, true);
        const Ray world_camera_space_ray = camera_space_ray.transform(invViewMatrix);

        output_radience += traceRay(world_camera_space_ray, scene, 0);
    }

    intensityValues[output_index] = output_radience / M_NUM_SAMPLES;
}

//pass in &path
Vector3f PathTracer::traceRay(const Ray& ray, const Scene& scene, int depth)
{
    IntersectionInfo i;
    Vector3f total_light(0.f, 0.f, 0.f); // Accumulate different types of light

    if(scene.getBVH().getIntersection(ray, &i, false)) {
        const Mesh * m = static_cast<const Mesh *>(i.object);//Get the mesh that was intersected
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = m->getMaterial(t->getIndex());//Get the material of the triangle from the mesh
        const tinyobj::real_t *e = mat.emission; //Emitted color
        const Vector3f normal = t->getNormal(i); //surface normal

        // Ignore all Emitted Light
        const Vector3f emitted_light = Vector3f(e[0], e[1], e[2]);
        if (emitted_light.norm() > 0) {
            return (depth == 0) ? emitted_light : Vector3f(0.f, 0.f, 0.f);
        }

        MaterialType type = BSDF::getType(mat);
        float schlick = 1.f;

        // If the cos_theta_squared term is negative or based on schlick's probability, then
        // We should reflect instead of refract.
        if (type == REFRACTION) {
            float cos_t_theta_squared = SampleRay::refractionGetAngleSquared(ray, normal, mat);
            if (cos_t_theta_squared < 0.0) {
                type = IDEAL_SPECULAR;
            }

            Vector3f N = ray.is_in_air ? normal : -normal;
            Vector3f I = -ray.d;
            float n_i = ray.index_of_refraction;
            float n_t = ray.is_in_air ? mat.ior : AIR_IOR;
            float r_0 = pow((n_i - n_t) / (n_i + n_t), 2);
            float cos_theta = fabs(N.dot(I));
            schlick = r_0 + (1 - r_0) * pow(1 - cos_theta, 5);
            if (MathUtils::random() < schlick) {
                type = IDEAL_SPECULAR;
            }
        }

        // Direct Light Contribution
        SampledLightInfo light_info = scene.sampleLight();
        if (lightIsVisible(light_info.position, i.hit, scene)) {
            int num_direct_light = 10;
            for (int j = 0; j < num_direct_light; j++) {
                total_light += directLightContribution(light_info, normal, type, i.hit, ray, mat) / num_direct_light;
            }
        }

        const SampledRayInfo next_ray_info = SampleRay::sampleRay(type, i.hit, ray, normal, mat);

        const Ray next_ray = next_ray_info.ray;
        const Vector3f brdf = BSDF::getBsdfFromType(ray, next_ray.d, normal, mat, type);

        const float pdf_rr = getContinueProbability(brdf);
        if (MathUtils::random() < pdf_rr) {
            // Deal with refraction separately
            if (type == REFRACTION) {
                float d = (i.hit - ray.o).norm();
                Vector3f N = ray.is_in_air ? normal : -normal;
                const Vector3f refracted_light = traceRay(next_ray, scene, 0).cwiseProduct(brdf) * N.dot(next_ray.d)
                        / (pdf_rr * (1.f - schlick) * fmax(d*3, 1.f));
                return refracted_light + total_light;
            }
            // All other material types
            Vector3f N = ray.is_in_air ? normal : -normal;
            int next_depth = (type == IDEAL_SPECULAR) ? 0 : depth + 1;
            const Vector3f reflected_light = traceRay(next_ray, scene, next_depth).cwiseProduct(brdf) * N.dot(next_ray.d)
                    / (next_ray_info.prob * pdf_rr * schlick);
            total_light += reflected_light;
        }
    }

    return total_light;
}

float PathTracer::getContinueProbability(Vector3f brdf) {
//    return fmin(fmax(brdf.sum() / 3.f, 0.1f), 0.99f); // Continue Probability based on brdf
    return 0.5f; // Fixed Continue Probability
}

void PathTracer::toneMap(QRgb *imageData, Vector3f *intensityValues) {
    for(int y = 0; y < m_output_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f hdr_intensity = intensityValues[offset];
            Vector3f ones(1, 1, 1);
            Vector3f out_intensity = hdr_intensity.cwiseQuotient(ones + hdr_intensity) * 256;
            imageData[offset] = qRgb(out_intensity.x(), out_intensity.y(), out_intensity.z());
        }
    }
}

bool PathTracer::lightIsVisible(Vector3f light_position, Vector3f surface_position, const Scene& scene) {
    float epsilon = 0.001; // Epsilon for distance to the light

    Vector3f direction_to_light = (light_position - surface_position).normalized();
    Ray ray(surface_position, direction_to_light, AIR_IOR, true);

    IntersectionInfo i;
    if(scene.getBVH().getIntersection(ray, &i, false)) {
        float distance_from_light = (i.hit - light_position).norm();
        return distance_from_light < epsilon;
    }

    return false;
}

Vector3f PathTracer::directLightContribution(SampledLightInfo light_info, Vector3f surface_normal, MaterialType type,
                                             Vector3f surface_position, Ray incoming_ray, const tinyobj::material_t& mat) {

    if (type == REFRACTION || type == IDEAL_SPECULAR) {
        return Vector3f(0.f, 0.f, 0.f);
    }

    const Vector3f direction_to_light = (light_info.position - surface_position).normalized();
    const float distance_squared = pow((surface_position - light_info.position).norm(), 2);
    const float cos_theta = fmax(surface_normal.dot(direction_to_light), 0);
    const float cos_theta_prime = fmax(light_info.normal.dot(-direction_to_light), 0);
    const Vector3f direct_brdf = BSDF::getBsdfFromType(incoming_ray, direction_to_light, surface_normal, mat, type);

    return light_info.emission.cwiseProduct(direct_brdf) * cos_theta * cos_theta_prime / (distance_squared * light_info.prob);
}



//FUNCTIONS FOR COMPUTING CONTRIBUTION FOR BIDIRECTIONAL


Vector3f PathTracer::computePathContribution(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                             int max_eye_index, int max_light_index) {
    //eye points light points
    //case 1 (0, 0)
    if (max_eye_index == 0 && max_light_index == 0) {
//        return computeZeroBouncePathContrib(eye, light);
    } else if (max_eye_index > 0 && max_light_index == 0) {

    }

    //case 2(s > 0, 0)
    //classical light tracing with direct lighting
}

Vector3f PathTracer::computeZeroBouncePathContrib(const PathNode &eye, const PathNode &light) {

    //TODO:: question about G(x <-> x') when x is eye -> must give normal for to eye?

    //TODO:: add probability calculation?
    Vector3f direction = (eye.position - light.position);
    float squared_dist = direction.squaredNorm();
    direction.normalize();
    return light.emission * eye.surface_normal.dot(-1.f * direction)
            * light.surface_normal.dot(direction) / squared_dist;
}

//path tracing case
Vector3f PathTracer::computePathTracingContrib(const std::vector<PathNode> &eye_path,  const PathNode &light, int max_eye_index) {
    Vector3f contrib(1, 1, 1); //set to zero initially
    for (int i = 1; i < max_eye_index; i++) {
        PathNode node =  eye_path[i];
        contrib = contrib.cwiseProduct(node.brdf) * node.surface_normal.dot(node.outgoing_ray.d) / node.prob;
    }
    PathNode max_eye_node = eye_path[max_eye_index];
    PathNode previous_eye_node = eye_path[max_eye_index - 1];
    Vector3f direction = (light.position - max_eye_node.position);
    float squared_dist = direction.squaredNorm();
    direction.normalize();
    Vector3f brdf = BSDF::getBsdfFromType(previous_eye_node.outgoing_ray, direction, max_eye_node.surface_normal,
                          max_eye_node.mat, max_eye_node.type);

    float throughput = -1.f * direction.dot(light.surface_normal)
            * direction.dot(max_eye_node.surface_normal) / squared_dist;
    contrib = contrib.cwiseProduct(brdf) * throughput;
    contrib = contrib.cwiseProduct(light.emission) / light.prob;
    return contrib;
}


//light tracing case ? do we need to consider it ?
Vector3f PathTracer::computeLightTracingContrib(const std::vector<PathNode> &light_path,  const PathNode &eye, int max_light_index) {

}

//general case
Vector3f PathTracer::computeBidirectionalContrib(const std::vector<PathNode> &eye_path,  const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index) {
    Vector3f light_path_contrib = light_path[0].emission / light_path[0].prob;
    for (int i = 1; i < max_light_index; i++) {
        PathNode light_node = light_path[i];
        float constant = light_node.surface_normal.dot(light_node.outgoing_ray.d) / light_node.prob;
        light_path_contrib = light_path_contrib.cwiseProduct(light_node.brdf) * constant;
    }

    Vector3f eye_path_contrib(1, 1, 1);
    for (int i = 1; i < max_eye_index; i++) {
        PathNode eye_node = eye_path[i];
        float constant = eye_node.surface_normal.dot(eye_node.outgoing_ray.d) / eye_node.prob;
        eye_path_contrib = eye_path_contrib.cwiseProduct(eye_node.brdf) * constant;
    }

    //compute for connection
    PathNode light_node = light_path[max_light_index];
    PathNode eye_node = eye_path[max_eye_index];
    Vector3f to_eye_node = (eye_node.position - light_node.position);
    float distSquared = to_eye_node.squaredNorm();
    to_eye_node.normalize();
    Ray incoming_ray = light_path[max_light_index - 1].outgoing_ray;
    Vector3f light_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_eye_node, light_node.surface_normal,
                                                     light_node.mat, light_node.type);
    incoming_ray = eye_path[max_eye_index - 1].outgoing_ray;
    Vector3f to_light_node = -1 * to_eye_node;
    Vector3f eye_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_light_node, eye_node.surface_normal,
                                                   eye_node.mat, eye_node.type);
    float throughput = to_eye_node.dot(light_node.surface_normal)
            * to_light_node.dot(eye_node.surface_normal) / distSquared;

    Vector3f total_contrib = eye_path_contrib.cwiseProduct(light_path_contrib);
    total_contrib = total_contrib.cwiseProduct(light_node_brdf.cwiseProduct(eye_node_brdf)) * throughput;
    return total_contrib;
}
