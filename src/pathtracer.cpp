#include "pathtracer.h"

#include <iostream>

#include <Eigen/Dense>

#include <util/CS123Common.h>
#include <util/MathUtils.h>
#include "SampleRay.h"
#include "renderthread.h"
#include "bsdf.h"
#include "util/statuslogger.h"
#include "bdpt.h"
#include <QThreadPool>
#include "bdpt2.h"

using namespace Eigen;

PathTracer::PathTracer(int width, int image_height, int output_height, int section_id)
    : m_width(width), m_image_height(image_height), m_output_height(output_height), m_section_id(section_id)
{
}

void PathTracer::traceScene(QRgb *imageData, const Scene& scene)
{
    if (LIGHT_TRACING_ONLY) {
        trace(imageData, scene);
//        lightTrace(imageData, scene);
        return;
    }

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
//            return emitted_light;
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
        int num_direct_light = 1;
        for (int j = 0; j < num_direct_light; j++) {
            SampledLightInfo light_info = scene.sampleLight();
            if (lightIsVisible(light_info.position, i.hit, scene)) {
                total_light += directLightContribution(light_info, normal, type, i.hit, ray, mat) / num_direct_light;
            }
        }

        const SampledRayInfo next_ray_info = SampleRay::sampleRay(type, i.hit, ray, normal, mat);

        const Ray next_ray = next_ray_info.ray;
        const Vector3f brdf = BSDF::getBsdfFromType(ray, next_ray.d, normal, mat, type);

        float pdf_rr = getContinueProbability(brdf);
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


//BIDIRECTIONAL FUNCTIONS

void PathTracer::trace(QRgb *imageData, const Scene& scene) {
    Vector3f intensityValues[m_width * m_output_height]; // Init intensity values
    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

    //initialize to zero
    for (int i = 0; i < m_output_height * m_width; i++) {
        intensityValues[i] = Vector3f(0, 0, 0);
    }

    //trace path from eye and from light for each pixel and connect
    for (int i = 0; i < m_output_height; i++) {
        for (int j = 0; j < m_width; j++) {
            tracePixel3(j, i, scene, intensityValues, invViewMat);
        }
    }

    toneMap(imageData, intensityValues);
}

void PathTracer::tracePixel3(int output_x, int output_y, const Scene& scene,
                             Vector3f *intensityValues, const Eigen::Matrix4f &invViewMatrix) {
    int pixel_x = output_x;
    int pixel_y = output_y + m_section_id * m_output_height;
    int output_index = output_x + output_y * m_width;

    Vector3f output_radience = Vector3f::Zero();
    Vector3f eye_center(0, 0, 0);
    Vector3f eye_normal_world = (invViewMatrix * Vector4f(0, 0, -1, 0)).head<3>();
    int total = 0;

    for (int i = 0; i < M_NUM_SAMPLES; i++) {

        /* Sample an x and y randomly within the sensor square */
        float x = pixel_x + MathUtils::random() - 0.5f;
        float y = pixel_y + MathUtils::random() - 0.5f;
        Vector3f screen_plane_pos(((2.f * x / m_width) - 1), (1 - (2.f * y / m_image_height)), -1);
        Vector3f d = (screen_plane_pos - eye_center).normalized();

        //first node in eye path
        const Ray camera_space_ray(eye_center, d, AIR_IOR, true);
        const Ray world_camera_space_ray = camera_space_ray.transform(invViewMatrix);
        PathNode eye_node = PathNode(world_camera_space_ray.o, eye_normal_world, Vector3f(1, 1, 1),
                                 Vector3f(0, 0, 0), world_camera_space_ray, 1, 1);

        //first node in light path
        SampledLightInfo light_info = scene.sampleLight();
        const Ray init_ray(light_info.position, Vector3f(0.f, 0.f, 0.f), AIR_IOR, true);
        SampledRayInfo ray_info = SampleRay::uniformSampleHemisphere(light_info.position, init_ray, light_info.normal);
        PathNode light_node = PathNode(light_info.position, light_info.normal, Vector3f(0, 0, 0),
                                       light_info.emission, ray_info.ray, LIGHT, ray_info.prob, light_info.prob);


        //trace the paths
        std::vector<PathNode> eye_path = { eye_node };
        std::vector<PathNode> light_path = { light_node };
        tracePath(world_camera_space_ray, scene, 0, eye_path);
        tracePath(ray_info.ray, scene, 0, light_path);

        if (eye_path.size() == 1) {
            continue;
            total++;
        }

        BDPT_Samples samples = BDPT::combinePaths(scene, eye_path, light_path);
        output_radience += samples.contrib;
        total += samples.num_samples;
      }
    intensityValues[output_index] = output_radience / M_NUM_SAMPLES;
//    intensityValues[output_index] = output_radience / total;

}


//Refraction not considered in trace path yet
void PathTracer::tracePath(const Ray &ray, const Scene &scene, int depth, std::vector<PathNode> &pathNodes) {
    IntersectionInfo i;

    if(scene.getBVH().getIntersection(ray, &i, false)) {
        const Mesh * m = static_cast<const Mesh *>(i.object);//Get the mesh that was intersected
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = m->getMaterial(t->getIndex());//Get the material of the triangle from the mesh
        const tinyobj::real_t *e = mat.emission; //Emitted color
        const Vector3f normal = t->getNormal(i); //surface normal

        // Ignore all Emitted Light
        const Vector3f emitted_light = Vector3f(e[0], e[1], e[2]);
        if (emitted_light.norm() > 0) {
            Vector3f N = ray.is_in_air ? normal : -normal;
            PathNode node(i.hit, N,  Vector3f(1, 1, 1), emitted_light, ray, LIGHT, mat, 1, 1);
            pathNodes.push_back(node);
            return;
        }

        MaterialType type = BSDF::getType(mat);
        const SampledRayInfo next_ray_info = SampleRay::sampleRay(type, i.hit, ray, normal, mat);
        const Ray next_ray = next_ray_info.ray;
        const Vector3f brdf = BSDF::getBsdfFromType(ray, next_ray.d, normal, mat, type);
        const float pdf_rr = getContinueProbability(brdf);

        //test up to depth 1
        if (MathUtils::random() < pdf_rr) {
            Vector3f N = ray.is_in_air ? normal : -normal;
            PathNode node(next_ray.o, N, brdf, Vector3f(0, 0, 0), next_ray, type, mat, pdf_rr * next_ray_info.prob, 0);
            pathNodes.push_back(node);
            tracePath(next_ray, scene, depth + 1, pathNodes);
        } else {
            Vector3f N = ray.is_in_air ? normal : -normal;
            PathNode node(next_ray.o, N, brdf, Vector3f(0, 0, 0), next_ray, type, mat, (1 - pdf_rr), 0);
            pathNodes.push_back(node);
        }
    }
}
