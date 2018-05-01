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

using namespace Eigen;

PathTracer::PathTracer(int width, int image_height, int output_height, int section_id, QString name)
    : m_width(width), m_image_height(image_height), m_output_height(output_height), m_section_id(section_id), m_name(name), m_denoiser(m_width, m_image_height, 17, m_name)
{
}

void PathTracer::traceScene(const Scene& scene)
{
//    Vector3f intensityValues[m_width * m_output_height]; // Init intensity values
    PixelInfo pixelInfo[m_width * m_output_height];

    Matrix4f invViewMat = (scene.getCamera().getScaleMatrix() * scene.getCamera().getViewMatrix()).inverse();

    if (should_run_parallel) {
        // Initialize the tread pool
        QThreadPool *threadPool = QThreadPool::globalInstance();
        std::vector<RenderThread *> threads;
        threads.resize(m_width * m_output_height);

        // Setup intensity values and logging
        StatusLogger::initInstance(PARALLEL_RANGE, m_width, m_output_height); // Init status logger

        // start each thread
        for(int y = 0; y < m_output_height; y += PARALLEL_RANGE) {
            for(int x = 0; x < m_width; x += PARALLEL_RANGE) {
                int thread_index = x + (y * m_width);
                threads[thread_index] = new RenderThread;
                threads[thread_index]->setData(this, pixelInfo, scene, x, y, PARALLEL_RANGE, &invViewMat, render_type);
                threads[thread_index]->setAutoDelete(false);
                threadPool->start(threads[thread_index]);
            }
        }

        // Wait for rendering to finish
        threadPool->waitForDone();
    } else {
        for (int x = 0; x < m_width; x ++) {
            for (int y = 0; y < m_output_height; y++) {
                switch (render_type) {
                case PATH_TRACING:
                    tracePixelPT(x, y, scene, pixelInfo, invViewMat);
                    break;
                case BIDIRECTIONAL:
                    tracePixelBD(x, y, scene, pixelInfo, invViewMat);
                }
            }
        }
    }

    if (!should_denoise) {
        QImage image(m_width, m_output_height, QImage::Format_RGB32);
        QRgb *data = reinterpret_cast<QRgb *>(image.bits());
        toneMap(data, pixelInfo);
        bool success = image.save(m_name + ".png");
        if(!success) {
            success = image.save(m_name + ".png", "PNG");
        }
        if(success) {
            std::cout << "Wrote rendered image to " << (m_name + "-" + ".png").toStdString() << std::endl;
        } else {
            std::cerr << "Error: failed to write image to " << (m_name + ".png").toStdString() << std::endl;
        }
    } else {
        //TODO:: call denoise function here
        m_denoiser.init(pixelInfo);
        m_denoiser.run();
    }
}

void PathTracer::tracePixelPT(int output_x, int output_y, const Scene& scene,
                            PixelInfo *pixelInfo, const Eigen::Matrix4f &invViewMatrix)
{
    int pixel_x = output_x;
    int pixel_y = output_y + m_section_id * m_output_height;
    int output_index = output_x + output_y * m_width;

    Vector3f output_radience = Vector3f::Zero();
    Vector3f eye_center(0, 0, 0);

    PixelInfo info = PixelInfo(M_NUM_SAMPLES);
    for (int i = 0; i < M_NUM_SAMPLES; i++) {
        /* Sample an x and y randomly within the sensor square */
        float x = pixel_x + MathUtils::random() - 0.5f;
        float y = pixel_y + MathUtils::random() - 0.5f;

        Vector3f screen_plane_pos(((2.f * x / m_width) - 1), (1 - (2.f * y / m_image_height)), -1);

        Vector3f d = (screen_plane_pos - eye_center).normalized();

        const Ray camera_space_ray(eye_center, d, AIR_IOR, true);
        const Ray world_camera_space_ray = camera_space_ray.transform(invViewMatrix);

//        output_radience += traceRay(world_camera_space_ray, scene, 0);

        info.samplesPerPixel[i].sample_radiance = traceRay(world_camera_space_ray, scene, 0);
        output_radience += info.samplesPerPixel[i].sample_radiance;
    }
    info.radiance = output_radience / M_NUM_SAMPLES;
    pixelInfo[output_index] = info;
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
        if (emitted_light.norm() > 0 ) {
            if (normal.dot(ray.d) < 0 && depth == 0) {

                return emitted_light;
            } else {
                return Vector3f(0.f, 0.f, 0.f);
            }
        }

        MaterialType type = BSDF::getType(mat);
        //just return it up here
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

void PathTracer::toneMap(QRgb *imageData, PixelInfo *pixelInfo) {
    for(int y = 0; y < m_output_height; ++y) {
        for(int x = 0; x < m_width; ++x) {
            int offset = x + (y * m_width);
            Vector3f hdr_intensity = pixelInfo[offset].radiance;
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

    if (direction_to_light.dot(light_info.normal) < 0) {
        //return Vector3f(0, 0, 0);
    }

    const float distance_squared = pow((surface_position - light_info.position).norm(), 2);
    const float cos_theta = fmax(surface_normal.dot(direction_to_light), 0);
    const float cos_theta_prime = fmax(light_info.normal.dot(-direction_to_light), 0);
    const Vector3f direct_brdf = BSDF::getBsdfFromType(incoming_ray, direction_to_light, surface_normal, mat, type);

    return light_info.emission.cwiseProduct(direct_brdf) * cos_theta * cos_theta_prime / (distance_squared * light_info.prob);
}


//BIDIRECTIONAL FUNCTIONS
void PathTracer::tracePixelBD(int output_x, int output_y, const Scene& scene,
                             PixelInfo *pixelInfo, const Eigen::Matrix4f &invViewMatrix) {
    int pixel_x = output_x;
    int pixel_y = output_y + m_section_id * m_output_height;
    int output_index = output_x + output_y * m_width;

    Vector3f eye_center(0, 0, 0);
    Vector3f eye_normal_world = (invViewMatrix * Vector4f(0, 0, -1, 0)).head<3>();
    PixelInfo total_info = PixelInfo(M_NUM_SAMPLES);
    for (int i = 0; i < M_NUM_SAMPLES; i++) {

        /* Sample an x and y randomly within the sensor square */
        float x = pixel_x + MathUtils::random() - 0.5f;
        float y = pixel_y + MathUtils::random() - 0.5f;
        Vector3f screen_plane_pos(((2.f * x / m_width) - 1), (1 - (2.f * y / m_image_height)), -1);
        Vector3f d = (screen_plane_pos - eye_center).normalized();

        //first node in eye path
        const Ray camera_space_ray(eye_center, d, AIR_IOR, true);
        const Ray world_camera_space_ray = camera_space_ray.transform(invViewMatrix);
        const float cosine_theta = eye_normal_world.dot(world_camera_space_ray.d);
        PathNode eye = PathNode(world_camera_space_ray, Vector3f(1, 1, 1), world_camera_space_ray.o, eye_normal_world, EYE,
                        cosine_theta, 1.f);

        //first node in light path
        SampledLightInfo light_info = scene.sampleLight();
        const Ray init_ray(light_info.position, Vector3f(0.f, 0.f, 0.f), AIR_IOR, true);
        SampledRayInfo ray_info = SampleRay::uniformSampleHemisphere(light_info.position, init_ray, light_info.normal);
        PathNode light = PathNode(ray_info.ray, light_info.emission / light_info.prob, light_info.position, light_info.normal,
                          LIGHT, ray_info.prob, light_info.prob);


        std::vector<PathNode> eye_path = { eye };
        std::vector<PathNode> light_path = { light};
        tracePath(world_camera_space_ray, scene, 0, eye_path, Vector3f(1, 1, 1));
        tracePath(ray_info.ray, scene, 0, light_path, Vector3f(1, 1, 1));

        //the eye path contains only the eye node
        if (eye_path.size() == 1) {
            continue;
        }

        //need to remove because path might end on light
        int size = light_path.size();
        if (size > 1 && light_path[size - 1].type == LIGHT) {
            light_path.pop_back();
        }

        size = light_path.size() * (eye_path.size() - 1);
        SampleInfo sampleInfo = SampleInfo();

        //fills in sample_radiance in sampleInfo
        BDPT::combinePaths(scene, eye_path, light_path, sampleInfo);

        //get normal, depth, color for denoising information
        sampleInfo.sample_normal = eye_path[1].surface_normal;
        Vector3f emitted(eye_path[1].mat.emission[0], eye_path[1].mat.emission[2], eye_path[1].mat.emission[2]);
        if (emitted.norm() > 0) {
            sampleInfo.sample_color = emitted;
        } else {
            if (eye_path[1].type == IDEAL_DIFFUSE) {
                sampleInfo.sample_color = Vector3f(eye_path[1].mat.diffuse[0], eye_path[1].mat.diffuse[2], eye_path[1].mat.diffuse[2]);
            } else {
                sampleInfo.sample_color = Vector3f(eye_path[1].mat.specular[0], eye_path[1].mat.specular[2], eye_path[1].mat.specular[2]);
            }
        }
        sampleInfo.sample_depth = (eye_path[1].position - (Affine3f)invViewMatrix * eye_path[0].position).norm();
        total_info.samplesPerPixel[i] = sampleInfo;
        total_info.radiance += sampleInfo.sample_radiance;
      }
    total_info.radiance /= M_NUM_SAMPLES;
    pixelInfo[output_index] = total_info;
}


//THIS IS THE UPDATED TACE PATH FUNCTION THAT PRECOMPUTES RADIANCE AT EACH NODE
void PathTracer::tracePath(const Ray &ray, const Scene &scene, int depth, std::vector<PathNode> &nodes, const Vector3f &prev_brdf) {
    IntersectionInfo i;
    if (scene.getBVH().getIntersection(ray, &i, false)) {

        //compute the contribution that will arrive at this node
        int lastNodeIndex = nodes.size() - 1;
        PathNode lastNode = nodes[lastNodeIndex];
        float cosine_theta = fabsf(lastNode.surface_normal.dot(ray.d));
        Vector3f contrib = lastNode.contrib.cwiseProduct(prev_brdf) * cosine_theta / lastNode.directional_prob;

        const Mesh * m = static_cast<const Mesh *>(i.object);//Get the mesh that was intersected
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        const tinyobj::material_t& mat = m->getMaterial(t->getIndex());//Get the material of the triangle from the mesh
        const tinyobj::real_t *e = mat.emission; //Emitted color
        const Vector3f normal = t->getNormal(i); //surface normal

        //Deals with light sources
        const Vector3f emitted_light = Vector3f(e[0], e[1], e[2]);
        if (emitted_light.norm() > 0) {

            //check that the light is facing the right direction
            Vector3f N = ray.is_in_air ? normal : -normal;
            if (N.dot(ray.d) < 0) {
                if (lastNodeIndex == 0) {
                    contrib = emitted_light;
                } else {
                    float throughput = BDPT::getDifferentialThroughput(i.hit, N, lastNode.position, lastNode.surface_normal);
                    contrib = prev_brdf.cwiseProduct(lastNode.contrib.cwiseProduct(emitted_light)) * throughput;
                }
                PathNode node(ray, contrib, i.hit, N, LIGHT, mat, 1.f / (2.f * M_PI), 1.f);
                nodes.push_back(node);
            }
            return;
        }

        //get type of material
        MaterialType type = BSDF::getType(mat);
        SampledRayInfo next_ray_info = SampleRay::sampleRay(type, i.hit, ray, normal, mat);
        const Ray next_ray = next_ray_info.ray;
        const Vector3f brdf = BSDF::getBsdfFromType(ray, next_ray.d, normal, mat, type);
        float pdf_rr = getContinueProbability(brdf);

        if (MathUtils::random() < pdf_rr) {
            Vector3f N = ray.is_in_air ? normal : -normal;
            float directional_prob = next_ray_info.prob * pdf_rr;
            PathNode node(next_ray, contrib, next_ray.o, N, type, mat, directional_prob, 0);
            nodes.push_back(node);
            tracePath(next_ray, scene, depth + 1, nodes, brdf);
        } else {
            Vector3f N = ray.is_in_air ? normal : -normal;
            PathNode node(ray, contrib, next_ray.o, N, type, mat, (1 - pdf_rr), 0);
            nodes.push_back(node);
        }
    }
}

