#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"
#include "bsdf.h"

class PathTracer
{
public:
    PathTracer(int width, int image_height, int output_height, int section_id);

    void traceScene(QRgb *imageData, const Scene &scene);
    void traceSceneBackwards(QRgb *imageData, const Scene &scene);

    void tracePixel(int pixel_x, int pixel_y, const Scene& scene,
                    Eigen::Vector3f *intensityValues, const Eigen::Matrix4f &invViewMatrix);

private:
    int m_width, m_image_height, m_output_height, m_section_id;

    /* Adjust the number of samples for each pixel (N in equations) */
    const int M_NUM_SAMPLES = 1000;

    /* Helpers for parallelism and logging */
    const int PARALLEL_RANGE = 20;

    const bool RUN_WITH_MULTITHREADED = false;

    void toneMap(QRgb *imageData, Eigen::Vector3f *intensityValues);

    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, int depth);

    float getContinueProbability(Eigen::Vector3f brdf);

    bool lightIsVisible(Eigen::Vector3f light_position, Eigen::Vector3f surface_position, const Scene& scene);

    Eigen::Vector3f directLightContribution(SampledLightInfo light_info, Eigen::Vector3f surface_normal, MaterialType type,
                                            Eigen::Vector3f surface_position, Ray incoming_ray, const tinyobj::material_t& mat,
                                            const Scene &scene);

    Eigen::Vector3f directLightContribution2(SampledLightInfo light_info, Eigen::Vector3f surface_normal, MaterialType type,
                                            Eigen::Vector3f surface_position, Ray incoming_ray, const tinyobj::material_t& mat,
                                            const Scene &scene);

};

#endif // PATHTRACER_H
