#ifndef PATHTRACER_H
#define PATHTRACER_H

#include <QImage>

#include "scene/scene.h"
#include "bsdf.h"
#include "util/pathnode.h"

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
    const int M_NUM_SAMPLES = 100;

    /* Helpers for parallelism and logging */
    const int PARALLEL_RANGE = 50;

    void toneMap(QRgb *imageData, Eigen::Vector3f *intensityValues);

    Eigen::Vector3f traceRay(const Ray& r, const Scene &scene, int depth);

    float getContinueProbability(Eigen::Vector3f brdf);

    bool lightIsVisible(Eigen::Vector3f light_position, Eigen::Vector3f surface_position, const Scene& scene);

    Eigen::Vector3f directLightContribution(SampledLightInfo light_info, Eigen::Vector3f surface_normal, MaterialType type,
                                            Eigen::Vector3f surface_position, Ray incoming_ray, const tinyobj::material_t& mat);


    Eigen::Vector3f combinePaths(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);


    Eigen::Vector3f computePathContribution(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);

    Eigen::Vector3f computeZeroBouncePathContrib(const PathNode &eye, const PathNode &light);
    Eigen::Vector3f computePathTracingContrib(const std::vector<PathNode> &eye_path,  const PathNode &light, int max_eye_index);
    Eigen::Vector3f computeLightTracingContrib(const std::vector<PathNode> &light_path,  const PathNode &eye, int max_light_index);
    Eigen::Vector3f computeBidirectionalContrib(const std::vector<PathNode> &eye_path,  const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);

    Eigen::Vector3f computeEyeContrib(const std::vector<PathNode> &eye_path, int max_eye_index);
    Eigen::Vector3f computeLightContrib(const std::vector<PathNode> &light_path, int max_light_index);
    float getDifferentialThroughput(const PathNode &node1, const PathNode &node2);

};

#endif // PATHTRACER_H
