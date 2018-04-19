#ifndef BDPT2_H
#define BDPT2_H

#include "Eigen/Dense"
#include "util/pathnode.h"
#include "scene/scene.h"

struct Sample {
    Eigen::Vector3f radiance;
    int number_samples;
};

class BDPT2
{
public:
    BDPT2();

    static Sample combinePaths(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);
    static std::vector<Eigen::Vector3f> computeEyeWeights(const std::vector<PathNode> eye_path);
    static std::vector<Eigen::Vector3f> computeLightWeights(const std::vector<PathNode> light_path);

    static bool isVisible(const Scene&scene, const Eigen::Vector3f &position1, const Eigen::Vector3f &position2);

    static float getThroughput(const Eigen::Vector3f &position1, const Eigen::Vector3f &normal1,
                                    const Eigen::Vector3f &position2, const Eigen::Vector3f &normal2);

    static Eigen::Vector3f computeRadiance(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);

    static Eigen::Vector3f computeStuff(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);
};

#endif // BDPT2_H
