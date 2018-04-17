#ifndef BDPT_H
#define BDPT_H

#include "Eigen/Dense"
#include "util/pathnode.h"
#include "scene/scene.h"

struct BDPT_Samples {

    BDPT_Samples() : contrib(Eigen::Vector3f(0, 0, 0)), num_samples(0) {}

    Eigen::Vector3f contrib;
    int num_samples;
};


class BDPT
{
public:
    BDPT();


    static bool isVisible(const Scene&scene, const Eigen::Vector3f &position1, const Eigen::Vector3f &position2);

    static BDPT_Samples combinePaths(const Scene&scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);

    static Eigen::Vector3f computeContribution(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);

    static Eigen::Vector3f computeZeroBounceContrib(const PathNode &eye, const PathNode &light);

    static Eigen::Vector3f computePathTracingContrib(const std::vector<PathNode> &eye_path,  const PathNode &light, int max_eye_index);

    static Eigen::Vector3f computeLightTracingContrib(const std::vector<PathNode> &light_path,  const PathNode &eye, int max_light_index);

    static Eigen::Vector3f computeBidirectionalContrib(const std::vector<PathNode> &eye_path,  const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);



    static Eigen::Vector3f computeEyeContrib(const std::vector<PathNode> &eye_path, int max_eye_index);
    static Eigen::Vector3f computeLightContrib(const std::vector<PathNode> &light_path, int max_light_index);
    static float getDifferentialThroughput(const Eigen::Vector3f &position1, const Eigen::Vector3f &normal1,
                                    const Eigen::Vector3f &position2, const Eigen::Vector3f &normal2);

    static float computePathWeight(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                        int max_eye_index, int max_light_index);
    static float computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                 int max_eye_index, int max_light_index);

};

#endif // BDPT_H
