#ifndef BDPT_H
#define BDPT_H

#include "Eigen/Dense"
#include "util/pathnode.h"
#include "scene/scene.h"
class BDPT
{
public:
    BDPT();

    static Eigen::Vector3f combinePaths(const Scene& scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);


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
