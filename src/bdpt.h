#ifndef BDPT_H
#define BDPT_H

#include "Eigen/Dense"
#include "util/pathnode.h"
#include "scene/scene.h"
#include "util/sampleinfo.h"

struct BDPT_Samples {

    BDPT_Samples() : contrib(Eigen::Vector3f(0, 0, 0)), num_samples(0) {}

    Eigen::Vector3f contrib;
    int num_samples;
};


class BDPT
{
public:
    BDPT();

    static void combinePaths2(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, PixelInfo &info, const Eigen::Matrix4f &invViewMatrix);

    static bool isVisible(const Scene&scene, const Eigen::Vector3f &position1, const Eigen::Vector3f &position2);


    static BDPT_Samples combinePaths(const Scene&scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path);
    static Eigen::Vector3f computeContribution(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);
    static Eigen::Vector3f computeZeroBounceContrib(const PathNode &eye, const PathNode &light);
    static Eigen::Vector3f computePathTracingContrib(const std::vector<PathNode> &eye_path,  const PathNode &light, int max_eye_index);
    static Eigen::Vector3f computeBidirectionalContrib(const std::vector<PathNode> &eye_path,  const std::vector<PathNode> &light_path, int max_eye_index, int max_light_index);

    static float getDifferentialThroughput(const Eigen::Vector3f &position1, const Eigen::Vector3f &normal1,
                                    const Eigen::Vector3f &position2, const Eigen::Vector3f &normal2);

    static float computePathWeight(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                        int max_eye_index, int max_light_index);
    static float computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                 int max_eye_index, int max_light_index);
    static float computeSubpathProbability(const std::vector<PathNode> &subpath, int max_index);

    static Eigen::Vector3f computePathContrib(const std::vector<PathNode> &path, int max_index, bool divide_by_prob);

    static float threePointProbability(const PathNode &point, const PathNode &pointFrom, const PathNode &prior);

    static float twoPointProbability(const PathNode &point, const PathNode &pointFrom);
};

#endif // BDPT_H
