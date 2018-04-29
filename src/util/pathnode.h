#ifndef PATHNODE_H
#define PATHNODE_H

#include "Eigen/Dense"
#include "BVH/Ray.h"
#include "bsdf.h"


struct PathNode {

    PathNode(const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, const Eigen::Vector3f &brdf,
             const Eigen::Vector3f &emission, const Ray &outgoing_ray, MaterialType type,
             const tinyobj::material_t &mat, float directional_prob, float point_prob) : position(pos), surface_normal(normal), brdf(brdf),
             emission(emission), outgoing_ray(outgoing_ray), type(type), mat(mat), directional_prob(directional_prob), point_prob(point_prob){}

    PathNode(const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, const Eigen::Vector3f &brdf,
             const Eigen::Vector3f &emission, const Ray &outgoing_ray,
             float directional_prob, float point_prob) : position(pos), surface_normal(normal), brdf(brdf),
             emission(emission), outgoing_ray(outgoing_ray), directional_prob(directional_prob), point_prob(point_prob){}

    PathNode(const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, const Eigen::Vector3f &brdf,
             const Eigen::Vector3f &emission, const Ray &outgoing_ray, MaterialType type,
             float directional_prob, float point_prob) : position(pos), surface_normal(normal), brdf(brdf),
             emission(emission), outgoing_ray(outgoing_ray), type(type), directional_prob(directional_prob), point_prob(point_prob){}



    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          const tinyobj::material_t &mat, bool next_ray_in_air, float eta, float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), position(pos), surface_normal(normal), type(type), mat(mat), next_ray_in_air(next_ray_in_air),
          eta(eta), directional_prob(directional_prob), point_prob(point_prob){}

    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          bool next_ray_in_air, float eta, float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), position(pos), surface_normal(normal), type(type), next_ray_in_air(next_ray_in_air), eta(eta),
          directional_prob(directional_prob), point_prob(point_prob){}


    Eigen::Vector3f position;
    Eigen::Vector3f surface_normal;
    Eigen::Vector3f brdf;
    Eigen::Vector3f emission;
    Ray outgoing_ray;
    MaterialType type;
    tinyobj::material_t mat;
    float directional_prob;
    float point_prob;

    Eigen::Vector3f contrib;
    bool next_ray_in_air;
    float eta;
};

struct Node {

    Node (const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          const tinyobj::material_t &mat, bool next_ray_in_air, float eta, float directional_prob, float point_prob)
        : contrib(contrib), position(pos), surface_normal(normal), type(type), mat(mat), next_ray_in_air(next_ray_in_air),
          eta(eta), directional_prob(directional_prob), point_prob(point_prob){}

    Node (const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          bool next_ray_in_air, float eta, float directional_prob, float point_prob)
        : contrib(contrib), position(pos), surface_normal(normal), type(type), next_ray_in_air(next_ray_in_air), eta(eta),
          directional_prob(directional_prob), point_prob(point_prob){}

    //precomputed
    Eigen::Vector3f contrib;

    //need for computing probabilities and connections
    Eigen::Vector3f position;
    Eigen::Vector3f surface_normal;
    MaterialType type;
    tinyobj::material_t mat;
    bool next_ray_in_air;
    float eta;
    float directional_prob;
    float point_prob;
};

struct DenoiseInfo {

    //initialize denoise info to all infinity except for radiance
    DenoiseInfo() : sample_radiance(Eigen::Vector3f(0, 0, 0)), sample_normal(Eigen::Vector3f(INFINITY, INFINITY, INFINITY)),
        sample_color(Eigen::Vector3f(INFINITY, INFINITY, INFINITY)), sample_depth(INFINITY) {}

    Eigen::Vector3f sample_radiance;
    Eigen::Vector3f sample_normal;
    Eigen::Vector3f sample_color;
    float sample_depth;
};

struct PixelInfo {
    PixelInfo(int num_samples) : num_samples(num_samples), samplesPerPixel(std::vector<DenoiseInfo>(num_samples)),
        radiance(Eigen::Vector3f(0, 0, 0)) {}

    std::vector<DenoiseInfo> samplesPerPixel;
    Eigen::Vector3f radiance;
    int num_samples;
};

struct PixelInfo2 {

//    PixelInfo2(std::vector<Eigen::Vector3f> sample_radiance, std::vector<Eigen::Vector3f> sample_normal,
//               std::vector<Eigen::Vector3f> sample_color, std::vector<float> sample_depth, int num_samples) :
//        sample_radiance(sample_radiance), sample_normal(sample_normal), sample_color(sample_color),
//        sample_depth(sample_depth), radiance(Eigen::Vector3f(0, 0, 0)), num_samples(num_samples) {}

    PixelInfo2(int num_samples) : sample_radiance(std::vector<Eigen::Vector3f>(num_samples, Eigen::Vector3f(0, 0, 0))),
                          sample_normal(std::vector<Eigen::Vector3f>(num_samples, Eigen::Vector3f(INFINITY, INFINITY, INFINITY))),
                          sample_color(std::vector<Eigen::Vector3f>(num_samples, Eigen::Vector3f(0, 0, 0))),
                          sample_depth(std::vector<float>(num_samples, INFINITY)), radiance(Eigen::Vector3f(0, 0, 0)), num_samples(num_samples) {}


    void addEmptySample() {
        sample_radiance.push_back(Eigen::Vector3f(0, 0, 0));
        sample_normal.push_back(Eigen::Vector3f(INFINITY, INFINITY, INFINITY));
        sample_color.push_back(Eigen::Vector3f(0, 0, 0));
        sample_depth.push_back(0);
        num_samples += 1;
    }

    void addInfo(const PixelInfo2 &infoToAdd) {
        sample_radiance.insert(sample_radiance.end(), infoToAdd.sample_radiance.begin(), infoToAdd.sample_radiance.end());
        sample_normal.insert(sample_normal.end(), infoToAdd.sample_normal.begin(), infoToAdd.sample_normal.end());
        sample_color.insert(sample_color.end(), infoToAdd.sample_color.begin(), infoToAdd.sample_color.end());
        sample_depth.insert(sample_depth.end(), infoToAdd.sample_depth.begin(), infoToAdd.sample_depth.end());
        num_samples += infoToAdd.num_samples;
        radiance += infoToAdd.radiance;
    }

    std::vector<Eigen::Vector3f> sample_radiance;
    std::vector<Eigen::Vector3f> sample_normal;
    std::vector<Eigen::Vector3f> sample_color;
    std::vector<float> sample_depth;
    Eigen::Vector3f radiance;
    int num_samples;

};


#endif // PATHNODE_H
