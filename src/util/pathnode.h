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



#endif // PATHNODE_H
