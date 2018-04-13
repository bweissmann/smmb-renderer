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

    Eigen::Vector3f position;
    Eigen::Vector3f surface_normal;
    Eigen::Vector3f brdf;
    Eigen::Vector3f emission;
    Ray outgoing_ray;
    MaterialType type;
    tinyobj::material_t mat;
    float directional_prob;
    float point_prob;

};




#endif // PATHNODE_H
