#ifndef PATHNODE_H
#define PATHNODE_H

#include "Eigen/Dense"
#include "BVH/Ray.h"
#include "bsdf.h"


struct PathNode {


    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          const tinyobj::material_t &mat, float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), position(pos), surface_normal(normal), type(type), mat(mat), directional_prob(directional_prob), point_prob(point_prob){}

    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &pos, const Eigen::Vector3f &normal, MaterialType type,
          float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), position(pos), surface_normal(normal), type(type),
          directional_prob(directional_prob), point_prob(point_prob){}


    Eigen::Vector3f position;
    Eigen::Vector3f surface_normal;
    Eigen::Vector3f brdf; //can probably delete
    Eigen::Vector3f emission; //can probably delete
    Ray outgoing_ray;
    MaterialType type;
    tinyobj::material_t mat;
    float directional_prob;
    float point_prob;
    Eigen::Vector3f contrib;
};


#endif // PATHNODE_H
