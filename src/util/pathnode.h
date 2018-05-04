#ifndef PATHNODE_H
#define PATHNODE_H

#include "Eigen/Dense"
#include "BVH/Ray.h"
#include "bsdf.h"


struct PathNode {


    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &hit, const Eigen::Vector3f &left_from,
              const Eigen::Vector3f &normal, MaterialType type, const tinyobj::material_t &mat, float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), hit_position(hit), left_from(left_from), surface_normal(normal), type(type), mat(mat),
          directional_prob(directional_prob), point_prob(point_prob), radius((hit - ray.o).norm()) {}

    PathNode (const Ray &ray, const Eigen::Vector3f &contrib, const Eigen::Vector3f &hit, const Eigen::Vector3f &left_from,
              const Eigen::Vector3f &normal, MaterialType type, float directional_prob, float point_prob)
        : outgoing_ray(ray), contrib(contrib), hit_position(hit), left_from(left_from), surface_normal(normal), type(type),
          directional_prob(directional_prob), point_prob(point_prob), radius((hit - ray.o).norm()) {}


    Eigen::Vector3f surface_normal;
    Eigen::Vector3f brdf; //can probably delete
    Eigen::Vector3f emission; //can probably delete
    Ray outgoing_ray;
    MaterialType type;
    tinyobj::material_t mat;
    float directional_prob;
    float point_prob;
    Eigen::Vector3f contrib;


    //SUBSURFACE
    Eigen::Vector3f hit_position; //these would be the same for everything but hit
    Eigen::Vector3f left_from;
    float radius; //0 except for subsurface scattering

    //might need surface normal of picked point
};


#endif // PATHNODE_H
