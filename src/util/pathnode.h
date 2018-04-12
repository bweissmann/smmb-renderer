#ifndef PATHNODE_H
#define PATHNODE_H

#include "Eigen/Dense"
#include "BVH/Ray.h"
#include "bsdf.h"


struct PathNode {

    PathNode(const Eigen::Vector4f &pos, const Eigen::Vector3f &normal, const Eigen::Vector3f &brdf,
             const Eigen::Vector3f &emission, const Ray &outgoing_ray, MaterialType type,
             const tinyobj::material_t &mat, float prob) : position(pos), surface_normal(normal), brdf(brdf),
             emission(emission), outgoing_ray(outgoing_ray), type(type), mat(*mat), prob(prob){}

    Eigen::Vector3f position;
    Eigen::Vector3f surface_normal;
    Eigen::Vector3f brdf;
    Eigen::Vector3f emission;
    Ray outgoing_ray;
    MaterialType type;
    tinyobj::material_t *mat;
    float prob;

}

//    //alternative construction
//    Eigen::Vector3f position;
//    Eigen::Vector3f surface_normal;
//    Ray outgoing_ray;


//    //computes the light flowing from light source to point on light path where
//    //eye path will meet
//    computeContrib(std::vector<PathNode> light_path, std::vector<PathNode> eye_path, int max_index, int max_eye_index) {

//        //check this equation
//        Eigen::Vector3f total_light = light_path[0].emission;
//                * light_path[0].surface_normal.dot(light_path[0].outgoing_ray.d);
//        for (int i = 1; i < max_index; i++) {
//            float cosine_phi = light_path[i - 1].outgoing_ray.d.dot(light_path[i].surface_normal);
//            total_light = total_light.cwiseProduct(light_path[i].brdf) * cosine_phi;
//        }

//        //light arriviing at eye_path point
//        Eigen::Vector3f direction = (eye_path[max_eye_index] - light_path[max_index]).normalized();
//        Ray connecting_ray(light_path[max_index].position, 1.f, true);

//        Eigen::Vector3f light_brdf;
//        total_light = total_light.cwiseProduct(light_brdf) * direction.dot(light_path[i].surface_normal);

//        //light arriving at previous point on eye_path / recompute brdf
//        light_brdf;
//        total_light = total_light.cwiseProduct(light_brdf) * direction.dot(eye_path[max_eye_index].surface_normal);

//        //each point computes radiance arriving at point i - 1.
//        for (int i = max_eye_index - 1; i > 0; i++) {

//            //reciporcity of brdf f(wi, wo) = f(wo, wi)
//            float cosine_phi = ( max_eye_index[i].outgoing_ray.d).dot(max_eye_index[i].surface_normal);
//            total_light = total_light.cwiseProduct(light_path[i].brdf) * cosine_phi;
//        }
//    }

//    computeProb(std::vector<PathNode> light_path, std::vector<PathNode> eye_path, int max_index, int max_eye_index) {
//        float light_prob = light_path[0].dirProb; //prob at point 0 on light path
//        for (int i = 1; i <= max_index; i++) {
//            float dist = (light_path[i].position - light_path[i - 1].position).norm();
//            float cosine_r = light_path[i].surface_normal.dot(light_path[i - 1].outgoing_ray.d);

//            //probability of picking direction * dist / r
//            light_prob *= light_path[i - 1].dirProb * cosine_r / dist;
//        }

//        //compute prob of eye path
//        float eye_prob = 1.f;
//        for (int i = 1; i <= max_index; i++) {
//            float dist = (eye_path[i].position - eye_path[i - 1].position).norm();
//            float cosine_r = eye_path[i].surface_normal.dot(eye_path[i - 1].outgoing_ray.d);
//            eye_prob *= eye_path[i - 1].dirProb * cosine_r / dist;
//        }

//        return eye_prob * light_prob;
//    }

//    computeDirectionalProb(Eigen::Vector3f incoming, PathNode &origin, PathNode &destination) {
//        //call specific functions
//        computeProb(type, material, incoming, outgoing);
//    }
};




#endif // PATHNODE_H
