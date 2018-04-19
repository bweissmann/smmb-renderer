#include "bdpt2.h"
#include <iostream>

using namespace Eigen;


BDPT2::BDPT2() {

}

Sample BDPT2::combinePaths(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path) {
    std::vector<Vector3f> eye_weights = computeEyeWeights(eye_path);
    std::vector<Vector3f> light_weights = computeLightWeights(light_path);
    int num_eye_nodes = eye_path.size();
    int num_light_nodes = light_path.size();

    Sample sample;
    sample.radiance = Vector3f(0, 0, 0);
    sample.number_samples = 0;

    for (int i = 1; i < num_eye_nodes; i++) {

        if (eye_path[i].type == LIGHT) {
            continue;
        }

        //going to assume 1 non-light vertex
        for (int j = 1; j < num_light_nodes; j++) {
            if (BDPT2::isVisible(scene, eye_path[i].position, light_path[j].position)) {
                Vector3f eye_contrib = eye_weights[i];
                Vector3f light_contrib = light_weights[j];

                Vector3f to_eye = (eye_path[i].position - light_path[j].position).normalized();
                Vector3f eye_brdf = BSDF::getBsdfFromType(eye_path[i-1].outgoing_ray, -1 * to_eye, eye_path[i].surface_normal, eye_path[i].mat, eye_path[i].type);
                Vector3f light_brdf = BSDF::getBsdfFromType(light_path[j-1].outgoing_ray, to_eye, light_path[j].surface_normal, light_path[j].mat, light_path[j].type);

                float throughput = getThroughput(eye_path[i].position, eye_path[i].surface_normal, light_path[j].position, light_path[j].surface_normal);

                Vector3f contrib = eye_contrib.cwiseProduct(light_contrib);
                contrib = contrib.cwiseProduct(eye_brdf);
                contrib = contrib.cwiseProduct(light_brdf);
                contrib *= throughput;
                sample.radiance += contrib;
                sample.number_samples++;
                std::cout << contrib << std::endl;
            } else {
//                std::cout << "not" << std::endl;
            }

        }
    }
    return sample;
}



Vector3f BDPT2::computeRadiance(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path) {
    Vector3f current_flux = light_path[0].emission / (light_path[0].point_prob * light_path[0].directional_prob);
    Ray current_ray = light_path[0].outgoing_ray;

    PathNode last_light_node = light_path[light_path.size() - 1];
    PathNode last_eye_node = eye_path[eye_path.size() - 1];
    PathNode second_to_last = eye_path[eye_path.size() - 2];

//    std::cout << "STARTING with light length " << light_path.size() << " and eye length " << eye_path.size() << std::endl;

    if (!isVisible(scene, last_eye_node.position, last_light_node.position)) {
        return Vector3f(0, 0, 0);
    }

//    std::cout << "is visible" << std::endl;

    for (int i = 1; i < light_path.size() - 1; i++) {
//        std::cout << "GETTING LIGHT " << i << std::endl;
        Vector3f brdf = light_path[i].brdf;
        Ray next_ray = light_path[i].outgoing_ray;
        Vector3f surface_normal = light_path[i].surface_normal;

//        float prob = BSDF::getBsdfDirectionalProb(current_ray.d,
//                next_ray.d,surface_normal,light_path[i].mat,light_path[i].type,1);
        float prob = light_path[i].directional_prob;

        Vector3f irr = current_flux * fabs(current_ray.d.dot(surface_normal));
        Vector3f rad = irr.cwiseProduct(brdf);

        current_flux = rad / prob;
        current_ray = next_ray;
    }


    // CONNECTION BETWEEN n-1 light path to N on light path
//    std::cout << "GETTING CONNECTION last light path" << std::endl;

    Vector3f outgoing_direction = (last_eye_node.position - last_light_node.position).normalized();

    Vector3f surface_normal = last_light_node.surface_normal;
    Vector3f irr = current_flux * fabs(current_ray.d.dot(surface_normal));
    Vector3f brdf = BSDF::getBsdfFromType(current_ray, outgoing_direction,
                                          surface_normal, last_light_node.mat, last_light_node.type);
    Vector3f rad = irr.cwiseProduct(brdf);

//    float prob = BSDF::getBsdfDirectionalProb(current_ray.d,
//            outgoing_direction,surface_normal,last_light_node.mat,last_light_node.type,1);
    float prob  = 1.f;

    current_flux = rad / prob;
    current_ray = Ray(last_light_node.position, outgoing_direction, AIR_IOR, true);

    // NOW we have the ray between last_light and last_eye and the flux for that ray.
//    std::cout << "GETTING CONNECTION last eye path" << std::endl;

    outgoing_direction = (second_to_last.position - last_eye_node.position).normalized();

    surface_normal = last_eye_node.surface_normal;
    irr = current_flux * fabs(current_ray.d.dot(surface_normal));
    brdf = BSDF::getBsdfFromType(current_ray, outgoing_direction,
                                      surface_normal, last_eye_node.mat, last_eye_node.type);
    rad = irr.cwiseProduct(brdf);

//    prob = BSDF::getBsdfDirectionalProb(current_ray.d,
//        outgoing_direction,surface_normal,last_eye_node.mat,last_eye_node.type,1);
    prob = last_eye_node.directional_prob;

    current_flux = rad / prob;
    current_ray = Ray(last_eye_node.position, outgoing_direction, AIR_IOR, true);

    // NOW we have the ray between last_eye and second_to_last :):)


    for (int i = eye_path.size() - 2; i > 0; i--) {
//        std::cout << "GETTING EYE " << i << std::endl;
        Vector3f brdf = eye_path[i].brdf;
        Vector3f outgoing_direction = (eye_path[i - 1].position - eye_path[i].position).normalized();
        Vector3f surface_normal = eye_path[i].surface_normal;

        //        float prob = BSDF::getBsdfDirectionalProb(current_ray.d,
        //                next_ray.d,surface_normal,light_path[i].mat,light_path[i].type,1);
        float prob = eye_path[i].directional_prob;

        Vector3f irr = current_flux * fabs(current_ray.d.dot(surface_normal));
        Vector3f rad = irr.cwiseProduct(brdf);

        current_flux = rad / prob;
        current_ray = Ray(eye_path[i].position, outgoing_direction, AIR_IOR, true);
    }


    return current_flux;
}


Vector3f BDPT2::computeStuff(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path) {
    Vector3f radiance = Vector3f(1, 1, 1);

    PathNode last_eye = eye_path[eye_path.size() - 1];
    PathNode light = light_path[0];
    int stop = eye_path.size() - 1;
    if (last_eye.type == LIGHT) {
//        if (stop == 1) {
//            return Vector3f(0, 0, 0);
//        }
//        last_eye = eye_path[stop - 1];
//        light = eye_path[stop];
//        stop = stop - 1;
//        if (stop == 1) {
//            return light.emission;
//        }
        return Vector3f(0, 0, 0);
    }

    for (int i = 1; i < stop; i++) {
        PathNode node = eye_path[i];
        float dot = node.outgoing_ray.d.dot(node.surface_normal);
        Vector3f brdf = BSDF::getBsdfFromType(eye_path[i - 1].outgoing_ray, node.outgoing_ray.d, node.surface_normal, node.mat, node.type);

        radiance = brdf.cwiseProduct(radiance) * dot / node.directional_prob;
    }

    if (!isVisible(scene, last_eye.position, light.position)) {
        return Vector3f(0, 0, 0);
    }

    const Vector3f direction_to_light = (light.position - last_eye.position).normalized();
    const float distance_squared = pow(( last_eye.position - light.position).norm(), 2);
    const float cos_theta = fmax(last_eye.surface_normal.dot(direction_to_light), 0);
    const float cos_theta_prime = fmax(light.surface_normal.dot(-direction_to_light), 0);


    const Vector3f direct_brdf = BSDF::getBsdfFromType(eye_path[eye_path.size() - 1].outgoing_ray, direction_to_light,
            last_eye.surface_normal, last_eye.mat, last_eye.type);

    Vector3f light_emission = light.emission.cwiseProduct(direct_brdf) * cos_theta * cos_theta_prime / (distance_squared * light.point_prob);

    radiance = radiance.cwiseProduct(light_emission);
    return radiance;
}



std::vector<Vector3f> BDPT2::computeEyeWeights(const std::vector<PathNode> eye_path) {
    int size = eye_path.size();
    std::vector<Vector3f> eye_weights = std::vector<Vector3f>(size);

    Vector3f contrib = Vector3f(1, 1, 1);
    eye_weights[0] = contrib / eye_path[0].point_prob;

    if (size > 1) {

        //maybe multiply by throughput
        contrib /= eye_path[0].directional_prob; //this should be 1 for a pinhole camera
        eye_weights[1] = contrib;
    }

    //compute the importance of node i based on nodes prior
    for (int i = 2; i < size; i++) {
        PathNode node = eye_path[i - 1];
//        float throughput = getThroughput(node.position, node.surface_normal,
//                                       eye_path[i - 2].position, eye_path[i - 2].surface_normal);
        float dot = node.surface_normal.dot(node.outgoing_ray.d);
        float projected_prob = node.directional_prob;
        contrib = node.brdf.cwiseProduct(contrib) * dot / (projected_prob);
        eye_weights[i] = contrib;
    }
    return eye_weights;
}

std::vector<Vector3f> BDPT2::computeLightWeights(const std::vector<PathNode> light_path) {
    int size = light_path.size();
    std::vector<Vector3f> light_weights = std::vector<Vector3f>(size);

    Vector3f contrib = Vector3f(1, 1, 1);
    light_weights[0] = contrib / light_path[0].point_prob;

    if (size > 1) {
        contrib /= light_path[0].directional_prob;
        light_weights[1] = contrib;
    }

    for (int i = 2; i < size; i++) {
        PathNode node = light_path[i - 1];
//        float throughput = getThroughput(node.position, node.surface_normal,
//                                       light_path[i - 2].position, light_path[i - 2].surface_normal);
        float dot = node.surface_normal.dot(node.outgoing_ray.d);
        float projected_prob = node.directional_prob;
        contrib = node.brdf.cwiseProduct(contrib) * dot / (projected_prob);
        light_weights[i] = contrib;

    }
    return light_weights;
}


float BDPT2::getThroughput(const Vector3f &position1, const Vector3f &normal1,
                                      const Vector3f &position2, const Vector3f &normal2) {
    Vector3f direction = position2 - position1;
    float squared_dist = direction.squaredNorm();
    direction.normalize();
    float cos_node1 = normal1.dot(direction);
    float cos_node2 = normal2.dot(-1.f * direction);

    //TODO:: why do I need this fmax part?
    return fabs(cos_node1 * cos_node2) / squared_dist;
}

bool BDPT2::isVisible(const Scene &scene, const Vector3f &position1, const Vector3f &position2) {
    float epsilon = 0.001; // Epsilon for distance to the light

    Vector3f to_position2 = (position2 - position1).normalized();
    Ray ray(position1, to_position2, AIR_IOR, true);

    IntersectionInfo i;
    if(scene.getBVH().getIntersection(ray, &i, false)) {
        const Triangle *t = static_cast<const Triangle *>(i.data);//Get the triangle in the mesh that was intersected
        if (t->getNormal(i).dot(to_position2) > 0) {
            return false;
        }
        float distance = (i.hit - position2).norm();
        return distance < epsilon;
    }

    return false;
}
