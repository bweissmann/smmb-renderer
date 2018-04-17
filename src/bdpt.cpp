#include "bdpt.h"

using namespace Eigen;

BDPT::BDPT() {

}

Vector3f combinePaths(const Scene& scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path) {
    int num_eye_nodes = eye_path.size();
    int num_light_nodes = light_path.size();
    Vector3f weighted_contribution(0, 0, 0);
    for (int i = 0; i < num_eye_nodes; i++) {
        int max_eye_index = i;
        if (eye_path[i].type == IDEAL_SPECULAR || eye_path[i].type == REFRACTION) {
            continue;
        }
        for (int j = 0; j < num_light_nodes; j++) {

            //do I need to check if connection point is also a specular surface?

//            if (lightIsVisible(eye_path[i].position, light_path[j].position, scene)) {

                Vector3f contrib = BDPT::computeContribution(eye_path, light_path, max_eye_index, j);
                float weight = BDPT::computePathWeight(eye_path, light_path, max_eye_index, j);
                weighted_contribution += weight * contrib;

//            }
        }
    }
    return weighted_contribution;
}


Vector3f BDPT::computeContribution(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                   int max_eye_index, int max_light_index) {

    if (max_eye_index == 0 && max_light_index == 0) {
        return BDPT::computeZeroBounceContrib(eye_path[0], light_path[0]);
    } else if (max_eye_index > 0 && max_light_index == 0) {
        return BDPT::computePathTracingContrib(eye_path, light_path[0], max_eye_index);
    } else if (max_eye_index > 0 && max_light_index > 0) {
        return BDPT::computeBidirectionalContrib(eye_path, light_path, max_eye_index, max_light_index);
    }
    return Vector3f(0, 0, 0);

}


Vector3f BDPT::computeZeroBounceContrib(const PathNode &eye, const PathNode &light) {

    //TODO:: question about G(x <-> x') when x is eye -> must give normal for to eye?
    //TODO:: add probability calculation?

//    float throughput = BDPT::getDifferentialThroughput(eye.position, eye.surface_normal, light.position, light.surface_normal);
    return light.emission; /** throughput;*/
}


Vector3f BDPT::computePathTracingContrib(const std::vector<PathNode> &eye_path, const PathNode &light, int max_eye_index) {

    //TODO::check if need probability with respect to area

    Vector3f contrib = computeEyeContrib(eye_path, max_eye_index);
    PathNode max_eye_node = eye_path[max_eye_index];
    PathNode previous_eye_node = eye_path[max_eye_index - 1];
    Vector3f direction = (light.position - max_eye_node.position).normalized();
    Vector3f brdf = BSDF::getBsdfFromType(previous_eye_node.outgoing_ray, direction, max_eye_node.surface_normal,
                          max_eye_node.mat, max_eye_node.type);
    float throughput = getDifferentialThroughput(max_eye_node.position, max_eye_node.surface_normal,
                                                 light.position, light.surface_normal);
    contrib = contrib.cwiseProduct(brdf);
    contrib = contrib.cwiseProduct(light.emission) * throughput / light.point_prob;
    return contrib;
}


Vector3f BDPT::computeLightTracingContrib(const std::vector<PathNode> &light_path, const PathNode &eye, int max_light_index) {
    return Vector3f(0, 0, 0);
}


Vector3f BDPT::computeBidirectionalContrib(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                           int max_eye_index, int max_light_index) {

    //contribution from separate paths
    Vector3f light_path_contrib = computeLightContrib(light_path, max_light_index);
    Vector3f eye_path_contrib = computeEyeContrib(eye_path, max_eye_index);

    PathNode light_node = light_path[max_light_index];
    PathNode eye_node = eye_path[max_eye_index];
    Vector3f to_eye_node = (eye_node.position - light_node.position).normalized();
    Vector3f to_light_node = -1.f * to_eye_node;

    //brdf at light node
    Ray incoming_ray = light_path[max_light_index - 1].outgoing_ray;
    Vector3f light_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_eye_node, light_node.surface_normal,
                                                     light_node.mat, light_node.type);

    //brdf at eye node
    incoming_ray = eye_path[max_eye_index - 1].outgoing_ray;
    Vector3f eye_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_light_node, eye_node.surface_normal,
                                                   eye_node.mat, eye_node.type);

    float throughput = getDifferentialThroughput(eye_node.position, eye_node.surface_normal, light_node.position, light_node.surface_normal);
    Vector3f total_contrib = light_path_contrib.cwiseProduct(eye_path_contrib);
    total_contrib = total_contrib.cwiseProduct(light_node_brdf);
    total_contrib = total_contrib.cwiseProduct(eye_node_brdf);
    total_contrib *= throughput;
    return total_contrib;


}



Vector3f BDPT::computeEyeContrib(const std::vector<PathNode> &eye_path, int max_eye_index) {
    Vector3f contrib(1, 1, 1);

    //TODO::check if need probability with respect to area
    //TODO:: do I need to deal with probability of picking certain direction.
    for (int i = 1; i < max_eye_index; i++) {
        PathNode node =  eye_path[i];
        contrib = contrib.cwiseProduct(node.brdf) * node.surface_normal.dot(node.outgoing_ray.d) / node.directional_prob;
    }
    return contrib;
}

Vector3f BDPT::computeLightContrib(const std::vector<PathNode> &light_path, int max_light_index) {

    //TODO::check if need probability with respect to area
    Vector3f contrib = light_path[0].emission / light_path[0].point_prob; //going to be direcitonal prob
    for (int i = 1; i < max_light_index; i++) {
        PathNode light_node = light_path[i];

        //TODO:: check that I am multiplying by the right constant in light-tracing
        float constant = light_node.surface_normal.dot(light_node.outgoing_ray.d) / light_node.directional_prob;
        contrib = contrib.cwiseProduct(light_node.brdf) * constant;
    }
    return contrib;
}

float BDPT::getDifferentialThroughput(const Vector3f &position1, const Vector3f &normal1,
                                      const Vector3f &position2, const Vector3f &normal2) {
    Vector3f direction = position2 - position1;
    float squared_dist = direction.squaredNorm();
    direction.normalize();

    //check this for throughput
    float cos_node1 = normal1.dot(direction);
    float cos_node2 = normal2.dot(-1.f * direction);
    return fabsf(cos_node1 * cos_node2) / squared_dist;
}

float BDPT::computePathWeight(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                 int max_eye_index, int max_light_index) {
    float combined_prob = 1.f;
    float prob_sum = 0.f;

    const PathNode *node = &eye_path[max_eye_index];
    const PathNode *previous = &eye_path[max_eye_index - 1];

    for (int i = max_light_index; i > 0; i++) {
        Vector3f from_old_node = (node->position - previous->position).normalized();
        Vector3f to_new_node = (light_path[i].position - node->position).normalized();

        float prob = BSDF::getBsdfDirectionalProb(from_old_node, to_new_node, node->surface_normal, node->mat, node->type, 1.f);
        float prob_to_light_node = light_path[i - 1].directional_prob;

        combined_prob *= powf(prob/prob_to_light_node, 2);
        prob_sum += combined_prob;
        previous = node;
        node = &light_path[i];
    }

    combined_prob = 1.f;
    node = &light_path[max_light_index];
    previous = &light_path[max_eye_index - 1];
    for (int i = max_eye_index; i > 0; i++) {
        Vector3f from_old_node = (node->position - previous->position).normalized();
        Vector3f to_new_node = (eye_path[i].position - node->position).normalized();
        float prob = BSDF::getBsdfDirectionalProb(from_old_node, to_new_node, node->surface_normal, node->mat, node->type, 1.f);
        float prob_to_eye_node = eye_path[i - 1].directional_prob;
        combined_prob *= powf(prob_to_eye_node / prob, 2);
        prob_sum += combined_prob;
        previous = node;
        node = &light_path[i];
    }

    //TODO::what to do for paths where light = 0 or eye = 0 (as max)
    return 1.f / prob_sum;

}

float BDPT::computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                   int max_eye_index, int max_light_index) {

    float eye_prob = 1.f; //probability of picking eye point is 1

    //probability at point i is probability of outgoing directon at i - 1 * cosine / distance squared
    for (int i = 1; i <= max_eye_index; i++) {
        Vector3f connecting_ray = eye_path[i - 1].position - eye_path[i].position;
        float squared_dist = connecting_ray.squaredNorm();
        connecting_ray.normalized();
        float cosine_r = eye_path[i].surface_normal.dot(connecting_ray);
        eye_prob *= eye_path[i - 1].directional_prob * cosine_r / squared_dist;
    }

    float light_prob = light_path[0].point_prob; //probability of selecting that point
    for (int i = 1; i <= max_light_index; i++) {
        Vector3f connecting_ray = light_path[i - 1].position - light_path[i].position;
        float squared_dist = connecting_ray.squaredNorm();
        connecting_ray.normalized();
        float cosine_r = light_path[i].surface_normal.dot(connecting_ray);
        light_prob *= light_path[i - 1].directional_prob * cosine_r / squared_dist;
    }

    //TODO:: what about connecting probabilities?

    return eye_prob * light_prob;
}
