#include "bdpt.h"
#include "iostream"

#include "pathtracer.h"

using namespace Eigen;

BDPT::BDPT() {

}

BDPT_Samples BDPT::combinePaths(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path) {
    BDPT_Samples samples = BDPT_Samples();
    int num_eye_nodes = eye_path.size();
    int num_light_nodes = light_path.size();
    samples.num_samples = 0;
    for (int i = 1; i < num_eye_nodes; i++) {
        if (eye_path[i].type == IDEAL_SPECULAR || eye_path[i].type == REFRACTION) {
            continue;
        } else if (eye_path[i].type == LIGHT) {
            samples.contrib += BDPT::computeContribution(eye_path, { eye_path[i] }, i - 1, 0) / (i);
            samples.num_samples++;
            continue;
         }
        for (int j = 0; j < num_light_nodes; j++) {
            if (BDPT::isVisible(scene, eye_path[i].position, light_path[j].position)) {
                Vector3f contrib = BDPT::computeContribution(eye_path, light_path, i, j);
                int divide = (i + j);
                float weight = computePathWeight(eye_path, light_path, i, j);
                samples.contrib += contrib / divide;
//                samples.contrib += contrib * weight;
            }
            samples.num_samples++;
        }
    }
//    samples.contrib /= (num_eye_nodes + num_light_nodes + 2);
    return samples;
}

/**
 * Given the two positions, this function checks if
 * the two positions can be connected by an edge.
 *
 * @brief BDPT::isVisible
 * @param scene
 * @param position1
 * @param position2
 * @return
 */
bool BDPT::isVisible(const Scene &scene, const Vector3f &position1, const Vector3f &position2) {
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

/**
 * Computes the contribution.
 *
 * @brief BDPT::computeContribution
 * @param eye_path
 * @param light_path
 * @param max_eye_index
 * @param max_light_index
 * @return
 */
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

/**
 * Computes the contribution for the case that is
 * equivalent to path tracing with direct lighting
 * at the end of the path.
 *
 * @brief BDPT::computePathTracingContrib
 * @param eye_path
 * @param light
 * @param max_eye_index
 * @return
 */
Vector3f BDPT::computePathTracingContrib(const std::vector<PathNode> &eye_path, const PathNode &light, int max_eye_index) {
    PathNode last_eye = eye_path[max_eye_index];
    Vector3f radiance = computePathContrib(eye_path, max_eye_index, true);
    const Vector3f direction_to_light = (light.position - last_eye.position).normalized();
    float throughput = getDifferentialThroughput(last_eye.position, last_eye.surface_normal, light.position, light.surface_normal);
    const Vector3f direct_brdf = BSDF::getBsdfFromType(eye_path[max_eye_index - 1].outgoing_ray, direction_to_light,
            last_eye.surface_normal, last_eye.mat, last_eye.type);

    Vector3f light_emission = light.emission.cwiseProduct(direct_brdf) * throughput / light.point_prob;
    radiance = radiance.cwiseProduct(light_emission);
    return radiance;
}

//TODO:: why do we not multiply by the throughput??
Vector3f BDPT::computeZeroBounceContrib(const PathNode &eye, const PathNode &light) {
    Vector3f to_light = (light.position - eye.position).normalized();
    return light.surface_normal.dot(to_light) < 0 ? light.emission : Vector3f(0,0,0);
}

Vector3f BDPT::computeLightTracingContrib(const std::vector<PathNode> &light_path, const PathNode &eye, int max_light_index) {
    return Vector3f(0, 0, 0);
}

/**
 * Computes the radiance for the combined eye and light
 * paths.
 *
 * @brief BDPT::computeBidirectionalContrib
 * @param eye_path
 * @param light_path
 * @param max_eye_index
 * @param max_light_index
 * @return
 */
Vector3f BDPT::computeBidirectionalContrib(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                           int max_eye_index, int max_light_index) {

    //contribution from separate paths
    Vector3f emission = light_path[0].emission;
    Vector3f light_path_contrib = computePathContrib(light_path, max_light_index, true);
    Vector3f eye_path_contrib = computePathContrib(eye_path, max_eye_index, true);
    light_path_contrib = light_path_contrib.cwiseProduct(emission) / light_path[0].point_prob;

    PathNode light_node = light_path[max_light_index];
    PathNode eye_node = eye_path[max_eye_index];
    Vector3f to_eye_node = (eye_node.position - light_node.position).normalized();
    Vector3f to_light_node = -1.f * to_eye_node;

//    //brdf at light node
    Ray incoming_ray = light_path[max_light_index - 1].outgoing_ray;

    Vector3f light_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_eye_node, light_node.surface_normal,
                                                     light_node.mat, light_node.type);

//    //brdf at eye node

//    incoming_ray = Ray(eye_node.position, eye_path[max_eye_index - 1] AIR_IOR, true);
    incoming_ray = eye_path[max_eye_index - 1].outgoing_ray;


    Vector3f eye_node_brdf = BSDF::getBsdfFromType(incoming_ray, to_light_node, eye_node.surface_normal,
                                                   eye_node.mat, eye_node.type);



//    //TODO:: ask about throughput
    float throughput = getDifferentialThroughput(eye_node.position, eye_node.surface_normal, light_node.position, light_node.surface_normal);
    Vector3f total_contrib = light_path_contrib.cwiseProduct(eye_path_contrib);
    total_contrib = total_contrib.cwiseProduct(light_node_brdf);
    total_contrib = total_contrib.cwiseProduct(eye_node_brdf);
    total_contrib *= throughput;
    return total_contrib;
}

/**
 * Computes the light contribution
 *
 * Computes the contribution of light from the
 * eye path. For every node in the path except
 * for the first one (the eye node),
 * the brdf is multiplied by the cosine of the outgong
 * ray with the normal and the contribution so far.
 * This product is then divided by the directional
 * probability
 * of taking that outgoing ray.
 *
 * @brief BDPT::computeEyeContrib
 * @param eye_path
 * @param max_eye_index
 * @return
 */
Vector3f BDPT::computeEyeContrib(const std::vector<PathNode> &eye_path, int max_eye_index) {
    Vector3f contrib(1, 1, 1);
    for (int i = 0; i < max_eye_index; i++) {
        PathNode node =  eye_path[i];
        float cosine_theta = node.surface_normal.dot(node.outgoing_ray.d);
        contrib = contrib.cwiseProduct(node.brdf) * cosine_theta / node.directional_prob;
    }
    return contrib;
}

/**
 * Computes the radiance that travels from
 * the light that reaches the node at
 * the index equal to max_light_index.
 *
 * @brief BDPT::computeLightContrib
 * @param light_path - light path vector
 * @param max_light_index - index of node to calculate the radiance
 * @return radiance up to node at max_light_index
 */
Vector3f BDPT::computeLightContrib(const std::vector<PathNode> &light_path, int max_light_index) {
    Vector3f contrib = light_path[0].emission / light_path[0].point_prob;
    for (int i = 0; i < max_light_index; i++) {
        PathNode node = light_path[i];
        float cosine_theta = fabsf(node.surface_normal.dot(node.outgoing_ray.d));
        contrib = contrib.cwiseProduct(node.brdf);
        contrib = contrib * cosine_theta / node.directional_prob;
    }
    return contrib;
}

Vector3f BDPT::computePathContrib(const std::vector<PathNode> &path, int max_index, bool divide_by_prob) {
    Vector3f contrib(1, 1, 1);
    for (int i = 0; i < max_index; i++) {
        PathNode node = path[i];
        contrib = contrib.cwiseProduct(node.brdf);
        if (divide_by_prob) {
            float cosine_theta = fabsf(node.surface_normal.dot(node.outgoing_ray.d));
            contrib = contrib * cosine_theta / node.directional_prob;
        }
    }
    return contrib;
}

/**
 * Computes and outputs the differential throughput
 * that connects these two positions with the
 * given normals.
 *
 * @brief BDPT::getDifferentialThroughput
 * @param position1 - position 1
 * @param normal1 - normal 1
 * @param position2 - position 2
 * @param normal2 - normal 2
 * @return differential throughput
 */
float BDPT::getDifferentialThroughput(const Vector3f &position1, const Vector3f &normal1,
                                      const Vector3f &position2, const Vector3f &normal2) {
    Vector3f direction = position2 - position1;
    float squared_dist = direction.squaredNorm();
    direction.normalize();
    float cos_node1 = fmax(0, normal1.dot(direction));
    float cos_node2 = fmax(0, normal2.dot(-1 * direction));
    float output = (cos_node1 * cos_node2) / squared_dist;
    return output;
}

float BDPT::getDifferentialThroughputButWithAbsNotMax(const Vector3f &position1, const Vector3f &normal1,
                                      const Vector3f &position2, const Vector3f &normal2) {
    Vector3f direction = position2 - position1;
    float squared_dist = direction.squaredNorm();
    direction.normalize();
    float cos_node1 = normal1.dot(direction);
    float cos_node2 = normal2.dot(-1 * direction);
    float output = fabsf(cos_node1 * cos_node2) / squared_dist;
    return output;
}


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//IGNORE THESE FUNCTIONS BELOW
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//TODO:: compute path weights
float BDPT::computePathWeight(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                 int max_eye_index, int max_light_index) {
    float true_prob = computePathProbability(eye_path, light_path, max_eye_index, max_light_index);
    int n = max_eye_index + max_light_index + 2;

    std::vector<PathNode> c_eye_path;
    std::vector<PathNode> c_light_path;

    //initialize everything as in the light path
    for (int i = 0; i <= max_light_index; i++) {
        c_light_path.push_back(light_path[i]);
    }

    for (int i = max_eye_index; i >= 0; i--) {
        c_light_path.push_back(eye_path[i]);
    }

    float sum = 0.f;
    for (int s = 0; s < n; s++) {

        float c_prob = computePathProbability(c_eye_path, c_light_path, s - 1, n - s - 1);

        sum += pow(c_prob / true_prob, 2);

        //transfer one to other path
        c_eye_path.push_back(c_light_path[c_light_path.size() - 1]);
        c_light_path.pop_back();
    }
    float output = 1.f / sum;
    return isnan(output) ? 0.f : output;
}

float BDPT::computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                   int max_eye_index, int max_light_index) {
    float eye_prob = 1.f;
    for (int i = 0; i < max_eye_index; i++) {
        PathNode node = eye_path[i];
        PathNode next = eye_path[i + 1];

        float dir_prob = node.directional_prob;
        Vector3f outgoing_direction = (next.position - node.position).normalized();
        float cosine_theta = fabsf(node.surface_normal.dot(outgoing_direction));

        if (i > 0) {
            PathNode prev = eye_path[i - 1];
            Vector3f incoming_direction = (node.position - prev.position).normalized();
            dir_prob = BSDF::getBsdfDirectionalProb(incoming_direction, outgoing_direction,
                                         node.surface_normal, node.mat, node.type, 1.f);
            dir_prob *= PathTracer::getContinueProbability(node.brdf);
        }
        float geometry_term = 1; /*getDifferentialThroughput(node.position, node.surface_normal, next.position, next.surface_normal);*/
        eye_prob *= (dir_prob * geometry_term) / cosine_theta;
    }


    float light_prob = 1.f;
    if (light_path.size() > 0) {
        light_prob = light_path[0].point_prob;
        for (int i = 0; i < max_light_index; i++) {
            PathNode node = light_path[i];
            PathNode next = light_path[i + 1];
            float dir_prob = node.directional_prob;
            Vector3f outgoing_direction = (next.position - node.position).normalized();
            float cosine_theta = fabsf(node.surface_normal.dot(outgoing_direction));

            if (i > 0) {
                PathNode prev = light_path[i - 1];
                Vector3f incoming_direction = (node.position - prev.position).normalized();
                dir_prob = BSDF::getBsdfDirectionalProb(incoming_direction, outgoing_direction,
                                             node.surface_normal, node.mat, node.type, 1.f);
                dir_prob *= PathTracer::getContinueProbability(node.brdf);
            }
            float geometry_term = 1; /* getDifferentialThroughput(node.position, node.surface_normal, next.position, next.surface_normal);*/
            light_prob *= (dir_prob * geometry_term) / cosine_theta;
        }
    }

    return light_prob * eye_prob;
}
