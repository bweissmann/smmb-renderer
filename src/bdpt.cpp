#include "bdpt.h"
#include "iostream"

#include "pathtracer.h"

using namespace Eigen;

BDPT::BDPT() {

}

/**
 * Combines the eye and light paths amd computes the
 * radiance of each constructed path.
 *
 * @brief BDPT::combinePaths
 * @param scene
 * @param eye_path - nodes sampled from the eye
 * @param light_path - nodes sampled from the light
 * @param info - stores the radiance and info needed for denoising
 * @param use_multiple_importance
 */
void BDPT::combinePaths(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, SampleInfo &info, bool use_multiple_importance) {
    int num_eye_nodes = eye_path.size();
    int num_light_nodes = light_path.size();

    //fix for subsurface scattering
    use_multiple_importance = true;
    for (int i = 0; i < num_light_nodes; i++) {
        if (light_path[i].type == DIFFUSE_SCATTERING || light_path[i].type == SINGLE_SCATTERING) {
            use_multiple_importance = false;
        }
    }
    for (int i = 0; i < num_eye_nodes; i++) {
        if (eye_path[i].type == DIFFUSE_SCATTERING || eye_path[i].type == SINGLE_SCATTERING) {
            use_multiple_importance = false;
        }
    }

    for (int i = 1; i < num_eye_nodes; i++) {
        if (eye_path[i].type == LIGHT) {
            float weight = !use_multiple_importance ? 1.f / i : computePathWeight(eye_path, { eye_path[i] }, i - 1, 0);
            if (eye_path.size() == 2) {
                weight = 1.f;
            }
            Vector3f weighted_contrib = eye_path[i].contrib * weight;
            info.sample_radiance += weighted_contrib;
            continue;
         }
        for (int j = 0; j < num_light_nodes; j++) {
            if (BDPT::isVisible(scene, eye_path[i].left_from, light_path[j].left_from)) {
                Vector3f contrib = BDPT::computeContribution(eye_path, light_path, i, j);
                float weight = !use_multiple_importance ? 1.f / (i + j) : computePathWeight(eye_path, light_path, i, j);
                Vector3f weighted_contrib = contrib * weight;
                info.sample_radiance += weighted_contrib;

            }
        }
    }
}

/**
 * Takes in two positions and determines if the shadow ray
 * between them exists. If so, returns true. If not, the
 * points are occuded by another object and false is
 * returned.
 *
 * @brief BDPT::isVisible
 * @param scene - the scene
 * @param position1 - position 1
 * @param position2 - position 2
 * @return true if visible; false otherwise
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
 * Computes the radiance of the path created
 * by connecting the node at the max_light_index
 * from the light path with the node at the
 * max_eye_index from the eye path.
 *
 * @brief BDPT::computeContribution
 * @param eye_path - nodes sampled from the eye
 * @param light_path - nodes sampled from the light
 * @param max_eye_index - index of the last eye node
 * @param max_light_index - index of the last light node
 * @return radiance
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
 * Computes the radiance of the path where the
 * node at the max_eye_index is connected directly
 * with a point on the light source.
 *
 * @brief BDPT::computePathTracingContrib
 * @param eye_path - nodes sampled from the eye
 * @param light - point on the light
 * @param max_eye_index - index of the node on the eye path
 *  connected with the light
 * @return
 */
Vector3f BDPT::computePathTracingContrib(const std::vector<PathNode> &eye_path, const PathNode &light, int max_eye_index) {
    PathNode last_eye = eye_path[max_eye_index];
    Vector3f radiance = eye_path[max_eye_index].contrib;
    const Vector3f direction_to_light = (light.left_from - last_eye.left_from).normalized();
    float throughput = getDifferentialThroughput(last_eye.left_from, last_eye.surface_normal, light.left_from, light.surface_normal);

    Ray ray(last_eye.left_from, direction_to_light, AIR_IOR, true);
    const Vector3f direct_brdf = BSDF::getBsdfFromType(eye_path[max_eye_index - 1].outgoing_ray, last_eye.hit_position,  ray,
            last_eye.surface_normal, last_eye.mat, last_eye.type);
    Vector3f light_emission = light.contrib.cwiseProduct(direct_brdf) * throughput;
    radiance = radiance.cwiseProduct(light_emission);
    return radiance;
}

/**
 * Computes the contribution of directly connecting
 * the eye to light without any bounces.
 *
 * @brief BDPT::computeZeroBounceContrib
 * @param eye - eye node
 * @param light - light node
 * @return radiance
 */
Vector3f BDPT::computeZeroBounceContrib(const PathNode &eye, const PathNode &light) {
    Vector3f to_light = (light.left_from - eye.left_from).normalized();
    return light.surface_normal.dot(to_light) < 0 ? light.emission : Vector3f(0,0,0);
}

/**
 * Computes the radiance of the path constructed by
 * connecting the node sampled by the eye path at
 * the max_eye_index wih the node sampled by the light
 * path at max-light_index.
 *
 * @brief BDPT::computeBidirectionalContrib
 * @param eye_path - eye path
 * @param light_path - light path
 * @param max_eye_index - index of the node sampled by the eye to connect
 * @param max_light_index - index of the node sampled by the light to connect
 * @return radiance of the connected path
 */
Vector3f BDPT::computeBidirectionalContrib(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                           int max_eye_index, int max_light_index) {

    //contribution from separate paths
    Vector3f light_path_contrib = light_path[max_light_index].contrib;
    Vector3f eye_path_contrib = eye_path[max_eye_index].contrib;

    PathNode light_node = light_path[max_light_index];
    PathNode eye_node = eye_path[max_eye_index];
    Vector3f to_eye_node = (eye_node.left_from - light_node.left_from).normalized();
    Vector3f to_light_node = -1.f * to_eye_node;

    //brdf at light node
    Ray incoming_ray = light_path[max_light_index - 1].outgoing_ray;
    Ray outgoing_ray(light_node.left_from, to_eye_node, incoming_ray.index_of_refraction, incoming_ray.is_in_air);

    //TODO::Change surface normal to left_from_normal
    Vector3f light_node_brdf = BSDF::getBsdfFromType(incoming_ray, light_node.hit_position, outgoing_ray, light_node.surface_normal,
                                                     light_node.mat, light_node.type);

    //brdf at eye node
    incoming_ray = eye_path[max_eye_index - 1].outgoing_ray;
    outgoing_ray = Ray(eye_node.left_from, to_light_node, incoming_ray.index_of_refraction, incoming_ray.is_in_air);


    //TODO::Change surface normal to left_from_normal
    Vector3f eye_node_brdf = BSDF::getBsdfFromType(incoming_ray, eye_node.hit_position, outgoing_ray, eye_node.surface_normal,
                                                   eye_node.mat, eye_node.type);
    float throughput = getDifferentialThroughput(eye_node.left_from, eye_node.surface_normal, light_node.left_from, light_node.surface_normal);
    Vector3f total_contrib = light_path_contrib.cwiseProduct(eye_path_contrib);
    total_contrib = total_contrib.cwiseProduct(light_node_brdf);
    total_contrib = total_contrib.cwiseProduct(eye_node_brdf);
    total_contrib *= throughput;
    return total_contrib;
}

/**
 * Computes the contribution or importance
 * of the path up to the node at the max_index.
 *
 * @brief BDPT::computePathContrib
 * @param path - path to compute
 * @param max_index - last node on the path
 * @param divide_by_prob - flag if
 * @return contribution at max_index
 */
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

/**
 * Computes the weight of the path connected
 * by the node sampled from the eye at the max
 * eye index and the node sampled from the light
 * at the max light index by weighting against
 * all of the ways the path could have been
 * sampled using the power heuristic.
 *
 * @brief BDPT::computePathWeight
 * @param eye_path - nodes sampled from the eye
 * @param light_path - nodes sampled from the light
 * @param max_eye_index - index of the last node sampled by the eye
 * @param max_light_index - index of the last node sampled by the light
 * @return
 */
float BDPT::computePathWeight(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                 int max_eye_index, int max_light_index) {
    float true_prob = computePathProbability(eye_path, light_path, max_eye_index, max_light_index);
    int n = max_eye_index + max_light_index + 2;
    if (true_prob < 1e-8) {
        return 0.f;
    }
    std::vector<PathNode> c_eye_path;
    std::vector<PathNode> c_light_path;

    //initialize everything as in the light path
    for (int i = 0; i <= max_light_index; i++) {
        c_light_path.push_back(light_path[i]);
    }

    //switch the left from and hit when adding the eye info
    int index = max_light_index + 1;
    for (int i = max_eye_index; i >= 0; i--) {
        c_light_path.push_back(eye_path[i]);

        //temporarily store the hit position and swap with the left from
        Vector3f temp_hit = c_light_path[index].hit_position;
        c_light_path[index].hit_position = c_light_path[index].left_from;
        c_light_path[index].left_from = temp_hit;
        index++;
    }

    //need to switch left from and hit because now it's in reverse
    for (int i = 0; i < n; i++) {
        c_eye_path.push_back(c_light_path[n - i - 1]);
        Vector3f temp_hit = c_eye_path[i].hit_position;
        c_eye_path[i].hit_position = c_eye_path[i].left_from;
        c_eye_path[i].left_from = temp_hit;
    }

    float sum = 0.f;
    for (int s = 2; s < n; s++) {
        float c_prob = computePathProbability(c_eye_path, c_light_path, s - 1, n - 1 - s);
        sum += pow(c_prob / true_prob, 2);
    }
    float output = 1.f / sum;
    return isnan(output) ? 0.f : output;
}

/**
 * Computes the probability of sampling nodes
 * starting from the eye up to the node at
 * the max_eye_index and of sampling
 * nodes starting from the light up to the node at
 * the max_light_index.
 *
 * @brief BDPT::computePathProbability
 * @param eye_path - nodes sampled from the eye
 * @param light_path - nodes sampled from the light
 * @param max_eye_index - index of the last node in the eye path
 * @param max_light_index - index of the last node in the light path
 * @return probability of sampling these two paths
 */
float BDPT::computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                   int max_eye_index, int max_light_index) {
    float eye_prob = computeSubpathProbability(eye_path, max_eye_index);
    float light_prob = computeSubpathProbability(light_path, max_light_index);
    return light_prob * eye_prob;
}

/**
 * Computes the probability of the subpath up
 * to the max_index.
 *
 * @brief BDPT::computeSubpathProbability
 * @param subpath - path to compute probabiliy of
 * @param max_index - index of the last node in the path
 * @return probability of the path
 */
float BDPT::computeSubpathProbability(const std::vector<PathNode> &subpath, int max_index) {
    float prob = 1.f;
    if (subpath.size() > 0) {
        prob = subpath[0].point_prob;
        for (int i = 0; i < max_index; i++) {
            PathNode node = subpath[i];
            PathNode next = subpath[i + 1];

            float dir_prob = node.directional_prob;
            Vector3f outgoing_direction = (next.hit_position - node.left_from).normalized();
            float cosine_theta = fabsf(node.surface_normal.dot(outgoing_direction));

            if (i > 0) {
                PathNode prev = subpath[i - 1];
                float radius = (node.hit_position - node.left_from).norm();
                Vector3f incoming_direction = (node.hit_position - prev.left_from).normalized();

                //change normal to left_from_normal
                dir_prob = BSDF::getBsdfDirectionalProb(incoming_direction, outgoing_direction,
                                             node.surface_normal, radius, node.mat, node.type, 1.f);
                dir_prob *= PathTracer::getContinueProbability(node.brdf);
            }

            //TODO::change to left from normals
            float geometry_term = getDifferentialThroughput(node.left_from, node.surface_normal, next.hit_position, next.surface_normal);
            prob *= (dir_prob * geometry_term) / cosine_theta;
        }
    }
    return prob;
}
