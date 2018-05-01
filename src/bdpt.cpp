#include "bdpt.h"
#include "iostream"

#include "pathtracer.h"

using namespace Eigen;

BDPT::BDPT() {

}

void BDPT::combinePaths(const Scene &scene, const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path, SampleInfo &info) { /*PixelInfo &info, const Matrix4f &invViewMatrix) {*/
    int num_eye_nodes = eye_path.size();
    int num_light_nodes = light_path.size();
    for (int i = 1; i < num_eye_nodes; i++) {
        if (eye_path[i].type == LIGHT) {
            float weight = computePathWeight(eye_path, { eye_path[i] }, i - 1, 0);
            if (eye_path.size() == 2) {
                weight = 1.f;
            }
            Vector3f weighted_contrib = eye_path[i].contrib * weight;
            info.sample_radiance += weighted_contrib;
            continue;
         }
        for (int j = 0; j < num_light_nodes; j++) {
            if (BDPT::isVisible(scene, eye_path[i].position, light_path[j].position)) {
                Vector3f contrib = BDPT::computeContribution(eye_path, light_path, i, j);
                float weight = computePathWeight(eye_path, light_path, i, j);
                Vector3f weighted_contrib = contrib * weight;
                info.sample_radiance += weighted_contrib;
            }
        }
    }
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
    Vector3f radiance = eye_path[max_eye_index].contrib;
    const Vector3f direction_to_light = (light.position - last_eye.position).normalized();
    float throughput = getDifferentialThroughput(last_eye.position, last_eye.surface_normal, light.position, light.surface_normal);
    const Vector3f direct_brdf = BSDF::getBsdfFromType(eye_path[max_eye_index - 1].outgoing_ray, direction_to_light,
            last_eye.surface_normal, last_eye.mat, last_eye.type);
    Vector3f light_emission = light.contrib.cwiseProduct(direct_brdf) * throughput;
    radiance = radiance.cwiseProduct(light_emission);
    return radiance;
}

//TODO:: why do we not multiply by the throughput??
Vector3f BDPT::computeZeroBounceContrib(const PathNode &eye, const PathNode &light) {
    Vector3f to_light = (light.position - eye.position).normalized();
    return light.surface_normal.dot(to_light) < 0 ? light.emission : Vector3f(0,0,0);
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
    Vector3f light_path_contrib = light_path[max_light_index].contrib;
    Vector3f eye_path_contrib = eye_path[max_eye_index].contrib;

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
    for (int i = max_eye_index; i >= 0; i--) {
        c_light_path.push_back(eye_path[i]);
    }
    for (int i = 0; i < n; i++) {
        c_eye_path.push_back(c_light_path[n - i - 1]);
    }

    float sum = 0.f;
    for (int s = 2; s < n; s++) {
        float c_prob = computePathProbability(c_eye_path, c_light_path, s - 1, n - 1 - s);
        sum += pow(c_prob / true_prob, 2);
    }
    float output = 1.f / sum;
    return isnan(output) ? 0.f : output;
}

float BDPT::threePointProbability(const PathNode &point, const PathNode &pointFrom, const PathNode &prior) {
    Vector3f outgoing = (point.position - pointFrom.position).normalized();
    Vector3f incoming = (pointFrom.position - prior.position).normalized();
    float brdf_prob = BSDF::getBsdfDirectionalProb(incoming, outgoing, pointFrom.surface_normal, pointFrom.mat, pointFrom.type, 1.f);
    float throughput = getDifferentialThroughput(point.position, point.surface_normal, pointFrom.position, pointFrom.surface_normal);
    float cosine_theta = fabsf(pointFrom.surface_normal.dot(outgoing));
    return brdf_prob * throughput / cosine_theta;
}

float BDPT::twoPointProbability(const PathNode &point, const PathNode &pointFrom) {
    Vector3f outgoing = (point.position - pointFrom.position).normalized();
    float dir_prob = pointFrom.directional_prob;
    float throughput = getDifferentialThroughput(point.position, point.surface_normal, pointFrom.position, pointFrom.surface_normal);
    float cosine_theta = fabsf(pointFrom.surface_normal.dot(outgoing));
    return dir_prob * throughput / cosine_theta;
}

/**
 * @brief BDPT::computePathProbability
 * @param eye_path
 * @param light_path
 * @param max_eye_index
 * @param max_light_index
 * @return
 */
float BDPT::computePathProbability(const std::vector<PathNode> &eye_path, const std::vector<PathNode> &light_path,
                                   int max_eye_index, int max_light_index) {
    float eye_prob = computeSubpathProbability(eye_path, max_eye_index);
    float light_prob = computeSubpathProbability(light_path, max_light_index);
    return light_prob * eye_prob;
}

/**
 * @brief BDPT::computeSubpathProbability
 * @param subpath
 * @param max_index
 * @return
 */
float BDPT::computeSubpathProbability(const std::vector<PathNode> &subpath, int max_index) {
    float prob = 1.f;
    if (subpath.size() > 0) {
        prob = subpath[0].point_prob;
        for (int i = 0; i < max_index; i++) {
            PathNode node = subpath[i];
            PathNode next = subpath[i + 1];

            float dir_prob = node.directional_prob;
            Vector3f outgoing_direction = (next.position - node.position).normalized();
            float cosine_theta = fabsf(node.surface_normal.dot(outgoing_direction));

            if (i > 0) {
                PathNode prev = subpath[i - 1];
                Vector3f incoming_direction = (node.position - prev.position).normalized();
                dir_prob = BSDF::getBsdfDirectionalProb(incoming_direction, outgoing_direction,
                                             node.surface_normal, node.mat, node.type, 1.f);
                dir_prob *= PathTracer::getContinueProbability(node.brdf);
            }
            float geometry_term = getDifferentialThroughput(node.position, node.surface_normal, next.position, next.surface_normal);
            prob *= (dir_prob * geometry_term) / cosine_theta;
        }
    }
    return prob;
}
