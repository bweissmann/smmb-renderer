#ifndef Ray_h
#define Ray_h

#include "Eigen/Dense"

#include "util/CS123Common.h"

#define AIR_IOR 1.00029

struct Ray {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Ray(const Eigen::Vector3f o, const Eigen::Vector3f d, float index_of_refraction, bool is_in_air)
        :o(o), d(d.normalized()), inv_d(Eigen::Vector3f(1, 1, 1).cwiseQuotient(d).normalized()),
          index_of_refraction(index_of_refraction), is_in_air(is_in_air){}

    Ray transform(Eigen::Matrix4f mat) const {
        Eigen::Vector4f oo = mat * vec3Tovec4(o, 1);
        Eigen::Vector4f od = mat * vec3Tovec4(d, 0);
        return Ray(oo.head<3>(), od.head<3>(), AIR_IOR, true);
    }

    Ray transform(Eigen::Affine3f transform) const {
        Eigen::Vector3f oo = transform * o;
        Eigen::Vector3f od = transform.linear().inverse().transpose() * d;
        return Ray(oo, od, AIR_IOR, true);
    }

  Eigen::Vector3f o; // Ray Origin
  Eigen::Vector3f d; // Ray Direction
  Eigen::Vector3f inv_d; // Inverse of each Ray Direction component
  float index_of_refraction; // Index of Refraction for the medium this ray is travelling through
  bool is_in_air; // Is the ray travelling through the air (used for refraction)
};

#endif
