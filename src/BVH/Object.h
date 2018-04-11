#ifndef Object_h_
#define Object_h_

#include <Eigen/Dense>

#include "IntersectionInfo.h"
#include "Ray.h"
#include "BBox.h"

struct SampledLightInfo;

#include "util/CS123SceneData.h"

struct Object {
    Object() {
        transform = Eigen::Affine3f::Identity();
    }
    virtual ~Object(){}

    //! All "Objects" must be able to test for intersections with rays.
    virtual bool getIntersection(
      const Ray& ray,
      IntersectionInfo* intersection)
    const = 0;

    //! Return an object normal based on an intersection
    virtual Eigen::Vector3f getNormal(const IntersectionInfo& I) const = 0;

    //! Return a bounding box for this object
    virtual BBox getBBox() const = 0;

    //! Return the centroid for this object. (Used in BVH Sorting)
    virtual Eigen::Vector3f getCentroid() const = 0;

    virtual bool hasLight() const = 0;
    virtual SampledLightInfo sampleLight() const = 0;

    float m_emissive_area;

    CS123SceneMaterial material;

    Eigen::Affine3f transform;
};

#endif
