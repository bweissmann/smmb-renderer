#ifndef SCENE_H
#define SCENE_H

#include <QString>

#include "BVH/BVH.h"

#include "basiccamera.h"

#include "util/CS123SceneData.h"

#include "shape/mesh.h"

/** an object to return an emissive point light, its emission and the probability of choosing that light */
struct SampledLightInfo {
    Eigen::Vector3f position;
    Eigen::Vector3f normal;
    Eigen::Vector3f emission;
    float prob;

    SampledLightInfo(Eigen::Vector3f _position, Eigen::Vector3f _normal, Eigen::Vector3f _emission, const float _prob) :
        position(_position), normal(_normal), emission(_emission), prob(_prob) {}
};

class Scene
{
public:
    Scene();
    virtual ~Scene();

    static bool load(QString filename, Scene **scenePointer);

    void setBVH(const BVH &bvh);
    const BVH& getBVH() const;

    const BasicCamera& getCamera() const;

    void setCamera(const BasicCamera& camera);
    void setGlobalData(const CS123SceneGlobalData& data);
    void addLight(const CS123SceneLightData& data);

    const std::vector<CS123SceneLightData>& getLights();

    SampledLightInfo sampleLight() const;

private:

    BVH *m_bvh;
    std::vector<Object *> *_objects;
    std::vector<int> _emissive_objects;
    float m_total_light_area;

    BasicCamera m_camera;

    CS123SceneGlobalData m_globalData;

    std::vector<CS123SceneLightData> m_lights;

    static bool parseTree(CS123SceneNode *root, Scene *scene, const std::string& baseDir);
    static void parseNode(CS123SceneNode *node, const Eigen::Affine3f &parentTransform, std::vector<Object *> *objects, const std::string& baseDir);
    static void addPrimitive(CS123ScenePrimitive *prim, const Eigen::Affine3f &transform, std::vector<Object *> *objects, const std::string& baseDir);
    static Mesh *loadMesh(std::string filePath, const CS123SceneMaterial &material, const Eigen::Affine3f &transform, const std::string& baseDir);
};

#endif // SCENE_H
