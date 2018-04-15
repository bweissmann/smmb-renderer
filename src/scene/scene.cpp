#include "scene.h"

#include "shape/Sphere.h"
#include "util/MathUtils.h"

#include <util/CS123XmlSceneParser.h>

#include <util/CS123Common.h>

#include <Eigen/Geometry>

#include <iostream>

#include <Eigen/StdVector>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

using namespace Eigen;

Scene::Scene()
{
}

Scene::~Scene()
{
    for(unsigned int i = 0; i < _objects->size(); ++i) {
        Object * o = (*_objects)[i];
        delete o;
    }
    delete _objects;
    delete m_bvh;
}

bool Scene::load(QString filename, Scene **scenePointer)
{
    CS123XmlSceneParser parser(filename.toStdString());
    if(!parser.parse()) {
        return false;
    }
    CS123SceneCameraData cameraData;
    parser.getCameraData(cameraData);
    BasicCamera camera(cameraData.pos.head<3>(),
                       cameraData.look.head<3>(),
                       cameraData.up.head<3>(),
                       cameraData.heightAngle,
                       (float)IMAGE_WIDTH / (float)IMAGE_HEIGHT);
    Scene *scene = new Scene;
    scene->setCamera(camera);

    CS123SceneGlobalData globalData;
    parser.getGlobalData(globalData);
    scene->setGlobalData(globalData);

    CS123SceneLightData lightData;
    for(int i = 0, size = parser.getNumLights(); i < size; ++i) {
        parser.getLightData(i, lightData);
        scene->addLight(lightData);
    }

    QFileInfo info(filename);
    QString dir = info.path();
    CS123SceneNode *root = parser.getRootNode();
    if(!parseTree(root, scene, dir.toStdString() + "/")) {
        return false;
    }

    *scenePointer = scene;
    return true;
}

void Scene::setBVH(const BVH &bvh)
{
    m_bvh = new BVH(bvh);
}

bool Scene::parseTree(CS123SceneNode *root, Scene *scene, const std::string &baseDir)
{
    std::vector<Object *> *objects = new std::vector<Object *>;
    parseNode(root, Affine3f::Identity(), objects, baseDir);
    if(objects->size() == 0) {
        return false;
    }
    std::cout << "Parsed tree, creating BVH" << std::endl;
    BVH *bvh = new BVH(objects);

    scene->m_total_light_area = 0.f;
    // Build list of objects with lights in them
    for(unsigned int i = 0; i < objects->size(); ++i) {
        Object * o = (*objects)[i];
        if (o->hasLight()) {
            scene->_emissive_objects.push_back(i);
            scene->m_total_light_area += o->m_emissive_area;
        }
    }

    // There must be some lights in the scene
    if (scene->_emissive_objects.size() == 0) {
        std::cerr << "No lights found in the scene. Terminating." << std::endl;
        exit(1);
    }

    scene->_objects = objects;
    scene->setBVH(*bvh);
    return true;
}

void Scene::parseNode(CS123SceneNode *node, const Affine3f &parentTransform, std::vector<Object *> *objects, const std::string &baseDir)
{
    Affine3f transform = parentTransform;
    for(CS123SceneTransformation *trans : node->transformations) {
        switch(trans->type) {
        case TRANSFORMATION_TRANSLATE:
            transform = transform * Translation<float, 3>(trans->translate);
            break;
        case TRANSFORMATION_SCALE:
            transform = transform * Scaling(trans->scale);
            break;
        case TRANSFORMATION_ROTATE:
            transform = transform * AngleAxis<float>(trans->angle, trans->rotate);
            break;
        case TRANSFORMATION_MATRIX:
            transform = transform * trans->matrix;
            break;
        }
    }
    for(CS123ScenePrimitive *prim : node->primitives) {
        addPrimitive(prim, transform, objects, baseDir);
    }
    for(CS123SceneNode *child : node->children) {
        parseNode(child, transform, objects, baseDir);
    }
}

void Scene::addPrimitive(CS123ScenePrimitive *prim, const Affine3f &transform, std::vector<Object *> *objects, const std::string &baseDir)
{
    switch(prim->type) {
    case PrimitiveType::PRIMITIVE_MESH:
        std::cout << "Loading mesh " << prim->meshfile << std::endl;
        objects->push_back(loadMesh(prim->meshfile, prim->material, transform, baseDir));
        std::cout << "Done loading mesh" << std::endl;
        break;
    default:
        std::cerr << "We don't handle any other formats yet" << std::endl;
        break;
    }
}

Mesh *Scene::loadMesh(std::string filePath, const CS123SceneMaterial & material, const Affine3f &transform, const std::string &baseDir)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    QFileInfo info(QString((baseDir + filePath).c_str()));
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if(!err.empty()) {
        std::cerr << err << std::endl;
    }

    if(!ret) {
        std::cerr << "Failed to load/parse .obj file" << std::endl;
        return nullptr;
    }

    std::vector<Vector3f> vertices;
    std::vector<Vector3f> normals;
    std::vector<Vector3f> colors;
    std::vector<Vector2f> uvs;
    std::vector<int> materialIds;
    std::vector<Vector3i> faces;

    //TODO populate vectors and use tranform
    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3*idx.vertex_index+0];
                tinyobj::real_t vy = attrib.vertices[3*idx.vertex_index+1];
                tinyobj::real_t vz = attrib.vertices[3*idx.vertex_index+2];
                tinyobj::real_t nx;
                tinyobj::real_t ny;
                tinyobj::real_t nz;
                tinyobj::real_t tx;
                tinyobj::real_t ty;

                if(idx.normal_index != -1) {
                    nx = attrib.normals[3*idx.normal_index+0];
                    ny = attrib.normals[3*idx.normal_index+1];
                    nz = attrib.normals[3*idx.normal_index+2];
                } else {
                    nx = 0;
                    ny = 0;
                    nz = 0;
                }
                if(idx.texcoord_index != -1) {
                    tx = attrib.texcoords[2*idx.texcoord_index+0];
                    ty = attrib.texcoords[2*idx.texcoord_index+1];
                } else {
                    tx = 0;
                    ty = 0;
                }

                tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
                tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
                tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];

                face[v] = vertices.size();
//                vertices.push_back(transform * Vector3f(vx, vy, vz));
//                normals.push_back((transform.linear().inverse().transpose() * Vector3f(nx, ny, nz)).normalized());
                vertices.push_back(Vector3f(vx, vy, vz));
                normals.push_back(Vector3f(nx, ny, nz).normalized());
                uvs.push_back(Vector2f(tx, ty));
                colors.push_back(Vector3f(red, green, blue));
            }
            faces.push_back(face);
            materialIds.push_back(shapes[s].mesh.material_ids[f]);

            index_offset += fv;
        }
    }
    std::cout << "Loaded " << faces.size() << " faces" << std::endl;

    Mesh *m = new Mesh;
    m->init(vertices,
            normals,
            uvs,
            colors,
            faces,
            materialIds,
            materials,
            material);
    m->transform = transform.inverse();
    return m;
}

const BVH &Scene::getBVH() const
{
    return *m_bvh;
}

const BasicCamera &Scene::getCamera() const
{
    return m_camera;
}

void Scene::setCamera(const BasicCamera &camera)
{
    m_camera = camera;
}

void Scene::setGlobalData(const CS123SceneGlobalData& data)
{
    m_globalData = data;
}

void Scene::addLight(const CS123SceneLightData &data)
{
    m_lights.push_back(data);
}

const std::vector<CS123SceneLightData> &Scene::getLights()
{
    return m_lights;
}

SampledLightInfo Scene::sampleLight() const {
    int n_objects = _emissive_objects.size();
    int emissive_index = static_cast<int>(floor(MathUtils::random() * n_objects)); // Choose a random object uniformly
    if (emissive_index == n_objects) emissive_index = n_objects - 1; // bug
    int object_index = _emissive_objects[emissive_index];

    // If this object has light, get light from it
    Object *o = (*_objects)[object_index];

    SampledLightInfo i = o->sampleLight();
    i.prob *= o->m_emissive_area / m_total_light_area; // Take into account probability of choosing object i
    return i;
}
