#include "mesh.h"


#include <iostream>

#include "scene/scene.h"
#include "util/MathUtils.h"

using namespace Eigen;
using namespace std;

void Mesh::init(const std::vector<Vector3f> &vertices,
           const std::vector<Vector3f> &normals,
           const std::vector<Vector2f> &uvs,
           const std::vector<Vector3f> &colors,
           const std::vector<Vector3i> &faces,
           const std::vector<int> &materialIds,
           const std::vector<tinyobj::material_t> &materials,
           const CS123SceneMaterial &wholeObjectMaterial)
{
    _vertices = vertices;
    _normals = normals;
    _colors = colors;
    _uvs = uvs;
    _faces = faces;
    _materialIds = materialIds;
    _materials = materials;
    _wholeObjectMaterial = wholeObjectMaterial;
    calculateMeshStats();
    createMeshBVH();
}

Mesh::~Mesh()
{
    delete _meshBvh;
    delete _objects;
    delete[] _triangles;
}

bool Mesh::getIntersection(const Ray &ray, IntersectionInfo *intersection) const
{
    Ray r(ray.transform(transform));
    IntersectionInfo i;
    bool col = _meshBvh->getIntersection(r, &i, false);
    if(col) {
        intersection->t = i.t;
        intersection->object = this;
        intersection->data = i.object;

        return true;
    }
    return false;
}

Vector3f Mesh::getNormal(const IntersectionInfo &I) const
{
    return static_cast<const Object *>(I.data)->getNormal(I);
}

BBox Mesh::getBBox() const
{
    return _bbox;
}

Vector3f Mesh::getCentroid() const
{
    return _centroid;
}

const Vector3i Mesh::getTriangleIndices(int faceIndex) const
{
    return _faces[faceIndex];
}

const tinyobj::material_t &Mesh::getMaterial(int faceIndex) const
{
    return _materials[_materialIds[faceIndex]];
}

const Vector3f Mesh::getVertex(int vertexIndex) const
{
    return _vertices[vertexIndex];
}

const Vector3f Mesh::getNormal(int vertexIndex) const
{
    return _normals[vertexIndex];
}

const Vector3f Mesh::getColor(int vertexIndex) const
{
    return _colors[vertexIndex];
}

const Vector2f Mesh::getUV(int vertexIndex) const
{
    return _uvs[vertexIndex];
}

const CS123SceneMaterial Mesh::getMaterialForWholeObject() const
{
    return _wholeObjectMaterial;
}

void Mesh::calculateMeshStats()
{
    _bbox.setP(_vertices[0]);
    for(auto v : _vertices) {
        _centroid += v;
        _bbox.expandToInclude(v);
    }
    _centroid /= _vertices.size();
}

void Mesh::createMeshBVH()
{
    _triangles = new Triangle[_faces.size()];
    _objects = new std::vector<Object *>;
    _objects->resize(_faces.size());
    m_emissive_area = 0.f;
    for(unsigned int i = 0; i < _faces.size(); ++i) {
        Vector3i face = _faces[i];
        Vector3f v1 = _vertices[face(0)];
        Vector3f v2 = _vertices[face(1)];
        Vector3f v3 = _vertices[face(2)];
        Vector3f n1 = _normals[face[0]];
        Vector3f n2 = _normals[face[1]];
        Vector3f n3 = _normals[face[2]];
        _triangles[i] = Triangle(v1, v2, v3, n1, n2, n3, i);
        _triangles[i].transform = transform;
        (*_objects)[i] = &_triangles[i];

        const tinyobj::material_t &mat = getMaterial(i);

        if (mat.emission[0] > 0.01 || mat.emission[1] > 0.01 || mat.emission[2] > 0.01) {
            _emissive_triangles.push_back(i);
            m_emissive_area += _triangles[i].getArea();
        }
    }

    _meshBvh = new BVH(_objects);
}

bool Mesh::hasLight() const {
    return _emissive_triangles.size() > 0;
}

SampledLightInfo Mesh::sampleLight() const {
    int n_objects = _emissive_triangles.size();
    int emissive_index = static_cast<int>(floor(MathUtils::random() * n_objects)); // Choose a random face uniformly
    if (emissive_index == n_objects) emissive_index = n_objects - 1;
    int triangle_index = _emissive_triangles[emissive_index];

    const tinyobj::material_t &mat = getMaterial(triangle_index);
    Triangle &t = _triangles[triangle_index];

    SampledLightInfo i = t.sampleLight();
    i.emission = Vector3f(mat.emission[0], mat.emission[1], mat.emission[2]);
    i.prob *= t.m_emissive_area / m_emissive_area; // Take into account probability of choosing object i

    return i;
}
