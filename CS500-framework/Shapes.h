#pragma once
class Material;
class MeshData;
class VertexData;

class Shape {
public:
    Material* material;
    Intersection IntersectionRecord;
    std::vector<vec3> BoundingBoxes;
public:
    virtual Intersection Intersect(Ray ray) = 0;
};

class Box : public Shape {
public:
    Box(const vec3 base, const vec3 diag, Material* mat);
    Intersection Intersect(Ray ray);
    std::vector<Slab> slabs;

    vec3 corner;
    vec3 diagonal;
};

class Cylinder : public Shape {
public:
    Cylinder(const vec3 base, const vec3 axis, const float radius, Material* mat);
    Intersection Intersect(Ray ray);
    mat3 rotateToZ(vec3 _A, bool reverse);
    vec3 A;
    vec3 B;
    float radius;
};

class Sphere : public Shape {
public:
    Sphere(const vec3 c, const float r, Material* mat);
    Intersection Intersect(Ray ray);

public:
    vec3 center;
    float radius;
};

class Triangle : public Shape {
public:
    Triangle(std::vector<vec3> tri, std::vector<vec3> nrm);
    Intersection Intersect(Ray ray);
    std::vector<vec3> triangle;
    std::vector<vec3> normal;
};