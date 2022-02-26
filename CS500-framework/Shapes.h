#pragma once
class Material;
class MeshData;
class VertexData;
class Slab;
class SimpleBox;

class Shape {
public:
    Material* material;
    SimpleBox boundingBox;
public:
    virtual bool Intersect(Ray ray, Intersection& record) = 0;
};

class Box : public Shape {
public:
    Box(const vec3 base, const vec3 diag, Material* mat);
    bool Intersect(Ray ray, Intersection& record);
    std::vector<Slab> slabs;

    vec3 corner;
    vec3 diagonal;
};

class Cylinder : public Shape {
public:
    Cylinder(const vec3 base, const vec3 axis, const float r, Material* mat);
    bool Intersect(Ray ray, Intersection& record);
    mat3 rotateToZ(vec3 _A, bool reverse);
    vec3 A;
    vec3 B;
    float radius;
};

class Sphere : public Shape {
public:
    Sphere(const vec3 c, const float r, Material* mat);
    bool Intersect(Ray ray, Intersection& record);

public:
    vec3 center;
    float radius;
};

class Triangle : public Shape {
public:
    Triangle(std::vector<vec3> tri, std::vector<vec3> nrm);
    bool Intersect(Ray ray, Intersection& record);
    std::vector<vec3> triangle;
    std::vector<vec3> normal;
};