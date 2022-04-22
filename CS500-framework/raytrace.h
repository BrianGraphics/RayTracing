///////////////////////////////////////////////////////////////////////
// A framework for a raytracer.
////////////////////////////////////////////////////////////////////////
const float PI = 3.14159f;
const float Radians = PI/180.0f;    // Convert degrees to radians

////////////////////////////////////////////////////////////////////////
// Material: encapsulates a BRDF and communication with a shader.
////////////////////////////////////////////////////////////////////////
class Material
{
 public:
    vec3 Kd, Ks, Kt;
    float alpha, GGX_alpha;
    float IOR;
    unsigned int texid;

    virtual bool isLight() { return false; }
    virtual vec3 EvalRadiance() { return Kd; }

    Material()  : Kd(vec3(1.0, 0.5, 0.0)), Ks(vec3(1,1,1)), alpha(1.0), texid(0), Kt(vec3(0.0,0.0,0.0)), IOR(1.0f) { GGX_alpha = 1.0f / sqrtf((1.0f + 2.0f) / 2.0f); }
    Material(const vec3 d, const vec3 s, const float a, const vec3 t, const float n)
        : Kd(d), Ks(s), alpha(a), Kt(t), IOR(n), texid(0) { GGX_alpha = 1.0f / sqrtf((a + 2.0f) / 2.0f); }
    Material(Material& o) { Kd = o.Kd;  Ks = o.Ks;  alpha = o.alpha; GGX_alpha = o.GGX_alpha; Kt = o.Kt; IOR = o.IOR; texid = o.texid; }

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////
// Data structures for storing meshes -- mostly used for model files
// read in via ASSIMP.
//
// A MeshData holds two lists (stl::vector) one for vertices
// (VertexData: consisting of point, normal, texture, and tangent
// vectors), and one for triangles (ivec3: consisting of three
// indices into the vertex array).
   
class VertexData
{
 public:
    vec3 pnt;
    vec3 nrm;
    vec2 tex;
    vec3 tan;
    VertexData(const vec3& p, const vec3& n, const vec2& t, const vec3& a) 
        : pnt(p), nrm(n), tex(t), tan(a) 
    {}
};

class Camera
{
public:
    Camera() = default;
    Camera(const vec3& _eye, const quat& _o, const float _ry) : eye(_eye), orientation(_o), ry(_ry) {}

    vec3 eye;
    quat orientation;
    float ry;
};

struct MeshData
{
    std::vector<VertexData> vertices;
    std::vector<ivec3> triangles;
    Material *mat;
};

////////////////////////////////////////////////////////////////////////
// Light: encapsulates a light and communiction with a shader.
////////////////////////////////////////////////////////////////////////
class Light: public Material
{
public:

    Light(const vec3 e) : Material() { Kd = e; }
    virtual bool isLight() { return true; }

    //virtual void apply(const unsigned int program);
};

////////////////////////////////////////////////////////////////////////////////
// Scene
////////////////////////////////////////////////////////////////////////////////
class Shape;
class Sphere;
class Ray;
class Sky;

#include "acceleration.h"
class BRDF;
class Scene {
public:
    int width, height;
    Camera camera;
    Material* currentMat;
    vec3 ambient;
    std::vector<Shape*> vectorOfShapes;
    Sky* sky;

    Scene();
    void Finit();

    // The scene reader-parser will call the Command method with the
    // contents of each line in the scene file.
    void Command(const std::vector<std::string>& strings,
                 const std::vector<float>& f);

    // To read a model file into the scene via ASSIMP, call ReadAssimpFile.  
    void ReadAssimpFile(const std::string& path, const mat4& M);

    // Once ReadAssimpFile parses the information from the model file,
    // it will call:
    void triangleMesh(MeshData* mesh);

    // The main program will call the TraceImage method to generate
    // and return the image.  This is the Ray Tracer!
    void TraceImage(Color* image, const int pass);

    // Find front most object
    Intersection TraceRay(Ray ray);

    Color TracePath(Ray& ray, AccelerationBvh& bvh);          
};


class BRDF {
public:
    Material mat;
    vec3 t_m;   
    vec3 _N;
    float pd;
    float pr;
    float pt;
    float ni;
    float no;
    float distance;
    float D;
    float _radicand;

public:
    vec3 SampleBrdf(const vec3 out, const vec3 N);

    float PdfBrdf(const vec3 out, const vec3 N, const vec3 in);

    vec3 EvalScattering(const vec3 out, const vec3 N, const vec3 in);

    vec3  F_factor(const float d);
    float D_factor(const vec3 m);
    float G_factor(const vec3 in, const vec3 out, const vec3 m);    
};

vec3 SampleLobe(vec3 A, float c, float phi);

float GeometryFactor(const Intersection& A, const Intersection& B);

Intersection SampleSphere(Shape* object, vec3 center, float radius);


class Sky {
public:
    int    width;
    int    height;
    float  radius;
    float  angle;
    float* pBuffer;
    float* pUDist;    
    Color* hdr;    

    void PreProcessing();

    Intersection SampleAsLight();
    float PdfAsLight(const Intersection& B) const;
    vec3 Radiance(const Intersection& A);
};


