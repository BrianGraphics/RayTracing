//////////////////////////////////////////////////////////////////////
// Provides the framework for a raytracer.
////////////////////////////////////////////////////////////////////////

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
#endif


#include "geom.h"
#include "Intersect.h"
#include "acceleration.h"
#include "Shapes.h"
#include "raytrace.h"



#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// A good quality *thread-safe* Mersenne Twister random number generator.
#include <random>
std::random_device device;
std::mt19937_64 RNGen(device());
std::uniform_real_distribution<> myrandom(0.0, 1.0);
// Call myrandom(RNGen) to get a uniformly distributed random number in [0,1].

Scene::Scene() 
{ 
    //realtime = new Realtime(); 
}

void Scene::Finit()
{
}

void Scene::triangleMesh(MeshData* mesh) 
{ 
    for (int i = 0; i < mesh->triangles.size(); ++i)
    {
        std::vector<vec3> tri, nrm;
        ivec3 index = mesh->triangles[i];
        tri.push_back(mesh->vertices[index.x].pnt);
        nrm.push_back(mesh->vertices[index.x].nrm);
        tri.push_back(mesh->vertices[index.y].pnt);
        nrm.push_back(mesh->vertices[index.y].nrm);
        tri.push_back(mesh->vertices[index.z].pnt);
        nrm.push_back(mesh->vertices[index.z].nrm);
        Shape* triangle = new Triangle(tri, nrm);
        triangle->material = mesh->mat;
        vectorOfShapes.push_back(triangle);       
    }
}

quat Orientation(int i, 
                        const std::vector<std::string>& strings,
                        const std::vector<float>& f)
{
    quat q(1,0,0,0); // Unit quaternion
    while (i<strings.size()) {
        std::string c = strings[i++];
        if (c == "x")  
            q *= angleAxis(f[i++]*Radians, Xaxis());
        else if (c == "y")  
            q *= angleAxis(f[i++]*Radians, Yaxis());
        else if (c == "z")  
            q *= angleAxis(f[i++]*Radians, Zaxis());
        else if (c == "q")  {
            q *= quat(f[i+0], f[i+1], f[i+2], f[i+3]);
            i+=4; }
        else if (c == "a")  {
            q *= angleAxis(f[i+0]*Radians, normalize(vec3(f[i+1], f[i+2], f[i+3])));
            i+=4; } }
    return q;
}

void Scene::Command(const std::vector<std::string>& strings,
                    const std::vector<float>& f)
{
    if (strings.size() == 0) return;
    std::string c = strings[0];
    
    if (c == "screen") {
        // syntax: screen width height
        width = int(f[1]);
        height = int(f[2]); }

    else if (c == "camera") {
        // syntax: camera x y z   ry   <orientation spec>
        // Eye position (x,y,z),  view orientation (qw qx qy qz),  frustum height ratio ry
        camera = Camera(vec3(f[1],f[2],f[3]), Orientation(5,strings,f), f[4]); }

    else if (c == "ambient") {
        // syntax: ambient r g b
        // Sets the ambient color.  Note: This parameter is temporary.
        // It will be ignored once your raytracer becomes capable of
        // accurately *calculating* the true ambient light.
        ambient = vec3(f[1], f[2], f[3]);
    }

    else if (c == "brdf")  {
        // syntax: brdf  r g b   r g b  alpha
        // later:  brdf  r g b   r g b  alpha  r g b ior
        // First rgb is Diffuse reflection, second is specular reflection.
        // third is beer's law transmission followed by index of refraction.
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7]); }

    else if (c == "light") {
        // syntax: light  r g b   
        // The rgb is the emission of the light
        // Creates a Material instance to be picked up by successive shapes
        currentMat = new Light(vec3(f[1], f[2], f[3])); 
        //Lights.push_back(currentMat);
    }
   
    else if (c == "sphere") {
        // syntax: sphere x y z   r
        // Creates a Shape instance for a sphere defined by a center and radius
        //realtime->sphere(vec3(f[1], f[2], f[3]), f[4], currentMat); 
        Shape* sphere = new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
        vectorOfShapes.push_back(sphere);

        if (currentMat->isLight()) {
            light = new Sphere(vec3(f[1], f[2], f[3]), f[4], currentMat);
            lightPos = vec3(f[1], f[2], f[3]);
        }
    }

    else if (c == "box") {
        // syntax: box bx by bz   dx dy dz
        // Creates a Shape instance for a box defined by a corner point and diagonal vector
        Shape* box = new Box(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), currentMat);
        vectorOfShapes.push_back(box);
    }

    else if (c == "cylinder") {
        // syntax: cylinder bx by bz   ax ay az  r
        // Creates a Shape instance for a cylinder defined by a base point, axis vector, and radius
        Shape* cylinder = new Cylinder(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], currentMat);
        vectorOfShapes.push_back(cylinder);
    }


    else if (c == "mesh") {
        // syntax: mesh   filename   tx ty tz   s   <orientation>
        // Creates many Shape instances (one per triangle) by reading
        // model(s) from filename. All triangles are rotated by a
        // quaternion (qw qx qy qz), uniformly scaled by s, and
        // translated by (tx ty tz) .
        mat4 modelTr = translate(vec3(f[2],f[3],f[4]))
                          *scale(vec3(f[5],f[5],f[5]))
                          *toMat4(Orientation(6,strings,f));
        ReadAssimpFile(strings[1], modelTr);  }

    
    else {
        fprintf(stderr, "\n*********************************************\n");
        fprintf(stderr, "* Unknown command: %s\n", c.c_str());
        fprintf(stderr, "*********************************************\n\n");
    }
}

void Scene::TraceImage(Color* image, const int pass)
{
    float rx = camera.ry * static_cast<float>(width) / static_cast<float>(height);
    float dx = 0.0f, dy = 0.0f;
    const float rr = 0.8f;
    const vec3 X = rx * transformVector(camera.orientation, Xaxis());
    const vec3 Y = camera.ry * transformVector(camera.orientation, Yaxis());
    const vec3 Z = transformVector(camera.orientation, Zaxis());

    AccelerationBvh bvh(vectorOfShapes);
    Color* tmp = new Color[width * height];
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            tmp[y * width + x] = Color(0, 0, 0);

    for (int i = 0; i < pass; ++i) {       
        #pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
        for (int y = 0; y < height; y++) {
            fprintf(stderr, "Rendering %4d\r", y);
            for (int x = 0; x < width; x++) {               
                dx = 2 * (x + myrandom(RNGen)) / width - 1.0f;
                dy = 2 * (y + myrandom(RNGen)) / height - 1.0f;
                Ray ray(camera.eye, normalize(dx * X + dy * Y - Z));
                int index = y * width + x;
                vec3 color = TracePath(ray, bvh);
                if (!glm::all(glm::isinf(color))) tmp[index] += color;
                if (myrandom(RNGen) <= rr) image[index] = tmp[index] / static_cast<float>(pass);
            }
        }

        fprintf(stderr, "\n");
    }
}

Intersection Scene::TraceRay(Ray ray) {
    Intersection front, current;
    front.t = std::numeric_limits<float>::infinity();
    front.isIntersect = false;
    for (auto shape : vectorOfShapes) {     
        if (shape->Intersect(ray, current))
            if (front.t > current.t)
                front = current;
    }

    return front;
}

Color Scene::TracePath(Ray& ray, AccelerationBvh& bvh)
{
    Color C = Color(0.0f, 0.0f, 0.0f);
    vec3 W = vec3(1.0f, 1.0f, 1.0f);
    vec3 N(0.0f), Wi(0.0f), Wo(0.0f), f(0.0f);
    vec3 m(0.0f);
    float p = 0.0f;
    float s = 0.0f, p_diffuse = 0.0f, p_reflection = 0.0f;
    float alpha = 0.0f;
    const float rr = 0.8f;    
    Intersection P, Q, L;

    // get closest point
    P = bvh.intersect(ray);   

    // hit nothing so return nothing
    if (!P.isIntersect) return C;

    // hit light so return light
    if (P.shape->material->isLight()) {
        return P.shape->material->Kd; // return EvalRadiance(P)
    }

    // init
    N = P.N;
    Wo = -ray.D;

    // extend ray
    while (myrandom(RNGen) <= rr) {        
        //Explicit light connection
        L = SampleSphere(light, light->center, light->radius);        
        Wi = normalize(L.P - P.P);
        const Ray new_ray1(P.P, Wi);

        Intersection I = bvh.intersect(new_ray1);
        if (I.isIntersect) {
            p = (1 / (4 * PI * light->radius * light->radius)) / GeometryFactor(P, L);
            if (p >= 0.000001f) {
                if (I.shape == L.shape) {
                    f = EvalScattering(Wo, N, Wi, *P.shape->material);
                    C += W * (f / p) * I.shape->material->EvalRadiance();
                }
            }
        }

        // Extend path
        float r1 = myrandom(RNGen), r2 = myrandom(RNGen);
        r2 *= 2.0f * PI;

        // probability
        p_diffuse    = length(P.shape->material->Kd);
        p_reflection = length(P.shape->material->Ks);
        s = p_diffuse + p_reflection;
        p_diffuse    /= s;
        p_reflection /= s;

        

        // roughness
        alpha = sqrtf(2.0f / (P.shape->material->alpha + 2.0f));

        // SampleBRDF, choose random direction
        if (myrandom(RNGen) < p_diffuse) { // choice = diffuse        
            r1 = sqrtf(r1);            
            Wi = SampleLobe(N, r1, r2);
        }
        else {  // choice = specular
            r1 = cos(atan(alpha * sqrtf(r1) / sqrtf(1 - r1)));
            //r1 = cos(atan( sqrtf(-alpha * alpha * log(1 - r1))));
            m = SampleLobe(N, r1, r2);
            Wi = 2 * fabsf(dot(Wo, m)) * m - Wo;
            Wi = normalize(Wi);
        }
               
        const Ray new_ray2(P.P, Wi);

        Q = bvh.intersect(new_ray2);
        if (!Q.isIntersect) break;

        f = EvalScattering(Wo, N, Wi, *P.shape->material);
        p = PdfBrdf(Wo, N, Wi, alpha, p_diffuse, p_reflection) * rr;
        if (p < 0.000001f) break;
                
        W *= f / p;

        if (Q.shape->material->isLight()) {                  
            C += W * Q.shape->material->EvalRadiance();            
            break;
        }

        P = Q;    
        N = P.N;
        Wo = -Wi;
    }

    //if (!glm::all(glm::isinf(C))) C = vec3(0.0f);
    return C;
}

vec3 SampleLobe(vec3 A, float c, float phi) {
    float s = 0.0f;
    vec3 K(0.0f), B(0.0f), C(0.0f);

    s = sqrtf(1.0f - (c * c));
    K = vec3(s * cosf(phi), s * sinf(phi), c);

    if (fabsf(A.z - 1.0f) < 0.001f)
        return K;
    else if (fabsf(A.z + 1.0f) < 0.001f)
        return vec3(K.x, -K.y, -K.z);

    B = normalize(vec3(-A.y, A.x, 0.0f));
    C = cross(A, B);

    return K.x * B + K.y * C + K.z * A;
}

Intersection SampleSphere(Shape* object, vec3 center, float radius)
{
    float r1 = myrandom(RNGen), r2 = myrandom(RNGen);
    float z = 2.0f * r1 - 1.0f;
    float r = sqrtf(1 - z * z);
    float a = 2.0f * PI * r2;
    Intersection ret;
    ret.N = normalize(vec3(r * cosf(a), r * sinf(a), z));
    ret.P = center + radius * ret.N;
    ret.shape = object;
    ret.isIntersect = true;
    return ret;
}

float GeometryFactor(const Intersection& A, const Intersection& B)
{
    vec3 D = A.P - B.P;
    float DD = dot(D, D);
    return fabsf(dot(A.N, D) * dot(B.N, D) / (DD * DD));
}

float PdfBrdf(vec3 out, vec3 N, vec3 in, float alpha, float pd, float pr) {
    vec3 m(0.0f);
    float Pd = 0.0f, Pr = 0.0f, D_term = 0.0f, r1 = myrandom(RNGen);
    float NdotM = 0.0f, alpha_square = alpha * alpha;    

    m = normalize(out + in);
    NdotM = dot(N, m);
    if (NdotM > 0.000001f) {
        const float tanM = sqrtf(1.0f - NdotM * NdotM) / NdotM;
        D_term = alpha_square / PI / (NdotM * NdotM * NdotM * NdotM) / (alpha_square + tanM * tanM) / (alpha_square + tanM * tanM);
    }

    Pd = fabsf(dot(in, N)) / PI;
    Pr = D_term * fabsf(dot(m, N)) / (4.0f * fabsf(dot(in, m)));

    return pd * Pd + pr * Pr;
}

vec3 EvalScattering(vec3 out, vec3 N, vec3 in, const Material& mat) {
    vec3 Ed(0.0f), Er(0.0f), m(0.0f);
    vec3 Ks = mat.Ks;
    vec3 F_term(0.0f);
    float NdotM = 0.0f, NdotI = 0.0f, NdotO = 0.0f;
    float IdotM = 0.0f, OdotM = 0.0f;
    float alpha = sqrtf(2.0f / (mat.alpha + 2.0f));
    float alpha_square = alpha * alpha;
    float tanI = 0.0f, tanO = 0.0f, tanM = 0.0f;
    float D_term = 0.0f, G_term = 0.0f;
    float G1 = 0.0f, G2 = 0.0f;

    m = normalize(out + in);

    NdotI = dot(N,  in);
    NdotO = dot(N, out);
    NdotM = dot(N,   m);
    IdotM = dot(in,  m);
    OdotM = dot(out, m);
    tanI = sqrtf(1.0f - NdotI * NdotI) / NdotI;
    tanO = sqrtf(1.0f - NdotO * NdotO) / NdotO;
    tanM = sqrtf(1.0f - NdotM * NdotM) / NdotM;
    

    // Characteristic factor
    if(NdotM > 0.000001f)
        D_term = alpha_square / PI / (NdotM * NdotM * NdotM * NdotM) / (alpha_square + tanM * tanM) / (alpha_square + tanM * tanM);

    // Characteristic factor
    if (NdotI > 1.0f) {
        G1 = 1.0f;
    }
    else if ((IdotM / NdotI) > 0.0f) {
        // tan may be zero
        if (tanI < 0.000001f)
            G1 = 1.0f;
        else
            G1 = 2.0f / (1 + sqrtf(1 + alpha_square * tanI * tanI));            
    }

    if (NdotO > 1.0f) {
        G2 = 1.0f;
    }
    else if ((OdotM / NdotO) > 0.0f) {
        if (tanO < 0.000001f)
            G2 = 1.0f;
        else
            G2 = 2.0f / (1 + sqrtf(1 + alpha_square * tanO * tanO));
            
    }

    G_term = G1 * G2;

    IdotM = fabsf(IdotM);
    F_term = Ks + (1.0f - Ks) * (1 - IdotM) * (1.0f - IdotM) * (1.0f - IdotM) * (1.0f - IdotM) * (1.0f - IdotM);


    Ed = mat.Kd / PI;

    NdotI = fabsf(NdotI);
    if (NdotI > 1.0f) NdotI = 1.0f;
    NdotO = fabsf(NdotO);
    if (NdotO > 1.0f) NdotO = 1.0f;

    Er = D_term * G_term * F_term / (4.0f * NdotI * NdotO);

    return NdotI * (Ed + Er);
}