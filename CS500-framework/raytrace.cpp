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
#define RR 0.8f
#define random static_cast <float> (rand()) / static_cast <float> (RAND_MAX)

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
        if (currentMat->isLight()) {
            light = sphere;
            lightPos = vec3(f[1], f[2], f[3]);
        }
        else {
            vectorOfShapes.push_back(sphere);
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
    float rx = camera.ry * width / height;
    float dx = 0.0f, dy = 0.0f;
    vec3 X = rx * transformVector(camera.orientation, Xaxis());
    vec3 Y = camera.ry * transformVector(camera.orientation, Yaxis());
    vec3 Z = transformVector(camera.orientation, Zaxis());
    vec3 L(0);
    Color color(0);
    Intersection front;
    AccelerationBvh bvh(vectorOfShapes);
    Ray ray(vec3(0), vec3(0));

#pragma omp parallel for schedule(dynamic, 1) // Magic: Multi-thread y loop
    for (int y = 0; y < height; y++) {
        fprintf(stderr, "Rendering %4d\r", y);
        for (int x = 0; x < width; x++) {
            dx = 2 * (x + 0.5f) / width - 1;
            dy = 2 * (y + 0.5f) / height - 1;
            ray.Q = camera.eye;
            ray.D = normalize(dx * X + dy * Y - Z);          
            front = bvh.intersect(ray);
            //front = TraceRay(ray);

            if (front.isIntersect) {                
                L = normalize(lightPos - front.P);
                color = glm::max(0.0f, dot(glm::abs(front.N), L)) * front.shape->material->Kd;
                //color = front.P;
                //color = front.shape->material->Kd;
                //color = glm::abs(front.N);
                //color = vec3((front.t -5.0f) / 4.0f);

             
            }

            image[y * width + x] = color;    
            color = vec3(0.0f);
            front.isIntersect = false;
        }
    }
    
    fprintf(stderr, "\n");
}

Intersection Scene::TraceRay(const Ray& ray) {
    Intersection front, current;
    front.t = std::numeric_limits<float>::infinity();
    for (auto shape : vectorOfShapes) {     
        if (shape->Intersect(ray, current))
            if (front.t > current.t)
                front = current;
    }

    return front;
}