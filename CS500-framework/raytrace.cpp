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
        currentMat = new Material(vec3(f[1], f[2], f[3]), vec3(f[4], f[5], f[6]), f[7], vec3(f[8], f[9], f[10]), f[11]); }

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
                tmp[y * width + x] += TracePath(ray, bvh);
                if (myrandom(RNGen) >= rr) image[y * width + x] = tmp[y * width + x] / static_cast<float>(pass);
            }
        }

        fprintf(stderr, "\n");
    }
    delete tmp;
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
    float p = 0.0f, q = 0.0f, Wmis = 0.0f;
    float s = 0.0f, p_diffuse = 0.0f, p_reflection = 0.0f, p_transmission = 0.0f;
    float alpha = 0.0f;
    float ni = 0.0f, no = 0.0f, n = 0.0f;
    const float rr = 0.8f;    
    Intersection P, Q, L, I;
    BRDF brdf;
    sky->angle = PI * 1.5f;

    // get closest point
    P = bvh.intersect(ray);   

    // hit nothing so return nothing
    if (!P.isIntersect) return C;

    // hit light so return light
    if (P.shape->material->isLight()) {    
        return sky->Radiance(P);
    }

    // init
    N = P.N;
    Wo = normalize(-ray.D);

    // extend ray
    while (myrandom(RNGen) < rr) {     

        // roughness
        alpha = P.shape->material->GGX_alpha;

        // probability
        p_diffuse      = length(P.shape->material->Kd);
        p_reflection   = length(P.shape->material->Ks);
        p_transmission = length(P.shape->material->Kt);       

        s = p_diffuse + p_reflection + p_transmission;

        p_diffuse      /= s;
        p_reflection   /= s;
        p_transmission /= s;

        if (dot(Wo, N) > 0.0f) {
            ni = 1.0f;
            no = P.shape->material->IOR;
        }
        else {
            ni = P.shape->material->IOR;
            no = 1.0f;
        }
        n = ni / no;  

        brdf.ni = ni;
        brdf.no = no;
        brdf.mat = *P.shape->material;
        brdf.pd = p_diffuse;
        brdf.pr = p_reflection;
        brdf.pt = p_transmission;
        brdf.distance = P.t;     
        brdf._N = N;
        brdf._radicand = 0.0f;

        //Explicit light connection     
        L = sky->SampleAsLight();
        Wi = normalize(L.P - P.P);
        const Ray new_ray1(P.P, Wi);
        I = bvh.intersect(new_ray1);
        if (I.isIntersect && length(I.P - L.P) < 0.0001f) 
        {
            p = sky->PdfAsLight(L) / GeometryFactor(P, L);
            if (p >= 0.000001f && !isnan(p)) 
            {
                q = brdf.PdfBrdf(Wo, N, Wi) * rr;
                Wmis = p * p / (p * p + q * q);
                f = brdf.EvalScattering(Wo, N, Wi);
                C += W * Wmis * (f / p) * sky->Radiance(I);            
            }
        }

        // Extend path
        // SampleBRDF, choose random direction
        Wi = brdf.SampleBrdf(Wo, N);

        const Ray new_ray2(P.P, Wi);

        Q = bvh.intersect(new_ray2);
        if (!Q.isIntersect) break;
        
        p = brdf.PdfBrdf(Wo, N, Wi) * rr;
        if (p < 0.000001f) break;

        f = brdf.EvalScattering(Wo, N, Wi);        

        W *= f / p;            

        if (Q.shape->material->isLight()) {     
            q = sky->PdfAsLight(Q) / GeometryFactor(P, Q);
            Wmis = p * p / (p * p + q * q);
            C += W * Wmis * sky->Radiance(Q);
            break;
        }

        P = Q;
        Wo = -Wi;
        N = P.N;
    }


    if (glm::all(glm::isnan(C))) C = vec3(0.0f);

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
    const float r1 = myrandom(RNGen), r2 = myrandom(RNGen);
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

vec3 BRDF::SampleBrdf(const vec3 out, const vec3 N) 
{
    float const r  = myrandom(RNGen);
    float const r1 = myrandom(RNGen);
    float const r2 = myrandom(RNGen) * 2.0f * PI;    
    float const alpha = mat.GGX_alpha;
    float const ttr = 0.0f;
    float tmp = 0.0f;
    vec3 m(0.0f);

    if (r <= pd) {                     // choice = diffuse                
        tmp = sqrtf(r1);
        return SampleLobe(N, tmp, r2);
    }
    else if (r <= (pd + pr)) {  // choice = specular        
        tmp = cos(atan(alpha * sqrtf(r1) / sqrtf(1.0f - r1)));
        m = SampleLobe(N, tmp, r2);
        return 2.0f * fabsf(dot(out, m)) * m - out;
    }
    else {                                         // choice = transmission
        tmp = cos(atan(alpha * sqrtf(r1) / sqrtf(1.0f - r1)));        
        m = SampleLobe(N, tmp, r2);
        const float WdotM = dot(out, m);
        float radicand = 1.0f - (ni / no) * (ni / no) * (1.0f - WdotM * WdotM);
        if (radicand < ttr) {
            return 2.0f * fabsf(WdotM) * m - out;
        }        
        else {
            const float sign = dot(out, N) >= 0 ? 1.0f : -1.0f;
            return normalize(((ni / no) * WdotM - sign * sqrtf(radicand)) * m - (ni / no) * out);
        }
    }
}

float BRDF::PdfBrdf(const vec3 out, const vec3 N, const vec3 in) {
    vec3 m_r(0.0f), m_t(0.0f);
    float Pd = 0.0f, Pr = 0.0f, Pt = 0.0f;
    float const ttr = 0.0f;

    if (pd != 0.0f) 
        Pd = fabsf(dot(in, N)) / PI;

    if (pr != 0.0f) 
    {
        m_r = normalize(out + in);
        Pr = D_factor(m_r) * fabsf(dot(m_r, N))
            / (4.0f * fabsf(dot(in, m_r)));
    }

    if (pt != 0.0f) 
    {
        m_t = -normalize(no * in + ni * out);        
        _radicand = 1.0f - (ni / no) * (ni / no) * (1.0f - dot(out, m_t) * dot(out, m_t));
        if (_radicand < ttr) {
            m_t = normalize(out + in);
            Pt = D_factor(m_t) * fabsf(dot(m_t, N))
                 / (4.0f * fabsf(dot(in, m_t)));
            Pt = Pr;
        }
        else {
            Pt = D_factor(m_t) * fabsf(dot(m_t, N)) 
                 * no * no * fabsf(dot(in, m_t))
                 / ((no * dot(in, m_t) + ni * dot(out, m_t)) * (no * dot(in, m_t) + ni * dot(out, m_t)));
        }        
    }

    return pd * Pd + pr * Pr + pt * Pt;
}

vec3 BRDF::EvalScattering(const vec3 out, const vec3 N, const vec3 in) {    
    float const ttr = 0.0f;
    vec3 Ed(0.0f), Er(0.0f), Et(0.0f), m_r(0.0f), m_t(0.0f);

    // diffuse
    if(pd != 0.0f)
        Ed = mat.Kd / PI;     

    if (pr != 0.f) {
        m_r = normalize(out + in);
        Er = D_factor(m_r) * G_factor(in, out, m_r) * F_factor(dot(in, m_r)) 
             / (4 * fabsf(dot(in, N)) * fabsf(dot(out, N)));
    }

    if (pt != 0.0f) 
    {
        m_t = -normalize(no * in + ni * out);       
        const vec3 At = dot(out, N) < 0.0f ? vec3(exp(distance * log(mat.Kt.x)), exp(distance * log(mat.Kt.y)), exp(distance * log(mat.Kt.z))) : vec3(1.0f);      
        
        if (_radicand < ttr) {
            m_t = normalize(out + in);
            Et = At * D_factor(m_t) * G_factor(in, out, m_t) * F_factor(dot(in, m_t))
                / (4.0f * fabsf(dot(in, N)) * fabsf(dot(out, N)));
            Et = Er;
        }
        else {
            Et = At * D_factor(m_t) * G_factor(in, out, m_t) * (vec3(1.0f) - F_factor(dot(in, m_t)))
                / (fabsf(dot(in, N)) * fabsf(dot(out, N)))
                * fabsf(dot(in, m_t)) * fabsf(dot(out, m_t)) * no * no
                / ((no * dot(in, m_t) + ni * dot(out, m_t)) * (no * dot(in, m_t) + ni * dot(out, m_t)));
        }

    }    
    
    return  fabsf(dot(N, in)) * (Ed + Er + Et);
}

vec3  BRDF::F_factor(const float d) 
{
    vec3 const Ks = mat.Ks;
    return Ks + (vec3(1.0f) - Ks) * static_cast<float>(powf((1.0f - fabsf(d)), 5.0f));
}

float BRDF::D_factor(const vec3 m)
{
    float const alpha = mat.GGX_alpha;
    float const MdotN = dot(m, _N);
    float c_factor = 0.0f;
    float tanM = 0.0f;
    
    // check 1: characteristic factor
    c_factor = MdotN > 0.0f ? 1.0f : 0.0f;
    if (c_factor == 0.0f) return c_factor;

    // check 2: nan tan value 
    tanM = sqrtf(1.0f - MdotN * MdotN) / MdotN;
    if (isnan(tanM)) return 0.0f;

    return alpha * alpha / (PI * powf(MdotN, 4.0f) * (alpha * alpha + tanM * tanM) * (alpha * alpha + tanM * tanM));
}

float BRDF::G_factor(const vec3 in, const vec3 out, const vec3 m)
{
    float const alpha = mat.GGX_alpha;
    float tanI = 0.0f, tanO = 0.0f;
    float c_factor1 = 0.0f, c_factor2 = 0.0f;
    float G1_first = 0.0f, G1_second = 0.0f;

    // first G1
    c_factor1 = (dot(in, m) / dot(in, _N)) > 0.0f ? 1.0f : 0.0f;
    if (c_factor1 == 0.0f) return 0.0f;
    if (dot(in, _N) > 1.0f)  G1_first  = 1.0f;

    // second G1
    c_factor2 = (dot(in, m) / dot(in, _N)) > 0.0f ? 1.0f : 0.0f;
    if (c_factor2 == 0.0f) return 0.0f;
    if (dot(out, _N) > 1.0f) G1_second = 1.0f;

    // test tan of G1_first
    tanI = sqrtf(1.0f - dot(in, _N) * dot(in, _N)) / dot(in, _N);
    if (tanI == 0.0f) G1_first = 1.0f;
    if (isnan(tanI))  return 0.0f;

    // test tan of G1_second
    tanO = sqrtf(1.0f - dot(out, _N) * dot(out, _N)) / dot(out, _N);
    if (tanO == 0.0f) G1_second = 1.0f;
    if (isnan(tanO))  return 0.0f;

    if (G1_first != 1.0f) {
        G1_first  = 2.0f / (1.0f + sqrtf(1.0f + alpha * alpha * tanI * tanI));
    }

    if (G1_second != 1.0f) {
        G1_second = 2.0f / (1.0f + sqrtf(1.0f + alpha * alpha * tanO * tanO));
    }

    return G1_first * G1_second;
}

void Sky::PreProcessing()
{
    pBuffer = new float[width * (height + 1)];
    pUDist = &pBuffer[width * height];
    float* pSinTheta = new float[height];
    float angleFrac = PI / float(height);
    float theta = angleFrac * 0.5f;
    for (unsigned int i = 0; i < height; i++, theta += angleFrac)
        pSinTheta[i] = sin(theta);

    for (unsigned int i = 0, m = 0; i < width; i++, m += height) {
        float* pVDist = &pBuffer[m];
        //unsigned int k = i * 3;
        pVDist[0] = 0.2126f * hdr[i].r + 0.7152f * hdr[i].g + 0.0722f * hdr[i].b;
        pVDist[0] *= pSinTheta[0];
        for (unsigned int j = 1, k = (width + i); j < height; j++, k += width) {
            float lum = 0.2126 * hdr[k].r + 0.7152 * hdr[k].g + 0.0722f * hdr[k].b;
            pVDist[j] = pVDist[j - 1] + lum * pSinTheta[j];
        }
        if (i == 0)
            pUDist[i] = pVDist[height - 1];
        else
            pUDist[i] = pUDist[i - 1] + pVDist[height - 1];
    }
}

Intersection Sky::SampleAsLight()
{
    Intersection B;
    double u = myrandom(RNGen);
    double v = myrandom(RNGen);
    float maxUVal = pUDist[width - 1];
    float* pUPos = std::lower_bound(pUDist, pUDist + width, u * maxUVal);
    int iu = pUPos - pUDist;
    float* pVDist = &pBuffer[height * iu];
    float* pVPos = std::lower_bound(pVDist, pVDist + height,
        v * pVDist[height - 1]);
    int iv = pVPos - pVDist;
    double phi = angle - 2.0f * PI * iu / width;    
    double theta = PI * iv / height;
    B.N = vec3(sin(theta) * cos(phi),
        sin(theta) * sin(phi),
        cos(theta));
    B.P = B.N * radius;
    B.shape = NULL;
    return B;
}

float Sky::PdfAsLight(const Intersection& B) const
{
    vec3 P = normalize(B.P);
    double fu = (angle - atan2(P[1], P[0])) / (PI * 2.0f);
    fu = fu - floor(fu); // Wrap to be within 0...1
    int u = floor(width * fu);
    int v = floor(height * acos(P[2]) / PI);
    float angleFrac = PI / float(height);
    float* pVDist = &pBuffer[height * u];
    float pdfU = (u == 0) ? (pUDist[0]) : (pUDist[u] - pUDist[u - 1]);
    pdfU /= pUDist[width - 1];
    pdfU *= width / (PI * 2.0f);
    float pdfV = (v == 0) ? (pVDist[0]) : (pVDist[v] - pVDist[v - 1]);
    pdfV /= pVDist[height - 1];
    pdfV *= height / PI;
    float theta = angleFrac * 0.5 + angleFrac * v;
    float pdf = pdfU * pdfV * sin(theta) / (4.0 * PI * radius * radius);

    return pdf;
}

vec3 Sky::Radiance(const Intersection& A)
{       
    vec3 P = normalize(A.P);
    double u = (angle - atan2(P[1], P[0])) / (PI * 2.0f);
    u = u - floor(u); // Wrap to be within 0...1
    double v = acos(P[2]) / PI;
    int i0 = floor(u * width);
    int j0 = floor(v * height);
    double uw[2], vw[2];
    uw[1] = u * width - i0; uw[0] = 1.0 - uw[1];
    vw[1] = v * height - j0; vw[0] = 1.0 - vw[1];
    vec3 r(0.0f, 0.0f, 0.0f);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            int k = (((j0 + j) % height) * width + ((i0 + i) % width));
            for (int c = 0; c < 3; c++) {
                r[c] += uw[i] * vw[j] * hdr[k][c];
            }
        }
    }

    if (glm::all(glm::isnan(r))) return vec3(0.0f);

    return r;
}