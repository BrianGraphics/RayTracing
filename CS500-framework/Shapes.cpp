#include "geom.h"
#include "acceleration.h"
#include "Intersect.h"
#include "Shapes.h"
#include "raytrace.h"





Sphere::Sphere(const vec3 c, const float r, Material* mat)
    : center(c), radius(r)
{
    vec3 rrr(r, r, r);

    material = mat;
    //IntersectionRecord.shape = this;
    boundingBox = SimpleBox(center + rrr);
    boundingBox.extend(center - rrr);
}

bool Sphere::Intersect(Ray ray, Intersection& record)
{
    float a = 0.0f, b = 0.0f, c = 0.0f, dis = 0.0f, t_p = 0.0f, t_m = 0.0f, t = 0.0f; // dis = discriminant
    vec3 Q = ray.Q, D = ray.D, newQ = ray.Q - center;

    a = glm::dot(D, D);
    b = 2 * glm::dot(newQ, D);
    c = glm::dot(newQ, newQ) - radius * radius;
    dis = b * b - 4 * a * c;

    if (dis < 0) {
        return false;
    }
    else
    {
        dis = sqrtf(dis);
        t_p = (-b + dis) / 2 / a;
        t_m = (-b - dis) / 2 / a;

        if (t_p < 0.0f && t_m < 0.0f)
            return false;
        else if (t_m >= 0.0f)
            t = t_m;
        else if (t_p >= 0.0f)
            t = t_p;
        else
            return false;
    }

    record.shape = this;
    record.t = t;
    record.P = Q + t * D;
    record.N = glm::normalize(record.P - center);

    return true;
}

Cylinder::Cylinder(const vec3 base, const vec3 axis, const float r, Material* mat) 
    : B(base), A(axis), radius(r)
{
    vec3 rrr(radius, radius, radius);

    material = mat;
    boundingBox = SimpleBox(B + rrr);
    boundingBox.extend(B - rrr);
    boundingBox.extend(A + B - rrr);
    boundingBox.extend(A + B + rrr);
}

bool Cylinder::Intersect(Ray ray, Intersection& record)
{
    vec3 base = vec3(0.0f);
    vec3 axis = vec3(0, 0, length(A));
    mat3 R = rotateToZ(A, false);
    mat3 t_R = rotateToZ(A, true);
    vec3 Q = R * (ray.Q - B);
    vec3 D = R * ray.D;
    const Slab slab(vec3(0.0, 0.0, 1.0), 0, -1.0f * length(axis));
    Interval intervalA, intervalB;
    Ray t_ray(Q, D);
    intervalA.Intersect(t_ray, slab);

    float a = 0.0f, b = 0.0f, c = 0.0f, dis = 0.0f,
          t0 = 0.0f, t1 = 0.0f, t = 0.0f;
    a = D.x * D.x + D.y * D.y;
    b = 2 * (D.x * Q.x + D.y * Q.y);
    c = Q.x * Q.x + Q.y * Q.y - radius * radius;
    dis = b * b - 4 * a * c;

    if (dis < 0) {
        return false;
    }
    else {
        dis = sqrtf(dis);
        intervalB = Interval((-b - dis) / 2 / a, (-b + dis) / 2 / a, vec3(0.0f, 0.0f, -1.0f), vec3(0.0f, 0.0f, 1.0f));
        
        intervalB.N0 = vec3(Q.x + intervalB.t0 * D.x, Q.y + intervalB.t0 * D.y, 0.0f);
        intervalB.N1 = vec3(Q.x + intervalB.t1 * D.x, Q.y + intervalB.t1 * D.y, 0.0f);
    }

    intervalA.Intersect(intervalB);
    t0 = intervalA.t0;
    t1 = intervalA.t1;

    if (t0 > t1)
        return false;
    else if (t0 >= 0)
        t = t0;
    else if (t1 >= 0)
        t = t1;
    else
        return false;

    record.shape = this;
    record.t = t;
    record.P = ray.Q + t * ray.D;
    record.N = t == t0 ? intervalA.N0 : intervalA.N1;
    record.N = normalize(t_R * record.N);

    return true;
}

mat3 Cylinder::rotateToZ(vec3 _A, bool reverse)
{
    vec3 n_A = normalize(_A);
    vec3 V_x = vec3(1.0f, 0.0f, 0.0f);
    vec3 V_z = vec3(0.0f, 0.0f, 1.0f);
    vec3 n_B = normalize(cross(V_x, n_A));
    if(glm::isnan(n_B).b)  n_B = normalize(cross(V_z, n_A));
    vec3 C   = cross(n_A, n_B);
    mat3 t_R = mat3(n_B, C, n_A);
    mat3 R   = glm::transpose(t_R);

    return reverse ? t_R : R;
}

Box::Box(const vec3 base, const vec3 diag, Material* mat)
    : corner(base), diagonal(diag)
{
    material = mat;
    slabs.push_back(Slab(vec3(1.0f, 0.0f, 0.0f), -base.x, -base.x - diag.x));
    slabs.push_back(Slab(vec3(0.0f, 1.0f, 0.0f), -base.y, -base.y - diag.y));
    slabs.push_back(Slab(vec3(0.0f, 0.0f, 1.0f), -base.z, -base.z - diag.z));
    boundingBox = SimpleBox(base);
    boundingBox.extend(base + diag);
}

bool Box::Intersect(Ray ray, Intersection& record)
{
    std::vector<Interval> intervals;
    const vec3 Q = ray.Q, D = ray.D;
    float t = 0.0f;

    for (auto it = slabs.begin(); it != slabs.end(); ++it) {
        Interval interval;
        interval.Intersect(ray, *it);
        intervals.push_back(interval);
    }

    Interval ret;
    for (Interval it : intervals) {
        ret.Intersect(it);
    }

    if (ret.t0 > ret.t1)
        return false;
    else if (ret.t0 >= 0)
        t = ret.t0;
    else if (ret.t1 >= 0)
        t = ret.t1;
    else
        return false;

    record.shape = this;
    record.t = t;
    record.P = Q + t * D;
    record.N = t == ret.t0 ? ret.N0 : ret.N1;
    
    return true;
}

Triangle::Triangle(std::vector<vec3> tri, std::vector<vec3> nrm)
{
    triangle = tri;
    normal   = nrm;
    boundingBox = SimpleBox(tri[0]);
    boundingBox.extend(tri[1]);
    boundingBox.extend(tri[2]);
}

bool Triangle::Intersect(Ray ray, Intersection& record)
{
    vec3 V0 = triangle[0], V1 = triangle[1], V2 = triangle[2];
    vec3 Q = ray.Q, D = ray.D;
    const vec3 E1 = V1 - V0, E2 = V2 - V0, S = Q - V0;
    vec3 p = cross(D, E2), q = vec3(0);
    float d = 0.0f, u = 0.0f, v = 0.0f, t = 0.0f;

    d = dot(p, E1);
    if (d == 0.0f) { record.t = -1.0f; return false; }
    
    u = dot(p, S) / d;
    if (u < 0.0f || u > 1.0f) { record.t = -1.0f; return false; }

    q = cross(S, E1);
    v = dot(D, q) / d;

    if(v < 0 || u + v > 1) { record.t = -1.0f; return false; }
    t = dot(E2, q) / d;
    if(t < 0) { record.t = -1.0f; return false; }

    record.shape = this;
    record.t = t;
    record.N = (1 - u - v) * normal[0] + u * normal[1] + v * normal[2];
    record.P = Q + t * D;

    return true;
}
