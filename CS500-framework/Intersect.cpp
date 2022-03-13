#include "geom.h"
#include "acceleration.h"
#include "Intersect.h"



Interval::Interval()
{
	t0 = 0.0f;
	t1 = std::numeric_limits<float>::infinity();
	N0 = vec3(0);
	N1 = vec3(0);
}

Interval::Interval(float _t0, float _t1, vec3 _N0, vec3 _N1)
{
	t0 = _t0;
	t1 = _t1;
	N0 = _N0;
	N1 = _N1;

	if (t0 > t1) {
		std::swap(t0, t1);
		std::swap(N0, N1);
	}
}

void Interval::empty()
{
	t0 =  1.0f;
	t1 =  0.0f;
}

void Interval::Intersect(Interval other)
{
	t0 = fmaxf(t0, other.t0);
	t1 = fminf(t1, other.t1);
	N0 = t0 == other.t0 ? other.N0 : N0;
	N1 = t1 == other.t1 ? other.N1 : N1;
}

void Interval::Intersect(Ray ray, Slab slab)
{
	vec3 Q = ray.Q, D = ray.D, N = slab.N;
	float d0 = slab.d0, d1 = slab.d1;
    float s0 = 0.0f, s1 = 0.0f, NdotD = 0.0f, NdotQ = 0.0f;
	NdotD = glm::dot(N, D);

	// not sure
	N0 = N;
	N1 = N;

	if (NdotD != 0) {
		NdotQ = glm::dot(N, Q);
		t0 = (-1.0) * (d0 + NdotQ) / NdotD;
		t1 = (-1.0) * (d1 + NdotQ) / NdotD;

		if (t0 > t1) std::swap(t0, t1);
	}
	else {
		NdotQ = glm::dot(N, Q);
		s0 = NdotQ + d0;
		s1 = NdotQ + d1;
		if (s0 * s1 < 0 && s0 != s1) {
			t0 = 0;
			t1 = std::numeric_limits<float>::infinity();
		}
		else {
			empty();
		}
	}
}


