#pragma once

class Slab {
public:
	Slab(vec3 _N, float _d0, float _d1) : N(_N), d0(_d0), d1(_d1) {}
	vec3 N;
	float d0;
	float d1;
};
class Ray;
class Interval {
public:
	Interval();
	Interval(float _t0, float _t1, vec3 _N0, vec3 _N1);
	
	void Intersect(Interval other);
	Interval Intersect(Ray ray, Slab slab);
	void empty();
	float t0;
	float t1;
	vec3 N0;
	vec3 N1;

};