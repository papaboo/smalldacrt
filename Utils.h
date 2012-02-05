#ifndef _SMALL_UTILS_H_
#define _SMALL_UTILS_H_

#include <math.h>
#include "AABPenteract.h"
#include "Cone.h"

template <class T>
inline T Clamp01(T v) { 
    return v < T(0) ? T(0) : v > T(1) ? T(1) : v;
}

inline float Rand01() {
    return (float)rand() / (float)RAND_MAX;
}

inline int ToByte(float v) {
    return int(pow(Clamp01(v),1/2.2)*255+.5);
}

AABB CalcAABB(vector<Sphere>::const_iterator begin, 
              vector<Sphere>::const_iterator end) {
    AABB res(*begin);
    begin++;
    while (begin != end) {
        res.Extend(*begin);
        begin++;
    }
    
    return res;
}

Sphere CalcBoundingSphere(const vector<Sphere> spheres, 
                          const vector<int>::const_iterator begin, const vector<int>::const_iterator end) {
    Vector3 pos = spheres[*begin].position;
    vector<int>::const_iterator itr = begin+1;
    while (itr != end) {
        pos += spheres[*itr].position;
        ++itr;
    }
    pos = pos / (float) (end - begin);

    itr = begin;
    float radius = 0.0f;
    while (itr != end) {
        const Sphere s = spheres[*itr];
        Vector3 dist = s.position - pos;
        float d = dist.Length() + s.radius;
        radius = d > radius ? d : radius;
        ++itr;
    }
    
    return Sphere(radius, pos);
}

#endif
