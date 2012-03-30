#ifndef _SMALL_UTILS_H_
#define _SMALL_UTILS_H_

#include <algorithm>
#include <math.h>
#include <string>
#include <vector>
#include <cstdio>

#include "AABPenteract.h"
#include "Cone.h"
#include "Vector.h"

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

inline AABB CalcAABB(std::vector<Sphere>::const_iterator begin, 
                     const std::vector<Sphere>::const_iterator end) {
    AABB res(*begin);
    begin++;
    while (begin != end) {
        res.Extend(*begin);
        begin++;
    }
    
    return res;
}

inline AABB CalcAABB(const std::vector<Sphere>& spheres, 
                     std::vector<int>::const_iterator begin, 
                     const std::vector<int>::const_iterator end) {
    AABB res(spheres[*begin]);
    begin++;
    while (begin != end) {
        res.Extend(spheres[*begin]);
        begin++;
    }
    return res;
}

inline AABB CalcAABB(const std::vector<Sphere>& spheres, 
                     std::vector<int>::const_iterator begin, 
                     const int count) {
    return CalcAABB(spheres, begin, begin + count);
}

Sphere CalcBoundingSphere(const std::vector<Sphere>& spheres, 
                          const std::vector<int>::const_iterator begin, const std::vector<int>::const_iterator end) {
    Vector3 pos = spheres[*begin].position;
    std::vector<int>::const_iterator itr = begin+1;
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

inline std::string BitmaskToString(unsigned int n){
    std::ostringstream out;
    out << "[";
    for (unsigned int i = 0; i < 31; ++i){
        if (n & 1<<i)
            out << 1 << ", ";
        else
            out << 0 << ", ";
    }
    if (n & (unsigned int)1<<31)
        out << 1 << "]";
    else
        out << 0 << "]";
    
    return out.str();
}

void SavePPM(const std::string path, const int width, const int height, const Color* cs) {
    FILE *f = fopen(path.c_str(), "w");
    fprintf(f, "P3\n%d %d\n%d\n", width, height, 255);
    for (int i = 0; i<width*height; i++)
        fprintf(f,"%d %d %d ", ToByte(cs[i].x), ToByte(cs[i].y), ToByte(cs[i].z));
}


#endif
