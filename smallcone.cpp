// smallcone. A rayTracer using a cone ray hierarchy as described by Roger et
// al. (2007) and with an implementation based on smallpt, a path tracer by
// Kevin Beason.  (But written slightly more human readable)

// Compile g++ -O2 smalldacrt.cpp -o smallcone
// Usage: ./smallcone 16 && xv image.ppm

#define PI ((float)3.14159265358979)

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <vector>
#include <algorithm>

#include "Vector.h"
#include "Ray.h"
#include "Sphere.h"
#include "AABB.h"
#include "Cone.h"

inline AABB Intersection(const AABB& lhs, const AABB& rhs) {
    return AABB(Vector3(std::max(lhs.min.x, rhs.min.x),
                        std::max(lhs.min.y, rhs.min.y),
                        std::max(lhs.min.z, rhs.min.z)),
                Vector3(std::min(lhs.max.x, rhs.max.x),
                        std::min(lhs.max.y, rhs.max.y),
                        std::min(lhs.max.z, rhs.max.z)));
}

template <class T>
inline T Clamp01(T v) { 
    return v < T(0) ? T(0) : v > T(1) ? T(1) : v;
}

inline double Rand01() {
    return (double)rand() / (double)RAND_MAX;
}

inline int ToByte(float v) {
    return int(pow(Clamp01(v),1/2.2)*255+.5);
}

const int WIDTH = 640, HEIGHT = 480, SPHERES = 200;
int sqrtSamples;
int samples;

int main(int argc, char *argv[]){
    Cone c = Cone(Vector3(-2, 0, 0), PI / 4.0f, Vector3(1,0,0));
    Sphere s = Sphere(1, Vector3(-2, 1, 0));
    
    fprintf(stderr, "Sphere and cone intersected? %d\n", (int)c.DoesIntersect(s));

    return 0;
}
