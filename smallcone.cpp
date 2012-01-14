// smallcone. A rayTracer using a cone ray hierarchy as described by Roger et
// al. (2007) and with an implementation based on smallpt, a path tracer by
// Kevin Beason.  (But written slightly more human readable)

// Compile ./make smallcone
// Usage: ./small 16 && xv image.ppm

#define PI ((float)3.14159265358979)

#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <vector>
#include <algorithm>

#include "Vector.h"
#include "Ray.h"
#include "HyperRay.h"
#include "Sphere.h"
#include "AABB.h"
#include "AABPenteract.h"
#include "Cone.h"
#include "Utils.h"

const int WIDTH = 640, HEIGHT = 480, SPHERES = 200;
int sqrtSamples;
int samples;

int main(int argc, char *argv[]){
    Cone c = Cone(Vector3(-2, 0, 0), Vector3(1,0,0), PI / 4.0f);
    Sphere s = Sphere(1, Vector3(-2, 1, 0));
    
    fprintf(stderr, "Sphere and cone intersected? %d\n", (int)c.DoesIntersect(s));

    Ray r = Ray(Vector3(-4,0,0), Vector3(1,-4,0));
    fprintf(stderr, "Ray included in cone? %d\n", (int)c.Includes(r));

    HyperRay hr(r);
    fprintf(stderr, "HyperRay: %s\n", hr.ToString().c_str());
    
    return 0;
}
