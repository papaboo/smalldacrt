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

using std::vector;

#include "Vector.h"
#include "Ray.h"
#include "HyperRay.h"
#include "Sphere.h"
#include "AABB.h"
#include "AABPenteract.h"
#include "Cone.h"
#include "Utils.h"

struct HyperCube {
    AABPenteract cube;
    Axis axis;
    float t;
    
    HyperCube() : cube(AABPenteract()), axis(negZ), t(0) {}

    HyperCube(Axis axis, vector<HyperRay>::iterator rayBegin, int rayOffset) 
        : cube(AABPenteract(rayBegin[0].point)), axis(axis), t(0) {
        for (int r = 1; r < rayOffset; ++r)
            cube.Extent(rayBegin[r].point);
    }

    inline Cone ConeBounds() const {
        Vector3 A = F(axis, cube.u.min, cube.v.max);
        Vector3 B = F(axis, cube.u.max, cube.v.min);
        Vector3 dir = (A + B) * 0.5f;
        
        Vector3 C = F(axis, cube.u.min, cube.v.min);
        Vector3 D = F(axis, cube.u.min, cube.v.max);
        
        // Angle in degrees
        float angle = acos(Dot(A, dir));
        angle = std::max(angle, (float)acos(Dot(B, dir)));
        angle = std::max(angle, (float)acos(Dot(C, dir)));
        angle = std::max(angle, (float)acos(Dot(D, dir)));
        
        // Apex
        Vector3 r0 = Vector3(cube.x.min, cube.y.min, cube.z.min);
        Vector3 r1 = Vector3(cube.x.max, cube.y.max, cube.z.max);
        Vector3 center = (r0 + r1) * 0.5f;
        Vector3 negOffset = dir * (r0 - r1).Length() / (2 * sin(angle));
        Vector3 apex = center - negOffset;
        
        return Cone(apex, dir, angle);
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[axis: " << axis << ", cube: " << cube.ToString() << ", t: " << t << "]";
        return out.str();
    }


private:
    inline Vector3 F(Axis a, float u, float v) const {
        switch(a) {
        case posX: return Vector3(1.0f, u, v);
        case negX: return Vector3(-1.0f, u, v);
        case posY: return Vector3(u, 1.0f, v);
        case negY: return Vector3(u, -1.0f, v);
        case posZ: return Vector3(u, v, 1.0f);
        case negZ: return Vector3(u, v, -1.0f);
        }        
    }
};

//const int WIDTH = 640, HEIGHT = 480;
const int WIDTH = 4, HEIGHT = 3;
int sqrtSamples;
int samples;

inline int Index(int x, int y, int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(int x, int y, int subX, int subY) {
    return (x + y * WIDTH) * samples + subX + subY * sqrtSamples;
}

std::vector<Ray> CreateRays() {
    Ray cam(Vector3(50,52,295.6), Vector3(0,-0.042612,-1).Normalize()); // cam pos, dir
    Vector3 cx = Vector3(WIDTH * 0.5135 / HEIGHT, 0, 0);
    Vector3 cy = (cx.Cross(cam.dir)).Normalize() * 0.5135;

    std::vector<Ray> rays = std::vector<Ray>(WIDTH * HEIGHT * samples);
    for (int y = 0; y < HEIGHT; y++){
        unsigned short Xi[3] = {0, 0, y*y*y};
        for (unsigned short x = 0; x < WIDTH; x++) {
            
            // subpixel grid
            for (int subY = 0; subY < sqrtSamples; ++subY)
                for (int subX = 0; subX < sqrtSamples; ++subX) {
                    // Samples
                    double r1 = 2 * erand48(Xi);
                    float dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                    double r2 = 2 * erand48(Xi);
                    float dy = r2 < 1 ? sqrt(r2) - 1: 1 - sqrt(2 - r2);
                    
                    Vector3 rayDir = cx * (((subX + 0.5 + dx) / sqrtSamples + x) / WIDTH - 0.5) 
                        + cy * (((subY + 0.5 + dy) / sqrtSamples + y) / HEIGHT - 0.5) + cam.dir;
                    rays[Index(x,y,subX,subY)] = Ray(cam.origin + rayDir * 140, rayDir.Normalize());
                }
        }
    }

    return rays;
}

std::vector<Sphere> CreateSpheres() {
    const int SPHERES = 9;
    std::vector<Sphere> spheres = std::vector<Sphere>(SPHERES);
    spheres[0] = Sphere(1e5, Vector3(1e5+1,40.8,81.6),   Vector3(),Vector3(.75,.25,.25),DIFFUSE); // Left
    spheres[1] = Sphere(1e5, Vector3(-1e5+99,40.8,81.6), Vector3(),Vector3(.25,.25,.75),DIFFUSE); // Right
    spheres[2] = Sphere(1e5, Vector3(50,40.8, 1e5),      Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Back
    spheres[3] = Sphere(1e5, Vector3(50,40.8,-1e5+170),  Vector3(),Vector3(),           DIFFUSE); // Front
    spheres[4] = Sphere(1e5, Vector3(50, 1e5, 81.6),     Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Bottom
    spheres[5] = Sphere(1e5, Vector3(50,-1e5+81.6,81.6), Vector3(),Vector3(.75,.75,.75),DIFFUSE) ;// Top
    spheres[6] = Sphere(600, Vector3(50,681.6-.27,81.6), Vector3(12,12,12),  Vector3(), DIFFUSE); // Light
    spheres[7] = Sphere(16.5,Vector3(73,16.5,78),        Vector3(),Vector3(1,1,1)*.999, REFRACTING); // Glas
    spheres[8] = Sphere(16.5,Vector3(27,16.5,47),        Vector3(),Vector3(1,1,1)*.999, SPECULAR) ;// Mirror

    for (int s = 9; s < SPHERES; ++s) {
        // Create weird random spheres
        Vector3 pos = Vector3(Rand01() * 100.0 , Rand01() * 100.0 , Rand01() * 100.0 + 50.0);
        Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
        spheres[s] = Sphere(1, pos, Vector3(),  c, DIFFUSE);
    }

    return spheres;
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

float Dacrt(vector<HyperRay>::iterator rayBegin, int rayOffset) {
    std::cout << "Dacrt with offsets " << rayOffset << " x spherecount" << std::endl;

    return 0.0f;
}


bool CompareRayAxis (HyperRay lhs, HyperRay rhs) { return (lhs.axis < rhs.axis);}

int main(int argc, char *argv[]){
    sqrtSamples = argc == 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    vector<Ray> rays = CreateRays();
    vector<HyperRay> hyperRays = vector<HyperRay>(rays.size());
    for (int r = 0; r < rays.size(); ++r) {
        hyperRays[r] = HyperRay(rays[r]);
        // std::cout << "HyperRay[" << r << "] " << hyperRays[r].ToString() << std::endl;
    }

    vector<Sphere> spheres = CreateSpheres();
    AABB B = CalcAABB(spheres.begin(), spheres.end());
    std::cout << "World bounds: " << B.ToString() << std::endl;

    // Sort rays according to axis.
    std::sort(hyperRays.begin(), hyperRays.end(), CompareRayAxis);
    
    // For each hypercube
    vector<HyperRay>::iterator rayBegin = hyperRays.begin();
    for (int a = 0; a < 6; ++a) {
    
        // Sort geometry according to hypercube
        int offset = 0;
        while (rayBegin[offset].axis == a)
            ++offset;
        
        std::cout << "Offset is " << offset << " for axis " << a << std::endl;
        if (offset == 0)
            continue;

        HyperCube hc((Axis)a, rayBegin, offset);
        std::cout << "HyberCube " << hc.ToString() << std::endl;
        

        // perform dacrt
        Dacrt(rayBegin, offset);
        
        // Apply shading
        
        // Iterator to beginning of next ray bundle.
        rayBegin += offset;
    }

    // Rinse 'n repeat
    
    return 0;
}
