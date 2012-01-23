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

enum Axis {X = 0, Y = 1, Z = 2, U = 3, V = 4};

struct HyperCube {
    AABPenteract cube;
    SignedAxis axis;
    float t;
    
    HyperCube() : cube(AABPenteract()), axis(negZ), t(0) {}

    HyperCube(const SignedAxis axis, vector<HyperRay>::iterator rayBegin, int rayOffset) 
        : cube(AABPenteract(rayBegin[0].point)), axis(axis), t(0) {
        for (int r = 1; r < rayOffset; ++r)
            cube.Extent(rayBegin[r].point);
    }

    HyperCube(const SignedAxis axis, const vector<HyperRay>& hyperRays, vector<int>::iterator rayIndexBegin, const int rayOffset) 
        : axis(axis), t(0) {
        cube = AABPenteract(hyperRays[*rayIndexBegin].point);
        for (int r = 1; r < rayOffset; ++r)
            cube.Extent(hyperRays[rayIndexBegin[r]].point);
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
    inline Vector3 F(SignedAxis a, float u, float v) const {
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

struct Hit {
    float t;
    int sphereID;
    Hit() : t(1e+30), sphereID(-1) {}
    Hit(const float t, const int s) 
        : t(t), sphereID(s) {}
};

// const int WIDTH = 640, HEIGHT = 480;
//const int WIDTH = 8, HEIGHT = 6;
const int WIDTH = 160, HEIGHT = 120;
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

void Shade(const vector<HyperRay> rays, const vector<Color*> colors, 
           const vector<Sphere> spheres, const vector<Hit> hits) {
    for (int i = 0; i < rays.size(); ++i) {
        if (hits[i].sphereID == -1) {
            *colors[i] = Color(0,0,0);
            break;
        }
        
        *colors[i] = spheres[hits[i].sphereID].color;
        // std::cout << i << ": " << colors[i]->ToString() << std::endl;
    }
}

// TODO replace sphere indices with sphere pointers, since we're only using that
// index for spheres.
void Exhaustive(const vector<HyperRay> &rays, vector<int> &rayIndices, const int indexOffset, const int indexCount,
                const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                vector<Hit> &hits) {
    
    for (int i = indexOffset; i < indexOffset + indexCount; ++i) {
        int rayID = rayIndices[i];
        const Ray charles = rays[rayID].ToRay();
        std::cout << rayID << ": " << charles.ToString() << std::endl;
        // std::cout << "rayID: " << rayID << " = " << charles.ToString() << std::endl;
        for (int s = sphereOffset; s < sphereOffset + sphereCount; ++s) {
            //std::cout << "  sphere: " << s << std::endl;
            const Sphere& sphere = spheres[sphereIDs[s]];
            float t = sphere.Intersect(charles);
            if (0 < t && t < hits[rayID].t)
                hits[rayID] = Hit(t, s);
        }
        // std::cout << "ray " << rayID << " hit sphere " << hits[rayID].sphereID << " with color " << spheres[sphereIDs[hits[rayID].sphereID]].color.ToString() << std::endl;
    }
};

float Dacrt(const HyperCube& cube, const Axis splitAxis, const float farDistance,
            const vector<HyperRay> &rays, vector<int> &rayIndices, const int indexOffset, const int rayCount,
            const vector<Sphere> &spheres, vector<int> &sphereIndices, const int sphereOffset, const int sphereCount,
            vector<Hit> &hits) {
    std::cout << "Dacrt with offsets " << rayCount << " x " << sphereCount << std::endl;

    if (rayCount * sphereCount < 64) { // Magic number
        // Perform exhaustive ray tracing

    } else {

        // Partition based on the distance from the apex and place far away
        // spheres first in the vector, so we don't accidentally overwrite them
        // while traversing.
        
        // Divide the hypercube and partition the rays
        
        // First iterate over the near chunks, then the far ones.

    }

    return 0.0f;
}


bool CompareRayAxis (HyperRay lhs, HyperRay rhs) { return (lhs.axis < rhs.axis);}

struct PartitionSpheresByCone {
    const vector<Sphere> spheres;
    const Cone cone;
    PartitionSpheresByCone(const vector<Sphere> spheres, const Cone cone)
        : spheres(spheres), cone(cone) {}
    
    bool operator()(int i) { return cone.DoesIntersect(spheres[i]); }
};

int main(int argc, char *argv[]){
    sqrtSamples = argc == 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    vector<Ray> rays = CreateRays();
    vector<HyperRay> hyperRays = vector<HyperRay>(rays.size());
    for (int r = 0; r < hyperRays.size(); ++r)
        hyperRays[r] = HyperRay(rays[r]);
    vector<Hit> hits(hyperRays.size());

    vector<Sphere> spheres = CreateSpheres();
    AABB B = CalcAABB(spheres.begin(), spheres.end());

    Color* colors = new Color[WIDTH * HEIGHT * samples];
    vector<Color*> rayColors(WIDTH * HEIGHT * samples);
    for (int c = 0; c < WIDTH * HEIGHT * samples; ++c)
        rayColors[c] = colors + c;

    // For each hypercube
    for (int a = 0; a < 6; ++a) {

        /*    
        // Sort rays to a hypercube
        int rayCount = 0;
        while (rayBegin[rayCount].axis == a)
            ++rayCount;
        
        std::cout << "Ray count is " << rayCount << " for axis " << a << std::endl;
        if (rayCount == 0) continue;

        HyperCube hc((SignedAxis)a, rayBegin, rayCount);
        std::cout << "HyberCube " << hc.ToString() << std::endl;

        vector<int> rayIndices(rayCount);
        for (int i = 0; i < rayCount; ++i)
            rayIndices[i] = i;
        */

        int rayOffset = 0;
        vector<int> rayIndices = vector<int>();
        for (int r = 0; r < hyperRays.size(); ++r)
            if (hyperRays[r].axis == a) 
                rayIndices.push_back(r);
        int rayCount = rayIndices.size();
        std::cout << "Ray count is " << rayCount << " for axis " << a << std::endl;

        if (rayCount == 0) continue;

        HyperCube hc((SignedAxis)a, hyperRays, rayIndices.begin(), rayCount);
        std::cout << "HyberCube " << hc.ToString() << std::endl;

        // Partition spheres according to hypercube
        vector<int> sphereIDs(spheres.size());
        for (int i = 0; i < sphereIDs.size(); ++i)
            sphereIDs[i] = i;
        
        vector<int>::iterator spherePivot = 
            std::partition(sphereIDs.begin(), sphereIDs.end(),
                           PartitionSpheresByCone(spheres, hc.ConeBounds()));
        int sphereCount = spherePivot - sphereIDs.begin();
        int sphereOffset = 0;

        // perform dacrt
        std::cout << "Dacrt with counts " << rayCount << " x " << sphereCount << std::endl;
        // Dacrt(hc, U, rayBegin, rayCount, spheres.size());
        Exhaustive(hyperRays, rayIndices, rayOffset, rayCount,
                   spheres, sphereIDs, sphereOffset, sphereCount,
                   hits);

        // for (int h = 0; h < hits.size(); ++h)
        //     if (hits[h].sphereID != -1)
        //         std::cout << hits[h].t << " x " << spheres[hits[h].sphereID].ToString() << std::endl;
        //     else
        //         std::cout << hits[h].t << " x NULL" << std::endl;
        
        // Apply shading
        Shade(hyperRays, rayColors, spheres, hits);
        
        // Iterator to beginning of next ray bundle.
    }

    // TODO Rinse 'n repeat


    // *********** CREATE IMAGE ****************

    // Combine colors into image
    Color* cs = new Color[WIDTH * HEIGHT];
    for (int x = 0; x < WIDTH; ++x)
        for (int y = 0; y < HEIGHT; ++y) {
            Color c = Color(0,0,0);
            for (int s = 0; s < samples; ++s)
                c += colors[Index(x,y,s)] / samples;
            cs[x + y * WIDTH] = c;
        }

    // Write image to PPM file.
    FILE *f = fopen("image.ppm", "w");
    fprintf(f, "P3\n%d %d\n%d\n", WIDTH, HEIGHT, 255);
    for (int i = 0; i<WIDTH*HEIGHT; i++)
        fprintf(f,"%d %d %d ", ToByte(cs[i].x), 
                ToByte(cs[i].y), ToByte(cs[i].z));

    
    return 0;
}
