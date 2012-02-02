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
#include "Sphere.h"
#include "HyperRay.h"
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
        Vector3 A = F(axis, cube.u.min, cube.v.max).Normalize();
        Vector3 B = F(axis, cube.u.max, cube.v.min).Normalize();
        Vector3 dir = ((A + B) * 0.5f).Normalize();
        
        Vector3 C = F(axis, cube.u.min, cube.v.min).Normalize();
        Vector3 D = F(axis, cube.u.max, cube.v.max).Normalize();
        
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

struct Fragment {
    Vector3 emission;
    int depth;
    Vector3 f; // What is this I wonder
    Fragment() : emission(Vector3(0,0,0)), depth(0), f(Vector3(1,1,1)) {}
};

const int WIDTH = 640, HEIGHT = 480;
//const int WIDTH = 8, HEIGHT = 6;
//const int WIDTH = 80, HEIGHT = 60;
int sqrtSamples;
int samples;

inline int Index(int x, int y, int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(int x, int y, int subX, int subY) {
    return (x + y * WIDTH) * samples + subX + subY * sqrtSamples;
}

std::vector<HyperRay> CreateRays() {
    Ray cam(Vector3(50,52,295.6), Vector3(0,-0.042612,-1).Normalize()); // cam pos, dir
    Vector3 cx = Vector3(WIDTH * 0.5135 / HEIGHT, 0, 0);
    Vector3 cy = (cx.Cross(cam.dir)).Normalize() * 0.5135;

    std::vector<HyperRay> rays = std::vector<HyperRay>(WIDTH * HEIGHT * samples);
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
                    // TODO create hyperrays directly
                    const Ray charles = Ray(cam.origin + rayDir * 130, rayDir.Normalize());
                    rays[Index(x,y,subX,subY)] = HyperRay(charles);
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

vector<int> Shade(vector<HyperRay>& rays, const vector<int>& rayIndices,
                  const vector<Fragment*>& frags, 
                  const vector<Sphere>& spheres, const vector<Hit>& hits) {
    vector<int> nextIndices;
    
    for (int i = 0; i < rayIndices.size(); ++i) {
        int rayID = rayIndices[i];
        int sphereID = hits[i].sphereID;

        if (sphereID == -1) continue;

        const Ray ray = rays[rayID].ToRay();
        const Sphere sphere = spheres[sphereID];
        const Vector3 hitPos = ray.origin + ray.dir * hits[i].t;
        const Vector3 norm = (hitPos - sphere.position).Normalize();
        const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;
        Color f = sphere.color;
        const float maxRefl = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z;
        if (++(frags[rayID]->depth) > 5)
            //if (erand48(Xi) < maxRefl) {
            if (Rand01() < maxRefl) {
                f = f * (1 / maxRefl); 
            } else {
                frags[rayID]->emission += frags[rayID]->f * sphere.emission;
                continue;
            }
                
        
        Vector3 newRayDir;
        switch(sphere.reflection) {
        case SPECULAR: {
            newRayDir = ray.dir - nl * 2 * Dot(nl, ray.dir);
            break;
        }
        case DIFFUSE: 
        default:
            float r1 = 2 * M_PI * Rand01();
            float r2 = Rand01(); 
            float r2s = sqrtf(r2);
            // Tangent space ?
            Vector3 w = nl; 
            Vector3 u = ((fabsf(w.x) > 0.1 ? Vector3(0,1,0) : Vector3(1,0,0)).Cross(w)).Normalize();
            Vector3 v = w.Cross(u);
            Vector3 newRayDir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrtf(1-r2)).Normalize();
            break;
        }

        rays[rayID] = HyperRay(Ray(hitPos + newRayDir * 0.1f, newRayDir));
        frags[rayID]->emission += frags[rayID]->f * sphere.emission;
        frags[rayID]->f = frags[rayID]->f * f;
        nextIndices.push_back(rayID);

    }
    
    return nextIndices;
}

// TODO replace sphere indices with sphere pointers, since we're only using that
// index for spheres.
inline void Exhaustive(const vector<HyperRay> &rays, vector<int> &rayIndices, const int indexOffset, const int indexCount,
                const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                vector<Hit> &hits) {
    
    for (int i = indexOffset; i < indexOffset + indexCount; ++i) {
        const int rayID = rayIndices[i];
        const Ray charles = rays[rayID].ToRay();
        for (int s = sphereOffset; s < sphereOffset + sphereCount; ++s) {
            const Sphere sphere = spheres[sphereIDs[s]];
            const float t = sphere.Intersect(charles);
            if (0 < t && t < hits[i].t)
                hits[i] = Hit(t, s);
        }
    }
};

void Dacrt(const HyperCube& cube, const Axis splitAxis, const float farDistance,
            const vector<HyperRay> &rays, vector<int> &rayIndices, const int indexOffset, const int rayCount,
            const vector<Sphere> &spheres, vector<int> &sphereIndices, const int sphereOffset, const int sphereCount,
            vector<Hit> &hits) {
    std::cout << "Dacrt with counts: " << rayCount << " x " << sphereCount << std::endl;

    if (sphereCount / rayCount < 16) { // Magic number
        // Perform exhaustive ray tracing

    } else {

        // Partition based on the distance from the apex and place far away
        // spheres first in the vector, so we don't accidentally overwrite them
        // while traversing.
        
        // Divide the hypercube and partition the rays
        
        // First iterate over the near chunks, then the far ones.

    }
}


bool CompareRayAxis (HyperRay lhs, HyperRay rhs) { return (lhs.axis < rhs.axis);}

struct SortRayIndicesByAxis {
    const vector<HyperRay>& rays;
    SortRayIndicesByAxis(const vector<HyperRay>& rs) : rays(rs) {}
    bool operator()(int i, int j) { return rays[i].axis < rays[j].axis; }
};

struct PartitionSpheresByCone {
    const vector<Sphere>& spheres;
    const Cone cone;
    PartitionSpheresByCone(const vector<Sphere>& spheres, const Cone cone)
        : spheres(spheres), cone(cone) {}
    bool operator()(int i) { return cone.DoesIntersect(spheres[i]); }
};

int main(int argc, char *argv[]){
    sqrtSamples = argc == 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    vector<HyperRay> rays = CreateRays();

    vector<Sphere> spheres = CreateSpheres();
    AABB B = CalcAABB(spheres.begin(), spheres.end());

    Fragment* frags = new Fragment[WIDTH * HEIGHT * samples];
    vector<Fragment*> rayFrags(WIDTH * HEIGHT * samples);
    for (int f = 0; f < WIDTH * HEIGHT * samples; ++f)
        rayFrags[f] = frags + f;

    // Create indices and sort them. New indices will be created along with the
    // shading.
    vector<int> rayIndices = vector<int>(rays.size());
    for(int i = 0; i < rayIndices.size(); ++i)
        rayIndices[i] = i;
    
    while (rayIndices.size() > 0) {

        std::cout << "rays this pass: " << rayIndices.size() << std::endl;

        std::sort(rayIndices.begin(), rayIndices.end(), SortRayIndicesByAxis(rays));

        vector<int> nextRayIndices = vector<int>();
        vector<Hit> hits(rayIndices.size());
        
        // For each hypercube
        int rayOffset = 0;
        for (int a = 0; a < 6; ++a) {
            
            int rayIndex = rayOffset;
            while(rayIndex < rayIndices.size() && rays[rayIndices[rayIndex]].axis == a)
                ++rayIndex;
            int rayCount = rayIndex - rayOffset;
            std::cout << "  RayCount is " << rayCount << " for axis " << a << std::endl;
            
            if (rayCount == 0) continue;
            
            HyperCube hc((SignedAxis)a, rays, rayIndices.begin(), rayCount);
            // std::cout << "HyberCube " << hc.ToString() << "\n   -> " << hc.ConeBounds().ToString() << std::endl;
            
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
            std::cout << "  Dacrt with counts " << rayCount << " x " << sphereCount << std::endl;
            // Dacrt(hc, U, rayBegin, rayCount, spheres.size());
            Exhaustive(rays, rayIndices, rayOffset, rayCount,
                       spheres, sphereIDs, sphereOffset, sphereCount,
                       hits);
            
            // Offset to beginning of next ray bundle.
            rayOffset += rayCount;
        }

        // Apply shading
        vector<int> newIndices = Shade(rays, rayIndices, rayFrags, spheres, hits);
        nextRayIndices.insert(nextRayIndices.end(), newIndices.begin(), newIndices.end());
        
        rayIndices = nextRayIndices;
    }

    // *********** CREATE IMAGE ****************

    // Combine colors into image
    Color* cs = new Color[WIDTH * HEIGHT];
    for (int x = 0; x < WIDTH; ++x)
        for (int y = 0; y < HEIGHT; ++y) {
            Color c = Color(0,0,0);
            for (int s = 0; s < samples; ++s)
                c += frags[Index(x,y,s)].emission / samples;
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
