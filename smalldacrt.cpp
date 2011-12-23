// smalldacrt. A Divide and Conquer RayTracer as described by *** and with an
// implementation based on smallpt, a path tracer be Kevin Beason. 
// (But written slightly more human readable)

// Compile g++ -O2 smalldacrt.cpp -o smalldacrt
// Usage: ./smalldacrt 16 && xv image.ppm

#define PI ((float)3.14159265358979)

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <vector>
#include <algorithm>

#include "Vector.h"

struct Ray {
    Vector3 origin;
    Vector3 dir;

    Ray() : origin(Vector3()), dir(Vector3()) {}

    Ray(const Vector3& o, const Vector3& d) 
        : origin(o), dir(d) {}

    inline Vector3 PositionAt(float t) {
        return origin + dir * t;
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[origin: " << origin.ToString() << ", direction: " << dir.ToString() + "]";
        return out.str();
    }
};

enum ReflectionType {DIFFUSE, SPECULAR, REFRACTING};

struct Sphere {
    float radius;
    Vector3 position, emission, color;
    ReflectionType reflection;

    Sphere() {}

    Sphere(const float r, const Vector3& p, const Vector3& e, 
           const Vector3& c, const ReflectionType rt) 
        : radius(r), position(p), emission(e), color(c), reflection(rt) {}
    
    // returns distance, 0 if nohit
    inline float Intersect(const Ray& r) const {
        // Solve t^2*d.d + 2*t*(o-p).d + (o-p).(o-p)-R^2 = 0
        const float eps = 1e-4;
        Vector3 dir = position - r.origin;
        float b = Dot(dir, r.dir);
        float det = b*b - Dot(dir, dir) + radius * radius;
        if (det < 0) return 0; else det = sqrt(det);
        float t;
        return (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0);
    }

    inline std::string ToString() {
        std::ostringstream out;
        out << "[radius: " << radius << ", position: " << position.ToString() + "]";
        return out.str();
    }
};

struct AABB {
    Vector3 min;
    Vector3 max;
    
    AABB(const Vector3& min, const Vector3& max) 
        : min(min), max(max) {}

    AABB(const Sphere& s)
        : min(s.position - s.radius), max(s.position + s.radius) {}

    inline void Extend(const Sphere& s) {
        min.x = std::min(min.x, s.position.x - s.radius);
        min.y = std::min(min.y, s.position.y - s.radius);
        min.z = std::min(min.z, s.position.z - s.radius);

        max.x = std::max(max.x, s.position.x + s.radius);
        max.y = std::max(max.y, s.position.y + s.radius);
        max.z = std::max(max.z, s.position.z + s.radius);
    }

    inline float Intersect(const Ray& ray) const {
        Vector3 minTs = (min - ray.origin) / ray.dir;
        Vector3 maxTs = (max - ray.origin) / ray.dir;
        
        float nearT = std::min(minTs.x, maxTs.x);
        nearT = std::max(nearT, std::min(minTs.y, maxTs.y));
        nearT = std::max(nearT, std::min(minTs.z, maxTs.z));

        float farT = std::max(minTs.x, maxTs.x);
        farT = std::min(farT, std::max(minTs.y, maxTs.y));
        farT = std::min(farT, std::max(minTs.z, maxTs.z));
        
        return farT < nearT ? -1e30 : (nearT <= 0 ? farT : nearT);
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[min: " << min.ToString() << ", max: " << max.ToString() + "]";
        return out.str();
    }
};

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

const int WIDTH = 640, HEIGHT = 480, SPHERES = 20000;
int sqrtSamples;
int samples;

inline int Index(int x, int y, int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(int x, int y, int subX, int subY) {
    return (x + y * WIDTH) * samples + subX + subY * sqrtSamples;
}

inline void PrintWhiteSpace(const int level) {
    for (int l = 0; l < level; ++l)
        fprintf(stderr, "  ");
}

Ray* CreateRays() {
    Ray cam(Vector3(50,52,295.6), Vector3(0,-0.042612,-1).Normalize()); // cam pos, dir
    Vector3 cx = Vector3(WIDTH * 0.5135 / HEIGHT, 0, 0);
    Vector3 cy = (cx.Cross(cam.dir)).Normalize() * 0.5135;

    Ray* rays = new Ray[WIDTH * HEIGHT * samples];
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

void ResetTs(float** ts) {
    if (*ts == NULL)
        *ts = new float[WIDTH * HEIGHT * samples];

    for (int i = 0; i < WIDTH * HEIGHT * samples; i++)
        (*ts)[i] = 1e+30;
}

Sphere* CreateSpheres() {
    Sphere* spheres = new Sphere[SPHERES];
    spheres[0] = Sphere(1e5, Vector3(1e5+1,40.8,81.6),   Vector3(),Vector3(.75,.25,.25),DIFFUSE); // Left
    spheres[1] = Sphere(1e5, Vector3(-1e5+99,40.8,81.6), Vector3(),Vector3(.25,.25,.75),DIFFUSE); // Right
    spheres[2] = Sphere(1e5, Vector3(50,40.8, 1e5),      Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Back
    spheres[3] = Sphere(1e5, Vector3(50,40.8,-1e5+170),  Vector3(),Vector3(),           DIFFUSE); // Front
    spheres[4] = Sphere(1e5, Vector3(50, 1e5, 81.6),     Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Bottom
    spheres[5] = Sphere(1e5, Vector3(50,-1e5+81.6,81.6), Vector3(),Vector3(.75,.75,.75),DIFFUSE) ;// Top
    spheres[6] = Sphere(600, Vector3(50,681.6-.27,81.6), Vector3(12,12,12),  Vector3(), DIFFUSE); // Light
    spheres[7] = Sphere(16.5,Vector3(73,16.5,78),        Vector3(),Vector3(1,1,1)*.999, REFRACTING); // Glas
    spheres[8] = Sphere(16.5,Vector3(27,16.5,47),        Vector3(),Vector3(1,1,1)*.999, SPECULAR) ;// Mirror

    for (int s = 7; s < SPHERES; ++s) {
        // Create weird random spheres
        Vector3 pos = Vector3(Rand01() * 100.0 , Rand01() * 100.0 , Rand01() * 100.0 + 50.0);
        Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
        spheres[s] = Sphere(1, pos, Vector3(),  c, DIFFUSE);
    }

    return spheres;
}

/**
 * Splits the array of sphere ids depending on their location relative to the median.
 *
 * Returns the split pivot.
 */
int SplitSpheres(Sphere* spheres, int* sphereIDs, int spherePivot, float median, bool (*test)(Vector3&, float)) {
    int headPtr = 0, tailPtr = spherePivot;
    while (headPtr < tailPtr) {
        while (headPtr < tailPtr && test(spheres[sphereIDs[headPtr]].position, median))
            ++headPtr;
        do
            --tailPtr;
        while (headPtr < tailPtr && !test(spheres[sphereIDs[tailPtr]].position, median));
        if (headPtr < tailPtr) 
            std::swap(*(sphereIDs + headPtr), *(sphereIDs + tailPtr));
    }
    return headPtr;
}

bool LeftSplitSpheresX(Vector3& pos, float median) { return pos.x <= median; }
bool LeftSplitSpheresY(Vector3& pos, float median) { return pos.y <= median; }
bool LeftSplitSpheresZ(Vector3& pos, float median) { return pos.z <= median; }
bool RightSplitSpheresX(Vector3& pos, float median) { return pos.x > median; }
bool RightSplitSpheresY(Vector3& pos, float median) { return pos.y > median; }
bool RightSplitSpheresZ(Vector3& pos, float median) { return pos.z > median; }

int SplitRays(Ray* rays, int* rayIDs, int rayPivot, AABB& aabb) {
    int headPtr = 0, tailPtr = rayPivot;
    while (headPtr < tailPtr) {
        while (headPtr < tailPtr && aabb.Intersect(rays[rayIDs[headPtr]]) > 0) 
            ++headPtr;
        do 
            --tailPtr; 
        while (headPtr < tailPtr && aabb.Intersect(rays[rayIDs[tailPtr]]) <= 0);
        if (headPtr < tailPtr) 
            std::swap(*(rayIDs + headPtr), *(rayIDs + tailPtr));
    }
    return headPtr;
}

/**
 * Takes as argument a list of rays and rayIDs, spheres and sphereIDs, and an
 * array of the rays max travel distance maxTs. Then recursively fills the
 * spheresHit array with the spheres that a given ray has intersected first.
 */
void Dacrt(Ray* rays, int* rayIDs, int rayPivot,
           Sphere* spheres, int* sphereIDs, int spherePivot,
           float* maxTs, int* spheresHit, const AABB& bounds) {

    static int level = -1;
    ++level;

    // PrintWhiteSpace(level);
    // fprintf(stderr, "DACRT(rayPivot: %d, spherePivot: %d)\n", rayPivot, spherePivot);

    if (rayPivot < 128 || spherePivot < 48) { // Dacrt
        //if (true) { // Exhaustive search
        // Naive intersection
        for (int r = 0; r < rayPivot; ++r) {
            int rayID = rayIDs[r];
            Ray charles = rays[rayID];
            for (int s = 0; s < spherePivot; ++s) {
                Sphere sphere = spheres[sphereIDs[s]];

                float t = sphere.Intersect(charles);
                if (0 < t && t < maxTs[rayID]) {
                    maxTs[rayID] = t;
                    spheresHit[rayID] = sphereIDs[s];
                }
            }
        }
    } else { // Perform DACRT

        // PrintWhiteSpace(level);
        // fprintf(stderr, "Bounds %s\n", bounds.ToString().c_str());
            
        Vector3 boundSize = bounds.max - bounds.min;
        // left side
        {
            // split spheres along the spatial median
            AABB halvedBounds = bounds;
            int newSpherePivot;
            if (boundSize.x > boundSize.y && boundSize.x > boundSize.z) {                
                float median = halvedBounds.max.x = (bounds.max.x + bounds.min.x) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, LeftSplitSpheresX);
            } else if (boundSize.y > boundSize.x && boundSize.y > boundSize.z) {
                float median = halvedBounds.max.y = (bounds.max.y + bounds.min.y) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, LeftSplitSpheresY);
            } else { // if (boundSize.z > boundSize.y && boundSize.z > boundSize.x) {
                float median = halvedBounds.max.z = (bounds.max.z + bounds.min.z) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, LeftSplitSpheresZ);
            }
            
            // Get new bounding box
            AABB leftAABB = AABB(spheres[sphereIDs[0]]);
            for (int s = 1; s < newSpherePivot; ++s)
                leftAABB.Extend(spheres[sphereIDs[s]]);
            
            // PrintWhiteSpace(level);
            // fprintf(stderr, "halvedBounds %s\n", halvedBounds.ToString().c_str());
            // PrintWhiteSpace(level);
            // fprintf(stderr, "LeftAABB %s\n", leftAABB.ToString().c_str());            
            // PrintWhiteSpace(level);
            // fprintf(stderr, "InterAABB %s\n", Intersection(leftAABB, halvedBounds).ToString().c_str());            
            
            // determine active rays
            int newRayPivot = SplitRays(rays, rayIDs, rayPivot, leftAABB);

            // recurse: use intersection of halved and calculated bounds to
            // ensure smaller problem domain.
            Dacrt(rays, rayIDs, newRayPivot,
                  spheres, sphereIDs, newSpherePivot,
                  maxTs, spheresHit, Intersection(leftAABB, halvedBounds));
        }

        // right side
        {
            // split spheres along the spatial median
            AABB halvedBounds = bounds;
            int newSpherePivot;
            if (boundSize.x > boundSize.y && boundSize.x > boundSize.z) {
                float median = halvedBounds.min.x = (bounds.max.x + bounds.min.x) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, RightSplitSpheresX);
            } else if (boundSize.y > boundSize.x && boundSize.y > boundSize.z) {
                float median = halvedBounds.min.y = (bounds.max.y + bounds.min.y) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, RightSplitSpheresY);
            } else { // if (boundSize.z > boundSize.y && boundSize.z > boundSize.x) {
                float median = halvedBounds.min.z = (bounds.max.z + bounds.min.z) * 0.5f;
                newSpherePivot = SplitSpheres(spheres, sphereIDs, spherePivot, median, RightSplitSpheresZ);
            }
            
            // Get new bounding box
            AABB rightAABB = AABB(spheres[sphereIDs[0]]);
            for (int s = 1; s < newSpherePivot; ++s)
                rightAABB.Extend(spheres[sphereIDs[s]]);
            
            // PrintWhiteSpace(level);
            // fprintf(stderr, "halvedBounds %s\n", halvedBounds.ToString().c_str());
            // PrintWhiteSpace(level);
            // fprintf(stderr, "RightAABB %s\n", rightAABB.ToString().c_str());            
            // PrintWhiteSpace(level);
            // fprintf(stderr, "InterAABB %s\n", Intersection(rightAABB, halvedBounds).ToString().c_str());            

            // determine active rays
            int newRayPivot = SplitRays(rays, rayIDs, rayPivot, rightAABB);

            // recurse
            Dacrt(rays, rayIDs, newRayPivot,
                  spheres, sphereIDs, newSpherePivot,
                  maxTs, spheresHit, Intersection(rightAABB, halvedBounds));
        }
    }

    --level;
}

void Radiance(Ray* rays, Sphere* spheres, int* spheresHit, Color* colors) {
    for (int i = 0; i < WIDTH * HEIGHT * samples; i++) {
        const int shID = spheresHit[i];
        if (shID < 0) {
            colors[i] = Color(0,0,0);
            return;
        }
        
        const Sphere& sphere = spheres[spheresHit[i]];
        colors[i] = sphere.color;
    }
}

int main(int argc, char *argv[]){
    sqrtSamples = argc == 2 ? atoi(argv[1]) : 2; // # samples
    samples = sqrtSamples * sqrtSamples;

    int rayCount = WIDTH * HEIGHT * samples;
    
    // Create rays
    Ray* rays = CreateRays();
    float* maxTs = NULL;
    ResetTs(&maxTs);
    int* rayIDs = new int[WIDTH * HEIGHT * samples];
    for (int r = 0; r < WIDTH * HEIGHT * samples; ++r)
        rayIDs[r] = r;

    // Create spheres
    Sphere* spheres = CreateSpheres();
    int* sphereIDs = new int[SPHERES];
    for (int s = 0; s < SPHERES; ++s)
        sphereIDs[s] = s;
    int* spheresHit = new int[WIDTH * HEIGHT * samples];
    for (int s = 0; s < WIDTH * HEIGHT * samples; ++s)
        spheresHit[s] = -1;

    fprintf(stderr, "Created %d rays and %d spheres\n", samples * WIDTH * HEIGHT, SPHERES);

    Color* colors = new Color[WIDTH * HEIGHT * samples];

    AABB sceneBounds = AABB(spheres[sphereIDs[0]]);
    for (int s = 1; s < SPHERES; ++s)
        sceneBounds.Extend(spheres[sphereIDs[s]]);

    Dacrt(rays, rayIDs, WIDTH * HEIGHT * samples,
          spheres, sphereIDs, SPHERES,
          maxTs, spheresHit, sceneBounds);
    Radiance(rays, spheres, spheresHit, colors);

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
    
}
