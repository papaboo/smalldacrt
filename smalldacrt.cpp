// smalldacrt. A Divide and Conquer RayTracer as described by Benjamin More and
// with an implementation based on smallpt, a path tracer be Kevin Beason. (But
// written slightly more human readable)

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
#include "Ray.h"
#include "Sphere.h"
#include "AABB.h"
#include "Utils.h"

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

    for (int s = 9; s < SPHERES; ++s) {
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
    int head = 0, tail = spherePivot;
    while (head < tail) {
        while (head < tail && test(spheres[sphereIDs[head]].position, median))
            ++head;
        do
            --tail;
        while (head < tail && !test(spheres[sphereIDs[tail]].position, median));
        if (head < tail) 
            std::swap(*(sphereIDs + head), *(sphereIDs + tail));
    }

    return head;
}

bool LeftSplitSpheresX(Vector3& pos, float median) { return pos.x <= median; }
bool LeftSplitSpheresY(Vector3& pos, float median) { return pos.y <= median; }
bool LeftSplitSpheresZ(Vector3& pos, float median) { return pos.z <= median; }
bool RightSplitSpheresX(Vector3& pos, float median) { return pos.x > median; }
bool RightSplitSpheresY(Vector3& pos, float median) { return pos.y > median; }
bool RightSplitSpheresZ(Vector3& pos, float median) { return pos.z > median; }

int SplitRays(Ray* rays, int* rayIDs, int rayPivot, AABB& aabb) {
    int head = 0, tail = rayPivot;
    while (head < tail) {
        while (head < tail && aabb.Intersect(rays[rayIDs[head]]) > 0) 
            ++head;
        do 
            --tail; 
        while (head < tail && aabb.Intersect(rays[rayIDs[tail]]) <= 0);
        if (head < tail) 
            std::swap(*(rayIDs + head), *(rayIDs + tail));
    }
    return head;
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
            const Ray charles = rays[rayID];
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

    return 0;
}
