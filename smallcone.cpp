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
using std::cout;
using std::endl;

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
    
    HyperCube() : cube(AABPenteract()), axis(negZ) {}

    HyperCube(const SignedAxis axis, vector<HyperRay>::iterator rayBegin, int rayOffset) 
        : cube(AABPenteract(rayBegin[0].point)), axis(axis) {
        for (int r = 1; r < rayOffset; ++r)
            cube.Extent(rayBegin[r].point);
    }

    HyperCube(const SignedAxis axis, const vector<HyperRay>& hyperRays, 
              const vector<int>::iterator rayIndexBegin, const int rayOffset) 
        : axis(axis) {
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
        out << "[axis: " << axis << ", cube: " << cube.ToString() << "]";
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

//const int WIDTH = 32, HEIGHT = 24;
//const int WIDTH = 64, HEIGHT = 48;
const int WIDTH = 320, HEIGHT = 240;
int sqrtSamples;
int samples;

inline int Index(const int x, const int y, const int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(const int x, const int y, const int subX, const int subY) {
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
        float radius = 1.0f + Rand01();
        Vector3 pos = Vector3(Rand01() * 100.0 , Rand01() * 100.0 , Rand01() * 100.0 + 50.0);
        Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
        float reflParam = Rand01();
        ReflectionType rt = reflParam < 0.1f ? REFRACTING : (reflParam > 0.9f ? SPECULAR : DIFFUSE);
        
        spheres[s] = Sphere(radius, pos, Vector3(),  c, rt);
    }

    return spheres;
}

void SimpleShade(vector<HyperRay>& rays, const vector<int>& rayIndices,
                 const vector<Fragment*>& frags, 
                 const vector<Sphere>& spheres, const vector<Hit>& hits,
                 vector<int>& nextIndices, int &nextOffset) {
    
    for (int i = 0; i < rayIndices.size(); ++i) {
        const int rayID = rayIndices[i];
        const int sphereID = hits[rayID].sphereID;

        if (sphereID == -1) continue;

        const Ray ray = rays[rayID].ToRay();
        const Sphere sphere = spheres[sphereID];
        const Vector3 hitPos = ray.origin + ray.dir * hits[rayID].t;
        const Vector3 norm = (hitPos - sphere.position).Normalize();
        const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;

        switch(sphere.reflection) {
        case SPECULAR: {
            Vector3 reflect = ray.dir - nl * 2 * Dot(nl, ray.dir);
            rays[rayID] = HyperRay(Ray(hitPos + reflect * 0.01f, reflect));
            nextIndices[nextOffset++] = rayID;
            break;
        }
        case REFRACTING: {
            Vector3 reflect = ray.dir - norm * 2.0f * Dot(norm, ray.dir);
            bool into = Dot(norm, nl) > 0.0f;
            float nc = 1.0f; 
            float nt = 1.5f;
            float nnt = into ? nc/nt : nt/nc;
            float ddn = Dot(ray.dir, nl);
            float cos2t = 1.0f - nnt * nnt * (1.0f - ddn * ddn);
            // If total internal reflection
            if (cos2t < 0.0f) {
                rays[rayID] = HyperRay(Ray(hitPos + reflect * 0.1f, reflect));
            } else {

                Vector3 tDir = (ray.dir * nnt - norm * ((into?1.0f:-1.0f) * (ddn*nnt+sqrt(cos2t)))).Normalize();
                float a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn : Dot(tDir, norm));
                float Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re;
                float P=0.25f + 0.5f * Re; 
                float RP = Re / P, TP = Tr / (1.0f-P);
                if (Rand01() < P) // reflection
                    rays[rayID] = HyperRay(Ray(hitPos + reflect * 0.01f, reflect));
                else 
                    rays[rayID] = HyperRay(Ray(hitPos + tDir * 0.01f, tDir));
                nextIndices[nextOffset++] = rayID;
            }
            break;
        }
        default:
            float mod = 0.5f + 0.5f * nl.y;
            frags[rayID]->emission = sphere.color * mod;
            break;
        }            
    }
}

void Shade(vector<HyperRay>& rays, vector<int>& rayIndices,
           const vector<Fragment*>& frags, 
           const vector<Sphere>& spheres, const vector<Hit>& hits,
           vector<int>& nextIndices, int &nextOffset) {

    for (int i = 0; i < rayIndices.size(); ++i) {
        const int rayID = rayIndices[i];
        const int sphereID = hits[rayID].sphereID;

        if (sphereID == -1) {
            // std::cout << "ray: " << rays[rayID].ToRay().ToString() << " completely missed" << std::endl;
            continue;
        }

        const Ray ray = rays[rayID].ToRay();
        const Sphere sphere = spheres[sphereID];
        const Vector3 hitPos = ray.origin + ray.dir * hits[rayID].t;
        const Vector3 norm = (hitPos - sphere.position).Normalize();
        const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;
        Color f = sphere.color;
        const float maxRefl = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z;
        if (++(frags[rayID]->depth) > 5)
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
            float r1 = 2 * PI * Rand01();
            float r2 = Rand01(); 
            float r2s = sqrtf(r2);
            // Tangent space ?
            Vector3 w = nl; 
            Vector3 u = ((fabsf(w.x) > 0.1 ? Vector3(0,1,0) : Vector3(1,0,0)).Cross(w)).Normalize();
            Vector3 v = w.Cross(u);
            newRayDir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrtf(1-r2)).Normalize();
            break;
        }

        rays[rayID] = HyperRay(Ray(hitPos + newRayDir * 0.05f, newRayDir));
        frags[rayID]->emission += frags[rayID]->f * sphere.emission;
        frags[rayID]->f = frags[rayID]->f * f;
        nextIndices[nextOffset++] = rayID;
    }
}

// TODO replace sphere indices with sphere pointers, since we're only using that
// index for spheres.
inline void Exhaustive(const int level,
                       const vector<HyperRay> &rays, vector<int> &rayIndices, const int indexOffset, const int indexCount,
                       const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                       vector<Hit> &hits) {

    for (int i = indexOffset; i < indexOffset + indexCount; ++i) {
        const int rayID = rayIndices[i];
        const Ray charles = rays[rayID].ToRay();
        for (int s = sphereOffset; s < sphereOffset + sphereCount; ++s) {
            const Sphere sphere = spheres[sphereIDs[s]];
            const float t = sphere.Intersect(charles);
            if (0 < t && t < hits[rayID].t)
                hits[rayID] = Hit(t, sphereIDs[s]);
        }
    }
};

void Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           const vector<HyperRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
           vector<Hit> &hits);

struct PartitionRaysByU {
    const vector<HyperRay>& rays;
    const float value;
    PartitionRaysByU(const vector<HyperRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].point.u <= value; }
};

struct PartitionRaysByV {
    const vector<HyperRay>& rays;
    const float value;
    PartitionRaysByV(const vector<HyperRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].point.v <= value; }
};

struct PartitionSpheresByCone {
    const vector<Sphere>& spheres;
    const Cone cone;
    float &min, &max;
    float sinAngle, cosAngleSqr;
    PartitionSpheresByCone(const vector<Sphere>& spheres, const Cone cone, float &min, float &max)
        : spheres(spheres), cone(cone), min(min), max(max), 
          sinAngle(std::sin(cone.spreadAngle)), 
          cosAngleSqr(std::cos(cone.spreadAngle) * std::cos(cone.spreadAngle)) {}
    bool operator()(int i) { 
        if (cone.DoesIntersect(spheres[i], sinAngle, cosAngleSqr)) {
            float dist = (cone.apex - spheres[i].position).Length();
            min = std::min(min, dist - spheres[i].radius);
            max = std::max(max, dist + spheres[i].radius);
            return true;
        } else
            return false;
    }
};

inline void DacrtBySpread(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
                          const vector<HyperRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
                          const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                          vector<Hit> &hits) {
    
    // Split the hypercube along either u or v and partition the ray ids
    float uRange = cube.cube.u.Range();
    float vRange = cube.cube.v.Range();
    vector<int>::iterator begin = rayIDs.begin() + rayOffset;
    vector<int>::iterator rayPivot = uRange > vRange ?
        std::partition(begin, begin + rayCount,
                       PartitionRaysByU(rays, cube.cube.u.Middle())) :
        std::partition(begin, begin + rayCount,
                       PartitionRaysByV(rays, cube.cube.v.Middle()));
    int newRayCount = rayPivot - begin;
    
    // Cube and cone for the lower side
    HyperCube lowerCube = HyperCube(cube.axis, rays, begin, newRayCount);
    Cone lowerCone = lowerCube.ConeBounds();
    
    // Partition spheres according to cone
    float min = coneMin + coneRange, max = coneMin;
    begin = sphereIDs.begin() + sphereOffset;
    vector<int>::iterator spherePivot = 
        std::partition(begin, begin + sphereCount, 
                       PartitionSpheresByCone(spheres, lowerCone, min, max));

    // min should be the maximum of the sphere nearest sphere and the distance from the ray AABB to the cones apex.
    AABB aabb = AABB(Vector3(lowerCube.cube.x.min, lowerCube.cube.y.min, lowerCube.cube.z.min),
                     Vector3(lowerCube.cube.x.max, lowerCube.cube.y.max, lowerCube.cube.z.max));
    float dist = (aabb.ClosestPointOnSurface(lowerCone.apex) - lowerCone.apex).Length();
    min = std::max(min, dist);

    int newSphereCount = spherePivot - begin;
    
    // Perform Dacrt
    Dacrt(lowerCube, lowerCone, level+1, min, std::min(coneRange, max - min), 
          rays, rayIDs, rayOffset, newRayCount,
          spheres, sphereIDs, sphereOffset, newSphereCount, 
          hits);
    
    
    // Cube and cone for the upper side
    int upperRayOffset = rayOffset + newRayCount;
    int upperRayCount = rayCount - newRayCount;
    HyperCube upperCube = HyperCube(cube.axis, rays, rayIDs.begin() + upperRayOffset, upperRayCount);
    Cone upperCone = upperCube.ConeBounds();
    
    // Partition spheres according to cone
    min = coneMin + coneRange; max = coneMin;
    begin = sphereIDs.begin() + sphereOffset;
    spherePivot = 
        std::partition(begin, begin + sphereCount, 
                       PartitionSpheresByCone(spheres, upperCone, min, max));

    // min should be the maximum of the sphere nearest sphere and the distance from the ray AABB to the cones apex.
    aabb = AABB(Vector3(upperCube.cube.x.min, upperCube.cube.y.min, upperCube.cube.z.min),
                Vector3(upperCube.cube.x.max, upperCube.cube.y.max, upperCube.cube.z.max));
    dist = (aabb.ClosestPointOnSurface(upperCone.apex) - upperCone.apex).Length();
    min = std::max(min, dist);
    newSphereCount = spherePivot - begin;
    
    // Perform Dacrt
    Dacrt(upperCube, upperCone, level+1, min, std::min(coneRange, max - min), 
          rays, rayIDs, upperRayOffset, upperRayCount,
          spheres, sphereIDs, sphereOffset, newSphereCount, 
          hits);
}

/**
 * Partition rays by their distance to the apex.
 */
struct PartitionRaysByDistance {
    const Vector3 apex;
    const float minDistSquared;
    const float maxDistSquared;
    const vector<HyperRay>& rays;
    const vector<Hit>& hits;
    PartitionRaysByDistance(const Vector3 a, const float min, const float max, 
                            const vector<HyperRay>& r, const vector<Hit>& h) 
        : apex(a), minDistSquared(min*min), maxDistSquared(max*max), rays(r), hits(h) {}
    bool operator()(int i) { 
        Hit h = hits[i];
        if (h.sphereID != -1) { // Ray has already hit something, 
            Vector3 pos = rays[i].ToRay().PositionAt(h.t);
            float dd = (apex - pos).LengthSquared();
            return minDistSquared < dd;
        } else { // Ray hasn't hit anything. Include it if it is below the maxDistance
            float dd = (apex - rays[i].Position()).LengthSquared();
            return dd < maxDistSquared;
        }
    }
};

struct PartitionSpheresByDistance {
    const Vector3 apex;
    const float min;
    const float max;
    const vector<Sphere>& spheres;
    PartitionSpheresByDistance(const Vector3 a, const float min,
                               const float max, const vector<Sphere>& ss) 
        : apex(a), min(min), max(max), spheres(ss) {}
    bool operator()(int i) {
        float l = (apex - spheres[i].position).Length();
        return min < l + spheres[i].radius && l - spheres[i].radius < max;
    }
};

/**
 * Partition based on the distance from the apex.
 */
inline void DacrtByDistance(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange, 
                            const vector<HyperRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
                            const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                            vector<Hit> &hits) {

    // Calc min and max while partitioning spheres by cone?
    // -- or -- 
    // Calc splitting position while partitioning spheres by cone?
    
    const int SPLITS = 4;
    const float range = coneRange / SPLITS;
    for (int i = 0; i < SPLITS; ++i) {
        const float min = coneMin + i * range;
        // If its the final partition (i == SPLITS-1) then set range to infinity
        // to make sure all rays and spheres are included.
        const float partitionMax = i == (SPLITS-1) ? 1e30 : min+range;

        // Partition the rays according to min and max
        vector<int>::iterator begin = rayIDs.begin() + rayOffset;
        vector<int>::iterator rayPivot = 
            std::partition(begin, begin + rayCount,
                           PartitionRaysByDistance(cone.apex, min, partitionMax,
                                                   rays, hits));
        int newRayCount = rayPivot - begin;

        // Partition spheres according to min and max.
        // @TODO use a sphereOffset to skip already tested spheres, by doing a non
        // disjunct partitioning.
        begin = sphereIDs.begin() + sphereOffset;
        vector<int>::iterator spherePivot = 
            std::partition(begin, begin + sphereCount, 
                           PartitionSpheresByDistance(cone.apex, min, partitionMax, spheres));
        int newSphereCount = spherePivot - begin;
        
        // @ TODO recalculate cube 'n cone? Will be done in DacBySpread anyway

        Dacrt(cube, cone, level+1, min, range,
              rays, rayIDs, rayOffset, newRayCount,
              spheres, sphereIDs, sphereOffset, newSphereCount, 
              hits);
    }
}

void Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           const vector<HyperRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
           vector<Hit> &hits) {

    const bool print = false;
    
    // The termination criteria expreses that once the exhaustive O(r * s)
    // search is faster than performing another split we terminate recursion.
    if (rayCount * sphereCount <= 16 * (rayCount + sphereCount)) {
        if (print) {
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << "Exhaustive with index valeus: " << rayOffset << " -> " << rayCount << 
                ", sphere: " << sphereOffset << " -> " << sphereCount << 
                ", [min: " << coneMin << ", range: " << coneRange << "]" << std::endl;
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << " +---Cube: " << cube.ToString() << std::endl;
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << " +---Cone: " << cone.ToString() << std::endl;
        }
        
        Exhaustive(level, rays, rayIDs, rayOffset, rayCount,
                   spheres, sphereIDs, sphereOffset, sphereCount, hits);
    } else {

        if (print) {
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << "Dacrt with ray valeus: " << rayOffset << " -> " << rayCount << 
                ", sphere: " << sphereOffset << " -> " << sphereCount << 
                ", [min: " << coneMin << ", range: " << coneRange << "]" << std::endl;
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << " +---Cube: " << cube.ToString() << std::endl;
            for (int i = -1; i < level; ++i) std::cout << "  ";
            std::cout << " +---Cone: " << cone.ToString() << std::endl;
        }
        
        if (level % 3 == 0)
            DacrtByDistance(cube, cone, level, coneMin, coneRange,
                            rays, rayIDs, rayOffset, rayCount,
                            spheres, sphereIDs, sphereOffset, sphereCount,
                            hits);
        else
            DacrtBySpread(cube, cone, level, coneMin, coneRange,
                          rays, rayIDs, rayOffset, rayCount,
                          spheres, sphereIDs, sphereOffset, sphereCount,
                          hits);
    }
}

bool CompareRayAxis (HyperRay lhs, HyperRay rhs) { return (lhs.axis < rhs.axis);}

struct SortRayIndicesByAxis {
    const vector<HyperRay>& rays;
    SortRayIndicesByAxis(const vector<HyperRay>& rs) : rays(rs) {}
    bool operator()(int i, int j) { return rays[i].axis < rays[j].axis; }
};

int main(int argc, char *argv[]){

    sqrtSamples = argc == 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    vector<HyperRay> rays = CreateRays();

    vector<Sphere> spheres = CreateSpheres();

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

        vector<int> nextRayIndices(rayIndices.size());
        int nextOffset = 0;
        vector<Hit> hits(rays.size());

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
            const Cone cone = hc.ConeBounds();
            // std::cout << "HyberCube " << hc.ToString() << "\n   -> " << hc.ConeBounds().ToString() << std::endl;
            
            // Partition spheres according to hypercube
            vector<int> sphereIDs(spheres.size());
            float min = 1e30, max = 0;
            for (int i = 0; i < sphereIDs.size(); ++i)
                sphereIDs[i] = i;

            vector<int>::iterator spherePivot = 
                std::partition(sphereIDs.begin(), sphereIDs.end(), 
                               PartitionSpheresByCone(spheres, hc.ConeBounds(), min, max));
            min = std::max(min, 0.0f);
            int sphereCount = spherePivot - sphereIDs.begin();
            int sphereOffset = 0;

            // perform dacrt
            Dacrt(hc, cone, 0, min, max - min, 
                  rays, rayIndices, rayOffset, rayCount,
                  spheres, sphereIDs, sphereOffset, sphereCount,
                  hits);
            
            // Offset to beginning of next ray bundle.
            rayOffset += rayCount;
        }

        std::sort(rayIndices.begin(), rayIndices.end());
        // Apply shading
        std::cout << "  Apply shading" << std::endl;        
        Shade(rays, rayIndices, rayFrags, spheres, hits, nextRayIndices, nextOffset);
        nextRayIndices.resize(nextOffset);
        
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
