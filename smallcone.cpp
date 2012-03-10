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
#include "Scenes.h"

enum Axis {X = 0, Y = 1, Z = 2, U = 3, V = 4};

struct BoundedRay {
    HyperRay hyperRay;
    float t; // The progress of the ray
    float tMax; // Comparable to the tMax used in kd-tree traversal

    BoundedRay()
        : hyperRay(HyperRay()), t(0.0f), tMax(1e30) {}

    BoundedRay(const Ray& ray, const float t = 0.0f, const float tMax = 1e30)
        : hyperRay(HyperRay(ray)), t(0.0f), tMax(tMax) {}

    BoundedRay(const HyperRay& ray, const float t = 0.0f, const float tMax = 1e30)
        : hyperRay(ray), t(0.0f), tMax(tMax) {}

    inline Vector3 Origin() const { return hyperRay.Origin(); }
    inline Vector3 Position() const { return ToRay().PositionAt(t); }
    inline Vector5 PointAtT() const { return Vector5(Position(), hyperRay.point.u, hyperRay.point.v); }
    inline Vector3 Direction() const { return hyperRay.Direction(); }
        
    inline Ray ToRay() const { return hyperRay.ToRay(); }

    inline std::string ToString() {
        std::ostringstream out;
        out << "[" << ToRay().ToString() << ", t: " << t << ", tMax: " << tMax << "]";;
        return out.str();
    }

};

inline float SimpleIntersect(const Cone& c, const Ray& r, const float radius) {
    const float eps = 1e-4;
    Vector3 dir = c.apex - r.origin;
    float b = Dot(dir, r.dir);
    float det = b*b - Dot(dir, dir) + radius * radius;
    if (det < 0) return 0; else det = sqrt(det);
    float t = b + det;
    return t > eps ? t : 0.0f;
}

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

    HyperCube(const SignedAxis axis, const vector<BoundedRay>& boundedRays, 
              const vector<int>::iterator rayIndexBegin, const int rayOffset) 
        : axis(axis) {
        // cube = AABPenteract(boundedRays[*rayIndexBegin].PointAtT());
        // for (int r = 1; r < rayOffset; ++r)
        //     cube.Extent(boundedRays[rayIndexBegin[r]].PointAtT());
        cube = AABPenteract(boundedRays[*rayIndexBegin].hyperRay.point);
        for (int r = 1; r < rayOffset; ++r)
            cube.Extent(boundedRays[rayIndexBegin[r]].hyperRay.point);
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
    inline std::string ToString() {
        std::ostringstream out;
        out << "[t: " << t << ", sphereID: " << sphereID << "]";
        return out.str();
    }
};

struct Fragment {
    Vector3 emission;
    int depth;
    Vector3 f; // What is this I wonder
    Fragment() : emission(Vector3(0,0,0)), depth(0), f(Vector3(1,1,1)) {}
};

//const int WIDTH = 160, HEIGHT = 120;
//const int WIDTH = 640, HEIGHT = 480;
const int WIDTH = 320, HEIGHT = 240;
int sqrtSamples;
int samples;

long exhaustives = 0;
long distanceDacrt = 0;
long spreadDacrt = 0;

inline int Index(const int x, const int y, const int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(const int x, const int y, const int subX, const int subY) {
    return (x + y * WIDTH) * samples + subX + subY * sqrtSamples;
}

inline std::vector<BoundedRay> CreateRays() {
    Ray cam(Vector3(50,52,295.6), Vector3(0,-0.042612,-1).Normalize()); // cam pos, dir
    Vector3 cx = Vector3(WIDTH * 0.5135 / HEIGHT, 0, 0);
    Vector3 cy = (cx.Cross(cam.dir)).Normalize() * 0.5135;

    std::vector<BoundedRay> rays = std::vector<BoundedRay>(WIDTH * HEIGHT * samples);
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
                    rays[Index(x,y,subX,subY)] = BoundedRay(HyperRay(charles));
                }
        }
    }

    return rays;
}

void SimpleShade(vector<BoundedRay>& rays, const vector<int>& rayIndices,
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
            rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + reflect * 0.01f, reflect)));
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
                rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + reflect * 0.1f, reflect)));
            } else {

                Vector3 tDir = (ray.dir * nnt - norm * ((into?1.0f:-1.0f) * (ddn*nnt+sqrt(cos2t)))).Normalize();
                float a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn : Dot(tDir, norm));
                float Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re;
                float P=0.25f + 0.5f * Re; 
                float RP = Re / P, TP = Tr / (1.0f-P);
                if (Rand01() < P) // reflection
                    rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + reflect * 0.01f, reflect)));
                else 
                    rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + tDir * 0.01f, tDir)));
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

void Shade(vector<BoundedRay>& rays, vector<int>& rayIndices,
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
                newRayDir = reflect;
            } else {
                            Vector3 tDir = (ray.dir * nnt - norm * ((into?1.0f:-1.0f) * (ddn*nnt+sqrt(cos2t)))).Normalize();
                float a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn : Dot(tDir, norm));
                float Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re;
                float P=0.25f + 0.5f * Re; 
                float RP = Re / P, TP = Tr / (1.0f-P);
                if (Rand01() < P) // reflection
                    newRayDir = reflect;
                else 
                    newRayDir = tDir;
            }
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

        rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + newRayDir * 0.05f, newRayDir)));
        frags[rayID]->emission += frags[rayID]->f * sphere.emission;
        frags[rayID]->f = frags[rayID]->f * f;
        nextIndices[nextOffset++] = rayID;
    }
}

struct PartitionDoneRays {
    const vector<BoundedRay>& rays;
    PartitionDoneRays(const vector<BoundedRay>& rays)
        : rays(rays) {}
    bool operator()(const int rayID) const {
        return rays[rayID].t < rays[rayID].tMax;
    }
};

inline int Exhaustive(vector<BoundedRay> &rays, vector<int> &rayIndices, const int indexOffset, const int indexCount,
                       const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                       vector<Hit> &hits) {

    for (int i = indexOffset; i < indexOffset + indexCount; ++i) {
        const int rayID = rayIndices[i];
        const Ray charles = rays[rayID].ToRay();
        Hit hit = hits[rayID];
        for (int s = sphereOffset; s < sphereOffset + sphereCount; ++s) {
            ++exhaustives;
            const Sphere sphere = spheres[sphereIDs[s]];
            const float t = sphere.Intersect(charles);
            if (0 < t && t < hit.t)
                hit = Hit(t, sphereIDs[s]);
        }
        hits[rayID] = hit;
        // @TODO t is stored in both hits and rays. Remove the one from hits?
        rays[rayID].t = std::min(rays[rayID].tMax, hit.t);
    }
    
    vector<int>::iterator begin = rayIndices.begin() + indexOffset;
    vector<int>::iterator pivot = std::partition(begin, begin + indexCount, 
                                                 PartitionDoneRays(rays));
    return pivot - begin;
};

int Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
           vector<Hit> &hits);

struct PartitionRaysByU {
    const vector<BoundedRay>& rays;
    const float value;
    PartitionRaysByU(const vector<BoundedRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].hyperRay.point.u <= value; }
};

struct PartitionRaysByV {
    const vector<BoundedRay>& rays;
    const float value;
    PartitionRaysByV(const vector<BoundedRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].hyperRay.point.v <= value; }
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

inline int DacrtBySpread(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
                          vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
                          const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                          vector<Hit> &hits) {

    ++spreadDacrt;
    
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

    return rayOffset +  rayCount;
}

/**
 * Partition rays by their distance to the apex.
 */
struct PartitionRaysByDistance {
    const Vector3 apex;
    const float minDistSquared;
    const float maxDistSquared;
    const vector<BoundedRay>& rays;
    const vector<Hit>& hits;
    PartitionRaysByDistance(const Vector3 a, const float min, const float max, 
                            const vector<BoundedRay>& r, const vector<Hit>& h) 
        : apex(a), minDistSquared(min*min), maxDistSquared(max*max), rays(r), hits(h) {}
    bool operator()(int i) { 
        Hit h = hits[i];
        if (h.sphereID != -1) { // Ray has already hit something, 
            Vector3 pos = rays[i].ToRay().PositionAt(h.t);
            float dd = (apex - pos).LengthSquared();
            return minDistSquared < dd;
        } else { // Ray hasn't hit anything. Include it if it is below the maxDistance
            float dd = (apex - rays[i].Origin()).LengthSquared();
            return dd < maxDistSquared;
        }
    }
};

struct CalcTMax {
    const Cone cone;
    vector<BoundedRay>& rays;
    const float radius;
    CalcTMax(const Cone& cone, vector<BoundedRay>& rays, const float radius)
        : cone(cone), rays(rays), radius(radius) {}
    void operator() (const int i) {
        const Ray charles = rays[i].ToRay();
        rays[i].tMax = SimpleIntersect(cone, charles, radius);
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
inline int DacrtByDistance(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange, 
                           vector<BoundedRay> &rays, vector<int> &rayIDs, int rayOffset, const int rayCount,
                           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                           vector<Hit> &hits) {

    ++distanceDacrt;

    // Calc min and max while partitioning spheres by cone?
    // -- or -- 
    // Calc splitting position while partitioning spheres by cone?
    
    const int SPLITS = 2;
    const float range = coneRange / SPLITS;
    const vector<int>::iterator sphereEnd = sphereIDs.begin() + sphereCount;
    for (int i = 0; i < SPLITS; ++i) {
        const float min = coneMin + i * range;
        // If its the final partition (i == SPLITS-1) then set range to infinity
        // to make sure all rays and spheres are included.
        const float partitionMax = i == (SPLITS-1) ? 1e30 : min+range;

        // Partition the rays according to min and max
        const vector<int>::iterator rayBegin = rayIDs.begin() + rayOffset;
        const vector<int>::iterator rayPivot = 
            std::partition(rayBegin, rayBegin + rayCount,
                           PartitionRaysByDistance(cone.apex, min, partitionMax,
                                                   rays, hits));
        const int newRayCount = rayPivot - rayBegin;
        
        std::for_each(rayBegin, rayBegin + newRayCount,
                      CalcTMax(cone, rays, partitionMax));

        // Partition spheres according to min and max.
        const vector<int>::iterator sphereBegin = sphereIDs.begin() + sphereOffset;
        const vector<int>::iterator spherePivot = 
            std::partition(sphereBegin, sphereEnd, 
                           PartitionSpheresByDistance(cone.apex, min, partitionMax, spheres));
        const int newSphereCount = spherePivot - sphereBegin;
        
        // @TODO For now we don't recalculate cube and cone. It costs more than we gain.
        // const HyperCube newCube = HyperCube(cube.axis, rays, rayBegin, newRayCount);
        // const Cone newCone = newCube.ConeBounds();

        // Incomment this when I fix it!
        /*rayOffset += */ Dacrt(cube, cone, level+1, min, range,
                           rays, rayIDs, rayOffset, newRayCount,
                           spheres, sphereIDs, sphereOffset, newSphereCount, 
                           hits);
        
        // for (int i = -1; i < level; ++i) std::cout << "  ";        
        // cout << "rayOffset: " << rayOffset << endl;
    }
    
    return rayOffset;
}

int Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
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
        
        return Exhaustive(rays, rayIDs, rayOffset, rayCount,
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
        
        // @TODO Better decision method
        if (level % 3 == 0)
            return DacrtByDistance(cube, cone, level, coneMin, coneRange,
                                   rays, rayIDs, rayOffset, rayCount,
                                   spheres, sphereIDs, sphereOffset, sphereCount,
                                   hits);
        else
            return DacrtBySpread(cube, cone, level, coneMin, coneRange,
                                 rays, rayIDs, rayOffset, rayCount,
                                 spheres, sphereIDs, sphereOffset, sphereCount,
                                 hits);
    }
}

struct SortRayIndicesByAxis {
    const vector<BoundedRay>& rays;
    SortRayIndicesByAxis(const vector<BoundedRay>& rs) : rays(rs) {}
    bool operator()(int i, int j) { return rays[i].hyperRay.axis < rays[j].hyperRay.axis; }
};



int main(int argc, char *argv[]){

    /*
    {
        float tMax = 1e30;
        vector<BoundedRay> rays = vector<BoundedRay>(5);
        rays[0] = BoundedRay(Ray(Vector3(0,0,-1), Vector3(4,0,-1).Normalize()), 0.0f, tMax);
        rays[1] = BoundedRay(Ray(Vector3(0,0,-0.5f), Vector3(4,0,-0.5f).Normalize()), 0.0f, tMax);
        rays[2] = BoundedRay(Ray(Vector3(0,0, 0), Vector3(1,0,0).Normalize()), 0.0f, tMax);
        rays[3] = BoundedRay(Ray(Vector3(0,0, 0.5f), Vector3(4,0,0.5f).Normalize()), 0.0f, tMax);
        rays[4] = BoundedRay(Ray(Vector3(0,0, 1.0f), Vector3(4,0,1).Normalize()), 0.0f, tMax);
        vector<int> rayIndices = vector<int>(5);
        for (int i = 0; i < rayIndices.size(); ++i) rayIndices[i] = i;
        vector<Hit> hits = vector<Hit>(rays.size());
        
        vector<Sphere> spheres = vector<Sphere>(3);
        spheres[0] = Sphere(0.5f, Vector3(1,0,0));
        spheres[1] = Sphere(0.5f, Vector3(4,0,2));
        spheres[2] = Sphere(0.5f, Vector3(1,0,-1));
        vector<int> sphereIDs = vector<int>(spheres.size());
        for (int i = 0; i < sphereIDs.size(); ++i) sphereIDs[i] = i;

        // int offset = Exhaustive(rays, rayIndices, 0, rayIndices.size(),
        //                         spheres, sphereIDs, 0, sphereIDs.size(),
        //                         hits);
        // cout << "offset: " << offset << endl;
        // for (int i = 0; i < rayIndices.size(); ++i)
        //     cout << rays[rayIndices[i]].ToString() << " hit " << hits[rayIndices[i]].ToString() << endl;

        const HyperCube cube(posX, rays, rayIndices.begin(), rayIndices.size());
        cout << "hyber cube: " << cube.ToString() << endl;
        const Cone cone = cube.ConeBounds();
        cout << "cone: " << cone.ToString() << endl;

        float min = 4.12f; // Because!
        float range = 6.0f; // why not!

            int offset = DacrtByDistance(cube, cone, 0, min, range, 
                                         rays, rayIndices, 0, rayIndices.size(),
                                         spheres, sphereIDs, 0, sphereIDs.size(),
                                         hits);
        cout << "offset: " << offset << endl;
        for (int i = 0; i < rayIndices.size(); ++i)
            cout << rays[rayIndices[i]].ToString() << " hit " << hits[rayIndices[i]].ToString() << endl;
        
        return 0;
    }
    */

    sqrtSamples = argc == 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    vector<BoundedRay> rays = CreateRays();

    vector<Sphere> spheres = Scenes::CornellBox();

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
            while(rayIndex < rayIndices.size() && rays[rayIndices[rayIndex]].hyperRay.axis == a)
                ++rayIndex;
            int rayCount = rayIndex - rayOffset;
            std::cout << "  RayCount is " << rayCount << " for axis " << a << 
                " [exhaustives: " << exhaustives << ", distanceDacrt: " << distanceDacrt << 
                ", spreadDacrt: " << spreadDacrt << std::endl;
            
            if (rayCount == 0) continue;
            
            const HyperCube hc((SignedAxis)a, rays, rayIndices.begin(), rayCount);
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

    SavePPM("image.ppm", WIDTH, HEIGHT, cs);

    return 0;
}
