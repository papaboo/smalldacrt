// smallcone. A rayTracer using a cone ray hierarchy as described by Roger et
// al. (2007) and with an implementation based on smallpt, a path tracer by
// Kevin Beason.  (But written slightly more human readable)

// Compile ./make smallcone
// Usage: ./small 4 16 && xv image.ppm

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
#include "HyperCube.h"
#include "HyperRay.h"
#include "AABB.h"
#include "AABPenteract.h"
#include "Cone.h"
#include "Utils.h"
#include "Scenes.h"
#include "Plane.h"

struct BoundedRay {
    HyperRay hyperRay;
    float t; // The progress of the ray

    BoundedRay()
        : hyperRay(HyperRay()), t(0.0f) {}

    BoundedRay(const Ray& ray, const float t = 0.0f)
        : hyperRay(HyperRay(ray)), t(0.0f) {}

    BoundedRay(const HyperRay& ray, const float t = 0.0f)
        : hyperRay(ray), t(0.0f) {}

    inline Vector3 Origin() const { return hyperRay.Origin(); }
    inline Vector3 Position() const { return ToRay().PositionAt(t); }
    inline Vector5 PointAtT() const { return Vector5(Position(), hyperRay.point.u, hyperRay.point.v); }
    inline Vector3 Direction() const { return hyperRay.Direction(); }
        
    inline Ray ToRay() const { return hyperRay.ToRay(); }

    inline std::string ToString() {
        std::ostringstream out;
        out << "[" << ToRay().ToString() << ", t: " << t << "]";;
        return out.str();
    }

};

inline HyperCube CreateHyperCube(const SignedAxis axis, const vector<BoundedRay>& boundedRays, 
                                 const vector<int>::iterator rayIndexBegin, const int rayOffset) {
    AABPenteract cube = AABPenteract(boundedRays[*rayIndexBegin].hyperRay.point);
    for (int r = 1; r < rayOffset; ++r)
        cube.Extent(boundedRays[rayIndexBegin[r]].hyperRay.point);
    return HyperCube(axis, cube);
}

struct Hit {
    int sphereID;
    Hit() : sphereID(-1) {}
    Hit(const int s) 
        : sphereID(s) {}
    inline std::string ToString() {
        std::ostringstream out;
        out << "[sphereID: " << sphereID << "]";
        return out.str();
    }
};

struct Fragment {
    Vector3 emission;
    int depth;
    Vector3 f; // What is this I wonder
    Fragment() : emission(Vector3(0,0,0)), depth(0), f(Vector3(1,1,1)) {}
};

const int WIDTH = 512, HEIGHT = 512;
//const int WIDTH = 640, HEIGHT = 480;
//const int WIDTH = 320, HEIGHT = 240;
//const int WIDTH = 1440, HEIGHT = 900;
int sqrtSamples;
int samples;

long exhaustives = 0;
long distanceDacrt = 0;
long rayDacrtRays = 0;
long rayDacrtSpheres = 0;

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
        const Vector3 hitPos = ray.origin + ray.dir * rays[rayID].t;
        const Vector3 norm = (hitPos - sphere.position).Normalize();
        const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;

        if (++(frags[rayID]->depth) > 5) {
            float mod = 0.5f + 0.5f * nl.y;
            frags[rayID]->emission = sphere.color * mod;
            continue;
        }

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
        const Vector3 hitPos = ray.origin + ray.dir * rays[rayID].t;
        const Vector3 norm = (hitPos - sphere.position).Normalize();
        const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;
        Color f = sphere.color;
        const float maxRefl = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z;
        if (++(frags[rayID]->depth) > 5)
            if (Rand01() < maxRefl)
            //if (false)
                f = f * (1 / maxRefl); 
            else {
                frags[rayID]->emission += frags[rayID]->f * sphere.emission;
                continue;
            }
        
        switch(sphere.reflection) {
        case SPECULAR: {
            Vector3 newRayDir = ray.dir - nl * 2 * Dot(nl, ray.dir);
            rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + nl * 0.02f, newRayDir)));
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
                rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + nl * 0.02f, reflect)));
            } else {
                            Vector3 tDir = (ray.dir * nnt - norm * ((into?1.0f:-1.0f) * (ddn*nnt+sqrt(cos2t)))).Normalize();
                float a=nt-nc, b=nt+nc, R0=a*a/(b*b), c = 1-(into?-ddn : Dot(tDir, norm));
                float Re=R0+(1-R0)*c*c*c*c*c,Tr=1-Re;
                float P=0.25f + 0.5f * Re; 
                float RP = Re / P, TP = Tr / (1.0f-P);
                if (Rand01() < P)// reflection
                    rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + nl * 0.02f, reflect)));
                else
                    rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + nl * -0.02f, tDir)));
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
            Vector3 newRayDir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrtf(1-r2)).Normalize();
            rays[rayID] = BoundedRay(HyperRay(Ray(hitPos + nl * 0.02f, newRayDir)));
            break;
        }

        frags[rayID]->emission += frags[rayID]->f * sphere.emission;
        frags[rayID]->f = frags[rayID]->f * f;
        nextIndices[nextOffset++] = rayID;
    }
}

inline void Exhaustive(vector<BoundedRay> &rays, vector<int> &rayIndices, const int indexOffset, const int indexCount,
                       const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                       vector<Hit> &hits) {

    for (int i = indexOffset; i < indexOffset + indexCount; ++i) {
        const int rayID = rayIndices[i];
        const Ray charles = rays[rayID].ToRay();
        float tHit = rays[rayID].t == 0.0f ? 1e30 : rays[rayID].t;
        Hit hit = hits[rayID];
        for (int s = sphereOffset; s < sphereOffset + sphereCount; ++s) {
            ++exhaustives;
            const Sphere sphere = spheres[sphereIDs[s]];
            const float t = sphere.Intersect(charles);
            if (0 < t && t < tHit) {
                hit = Hit(sphereIDs[s]);
                tHit = t;
            }
        }
        hits[rayID] = hit;
        rays[rayID].t = tHit;
    }
}

void Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
           vector<Hit> &hits);

struct PartitionSpheresByCone {
    const vector<Sphere>& spheres;
    const Cone cone;
    float &min, &max;
    float invSinAngle, cosAngleSqr;
    PartitionSpheresByCone(const vector<Sphere>& spheres, const Cone cone, float &min, float &max)
        : spheres(spheres), cone(cone), min(min), max(max), 
          invSinAngle(1.0f / std::sin(cone.spreadAngle)), 
          cosAngleSqr(std::cos(cone.spreadAngle) * std::cos(cone.spreadAngle)) {}
    bool operator()(int i) { 
        if (cone.DoesIntersect(spheres[i], invSinAngle, cosAngleSqr)) {
            float dist = (cone.apex - spheres[i].position).Length();
            min = std::min(min, dist - spheres[i].radius);
            max = std::max(max, dist - spheres[i].radius);
            return true;
        } else
            return false;
    }
};


struct PartitionRaysByX {
    const vector<BoundedRay>& rays;
    const float value;
    PartitionRaysByX(const vector<BoundedRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].hyperRay.point.x <= value; }
};
struct PartitionRaysByY {
    const vector<BoundedRay>& rays;
    const float value;
    PartitionRaysByY(const vector<BoundedRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].hyperRay.point.y <= value; }
};
struct PartitionRaysByZ {
    const vector<BoundedRay>& rays;
    const float value;
    PartitionRaysByZ(const vector<BoundedRay>& r, const float v)
        : rays(r), value(v) {}
    bool operator()(int i) { return rays[i].hyperRay.point.z <= value; }
};
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


/**
 * A simple implementation based on **** without caching.
 */
inline void DacrtByRays(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
                       vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
                       const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
                       vector<Hit> &hits) {

    rayDacrtRays += rayCount;
    rayDacrtSpheres += sphereCount;
    
    // Split the hypercube along the largest dimension and partition the ray ids
    float xRange = cube.cube.x.Range();
    float yRange = cube.cube.y.Range();
    float zRange = cube.cube.z.Range();
    float uRange = cube.cube.u.Range();
    float vRange = cube.cube.v.Range();
    float maxSpread = std::max(uRange, vRange);
    float maxPos = std::max(xRange, std::max(yRange, zRange));

    vector<int>::iterator begin = rayIDs.begin() + rayOffset;
    vector<int>::iterator rayPivot;
    if (maxSpread > maxPos) { // Split along the ray directions
        rayPivot = uRange > vRange ?
            std::partition(begin, begin + rayCount,
                           PartitionRaysByU(rays, cube.cube.u.Middle())) :
            std::partition(begin, begin + rayCount,
                           PartitionRaysByV(rays, cube.cube.v.Middle()));
    } else  { // Split along the ray positions
        rayPivot = 
            xRange > yRange && xRange > zRange ?
            std::partition(begin, begin + rayCount,
                           PartitionRaysByX(rays, cube.cube.x.Middle())) :
            (yRange > zRange ? 
             std::partition(begin, begin + rayCount,
                            PartitionRaysByY(rays, cube.cube.y.Middle())) :
             std::partition(begin, begin + rayCount,
                            PartitionRaysByZ(rays, cube.cube.z.Middle())));
    }
    int newRayCount = rayPivot - begin;
    

    // Cube and cone for the lower side
    HyperCube lowerCube = CreateHyperCube(cube.axis, rays, begin, newRayCount);
    Cone lowerCone = lowerCube.ConeBounds();
    
    // Partition spheres according to cone
    float min = coneMin + coneRange, max = coneMin;
    begin = sphereIDs.begin() + sphereOffset;
    vector<int>::iterator spherePivot = 
        std::partition(begin, begin + sphereCount, 
                       PartitionSpheresByCone(spheres, lowerCone, min, max));

    // min should be the maximum of the sphere nearest sphere and the distance from the ray AABB to the cones apex.
    AABB aabb = lowerCube.GetAABB();
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
    HyperCube upperCube = CreateHyperCube(cube.axis, rays, rayIDs.begin() + upperRayOffset, upperRayCount);
    Cone upperCone = upperCube.ConeBounds();
    
    // Partition spheres according to cone
    min = coneMin + coneRange; max = coneMin;
    begin = sphereIDs.begin() + sphereOffset;
    spherePivot = 
        std::partition(begin, begin + sphereCount, 
                       PartitionSpheresByCone(spheres, upperCone, min, max));

    // min should be the maximum of the sphere nearest sphere and the distance from the ray AABB to the cones apex.
    aabb = upperCube.GetAABB();
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
    const vector<BoundedRay>& rays;
    const vector<Hit>& hits;
    PartitionRaysByDistance(const Vector3 a, const float min, const float max, 
                            const vector<BoundedRay>& r, const vector<Hit>& h) 
        : apex(a), minDistSquared(min*min), maxDistSquared(max*max), rays(r), hits(h) {}
    bool operator()(int i) { 
        Hit h = hits[i];
        if (h.sphereID != -1) { // Ray has already hit something, 
            Vector3 pos = rays[i].Position();
            float dd = (apex - pos).LengthSquared();
            return minDistSquared < dd;
        } else { // Ray hasn't hit anything. Include it if it is below the maxDistance
            float dd = (apex - rays[i].Origin()).LengthSquared();
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
        
        // Partition spheres according to min and max.
        const vector<int>::iterator sphereBegin = sphereIDs.begin() + sphereOffset;
        const vector<int>::iterator spherePivot = 
            std::partition(sphereBegin, sphereEnd, 
                           PartitionSpheresByDistance(cone.apex, min, partitionMax, spheres));
        const int newSphereCount = spherePivot - sphereBegin;
        
        // @TODO For now we don't recalculate cube and cone. It costs more than we gain.
        // const HyperCube newCube = HyperCube(cube.axis, rays, rayBegin, newRayCount);
        // const Cone newCone = newCube.ConeBounds();

        Dacrt(cube, cone, level+1, min, range,
              rays, rayIDs, rayOffset, newRayCount,
              spheres, sphereIDs, sphereOffset, newSphereCount, 
              hits);
    }
}

void Dacrt(const HyperCube& cube, const Cone& cone, const int level, const float coneMin, const float coneRange,
           vector<BoundedRay> &rays, vector<int> &rayIDs, const int rayOffset, const int rayCount,
           const vector<Sphere> &spheres, vector<int> &sphereIDs, const int sphereOffset, const int sphereCount,
           vector<Hit> &hits) {

    const bool print = false;
    
    // The termination criteria expreses that once the exhaustive O(r * s)
    // search is faster than performing another split we terminate recursion.
    if ((long)rayCount * (long)sphereCount <= (long)16 * ((long)rayCount + (long)sphereCount)) {
        if (print) {
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << (rayCount * sphereCount) << " <= " << (16 * (rayCount + sphereCount)) << endl;
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << "Exhaustive with index valeus: " << rayOffset << " -> " << rayCount << 
                ", sphere: " << sphereOffset << " -> " << sphereCount << 
                ", [min: " << coneMin << ", range: " << coneRange << "]" << endl;
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << " +---Cube: " << cube.ToString() << endl;
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << " +---Cone: " << cone.ToString() << endl;
        }
        
        Exhaustive(rays, rayIDs, rayOffset, rayCount,
                   spheres, sphereIDs, sphereOffset, sphereCount, hits);
    } else {

        if (print) {
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << "Dacrt with ray valeus: " << rayOffset << " -> " << rayCount << 
                ", sphere: " << sphereOffset << " -> " << sphereCount << 
                ", [min: " << coneMin << ", range: " << coneRange << "]" << endl;
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << " +---Cube: " << cube.ToString() << endl;
            for (int i = -1; i < level; ++i) cout << "  ";
            cout << " +---Cone: " << cone.ToString() << endl;
        }
        
        // @TODO Better decision method
        if (level % 3 == -1)
            DacrtByDistance(cube, cone, level, coneMin, coneRange,
                            rays, rayIDs, rayOffset, rayCount,
                            spheres, sphereIDs, sphereOffset, sphereCount,
                            hits);
        else
            DacrtByRays(cube, cone, level, coneMin, coneRange,
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


void RayTrace(vector<Fragment*>& rayFrags, vector<Sphere>& spheres) {
    vector<BoundedRay> rays = CreateRays();
    
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
            
            if (rayCount == 0) continue;
            
            const HyperCube hc = CreateHyperCube((SignedAxis)a, rays, rayIndices.begin(), rayCount);
            const Cone cone = hc.ConeBounds();
            
            // Partition spheres according to hypercube
            vector<int> sphereIDs(spheres.size());
            float min = 1e30, max = 0;
            for (int i = 0; i < sphereIDs.size(); ++i)
                sphereIDs[i] = i;

            vector<int>::iterator spherePivot = 
                std::partition(sphereIDs.begin(), sphereIDs.end(), 
                               PartitionSpheresByCone(spheres, hc.ConeBounds(), min, max));
            float dist = (hc.GetAABB().ClosestPointOnSurface(cone.apex) - cone.apex).Length();
            min = std::max(min, dist);
            int sphereCount = spherePivot - sphereIDs.begin();
            int sphereOffset = 0;

            // perform dacrt
            Dacrt(hc, cone, 0, min, max - min, 
                  rays, rayIndices, rayOffset, rayCount,
                  spheres, sphereIDs, sphereOffset, sphereCount,
                  hits);
            
            // Offset to beginning of next ray bundle.
            rayOffset += rayCount;

            std::cout << "  RayCount is " << rayCount << " for axis " << a << 
                " [exhaustives: " << exhaustives << 
                ", ray dacrt [rays : " << rayDacrtRays << ", spheres: " << rayDacrtSpheres << "]]" << std::endl;
        }

        std::sort(rayIndices.begin(), rayIndices.end());
        // Apply shading
        std::cout << "  Apply shading" << std::endl;        
        Shade(rays, rayIndices, rayFrags, spheres, hits, nextRayIndices, nextOffset);
        nextRayIndices.resize(nextOffset);
        
        rayIndices = nextRayIndices;
    }
}


int main(int argc, char *argv[]){
    
    sqrtSamples = argc >= 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;

    // vector<HyperRay> rays = vector<HyperRay>(4);
    // float uMax = -0.1f, uMin = -0.2f, vMax = 0.3f, vMin = -0.4f;
    // rays[0] = HyperRay(Ray(Vector3(-1,-1,-1), Vector3(1, uMax, vMax)));
    // rays[1] = HyperRay(Ray(Vector3(-1,-1,-1), Vector3(1, uMax, vMin)));
    // rays[2] = HyperRay(Ray(Vector3( 1, 1, 1), Vector3(1, uMin, vMax))); 
    // rays[3] = HyperRay(Ray(Vector3( 1, 1, 1), Vector3(1, uMin, vMin)));
    // HyperCube c = HyperCube(posX, rays.begin(), 4);

    // cout << c.ToString() << endl;

    // cout << c.UpperUBoundingPlane().ToString() << endl;

    // Vector3 normal = Vector3(1, uMax, vMax).Cross(Vector3(1, uMax, vMin)).Normalize();
    // Vector3 point = Vector3(1,1,1);
    
    // cout << "normal: " << normal.ToString() << endl;
    // cout << "point: " << point.ToString() << endl;
    // cout << "plane: " << Plane(normal, point).ToString() << endl;

    // return 0;
    
    int iterations = argc >= 3 ? atoi(argv[2]) : 1; // # iterations
    Color* cs = NULL;

    //vector<Sphere> spheres = Scenes::CornellBox();
    vector<Sphere> spheres = Scenes::SphereBox();

    Fragment* frags = new Fragment[WIDTH * HEIGHT * samples];
    vector<Fragment*> rayFrags(WIDTH * HEIGHT * samples);
    for (int i = 0; i < iterations; ++i) {
        for (int f = 0; f < WIDTH * HEIGHT * samples; ++f)
            frags[f] = Fragment();
        for (int f = 0; f < WIDTH * HEIGHT * samples; ++f)
            rayFrags[f] = frags + f;
        
        RayTrace(rayFrags, spheres);
        
        // *********** CREATE IMAGE ****************
        
        // Combine colors into image
        if (cs == NULL) {
            cs = new Color[WIDTH * HEIGHT];
            for (int x = 0; x < WIDTH; ++x)
                for (int y = 0; y < HEIGHT; ++y) {
                    Color c = Color(0,0,0);
                    for (int s = 0; s < samples; ++s)
                        c += frags[Index(x,y,s)].emission;
                    cs[x + y * WIDTH] = c / samples;
                }
        } else {
            float mod = float(i) / (i+1.0f);
            float invMod = 1.0f - mod;
            cout << i+1 << " of " << iterations << ": mod: " << mod << ", invMod: " << invMod << endl;
            for (int x = 0; x < WIDTH; ++x)
                for (int y = 0; y < HEIGHT; ++y) {
                    Color c = Color(0,0,0);
                    for (int s = 0; s < samples; ++s)
                        c += frags[Index(x,y,s)].emission;
                    cs[x + y * WIDTH] = cs[x + y * WIDTH] * mod + c * invMod / samples;
                }
        }
    }   

    SavePPM("coneimage.ppm", WIDTH, HEIGHT, cs);
    
    return 0;
}
