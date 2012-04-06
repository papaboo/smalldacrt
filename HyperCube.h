#ifndef _SMALL_HYPER_CUBE_H_
#define _SMALL_HYPER_CUBE_H_

#include "AABB.h"
#include "AABPenteract.h"
#include "Cone.h"
#include "HyperRay.h"
#include "Plane.h"

#include <vector>
using std::vector;

struct HyperCube {
    AABPenteract cube;
    SignedAxis axis;
    
    HyperCube(const SignedAxis axis = negZ, const AABPenteract cube = AABPenteract()) 
        : cube(cube), axis(axis) {}

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

    inline AABB GetAABB() const {
        return AABB(Vector3(cube.x.min, cube.y.min, cube.z.min),
                    Vector3(cube.x.max, cube.y.max, cube.z.max));
    }

    inline Plane LowerUBoundingPlane() const {
        switch(axis) {
        case posX: 
        case negX: {
            float xDir = axis == posX ? 1 : -1;
            Vector3 v0(xDir, cube.u.min, cube.v.min);
            Vector3 v1(xDir, cube.u.min, cube.v.max);
            bool useXMax = cube.u.min < 0.0f ^ axis == posX;
            Vector3 point = Vector3(useXMax ? cube.x.max : cube.x.min, 
                                cube.y.min, cube.z.min);
            return Plane(v1.Cross(v0).Normalize(), point);
        }
        case posY: 
        case negY: {
            float yDir = axis == posY ? 1 : -1;
            Vector3 v0(cube.u.min, yDir, cube.v.min);
            Vector3 v1(cube.u.min, yDir, cube.v.max);
            bool useYMax = cube.u.min < 0.0f ^ axis == posY;
            Vector3 point = Vector3(cube.x.min, 
                                    useYMax ? cube.y.max : cube.y.min, cube.z.min);
            return Plane(v1.Cross(v0).Normalize(), point);
        }   
        case posZ: 
        case negZ: {
            float zDir = axis == posZ ? 1 : -1;
            Vector3 v0(cube.u.min, cube.v.min, zDir);
            Vector3 v1(cube.u.min, cube.v.max, zDir);
            bool useZMax = cube.u.min < 0.0f ^ axis == posZ;
            Vector3 point = Vector3(cube.x.min, cube.y.min,
                                    useZMax ? cube.z.max : cube.z.min);
            return Plane(v1.Cross(v0).Normalize(), point);
        }
        }
    }
        
    inline Plane UpperUBoundingPlane() const {
        switch(axis) {
        case posX: 
        case negX: {
            float xDir = axis == posX ? 1 : -1;
            Vector3 v0(xDir, cube.u.max, cube.v.min);
            Vector3 v1(xDir, cube.u.max, cube.v.max);
            bool useXMax = cube.u.max > 0.0f ^ axis == posX;
            Vector3 point = Vector3(useXMax ? cube.x.max : cube.x.min, 
                                cube.y.max, cube.z.max);
            return Plane(v1.Cross(v0).Normalize(), point);
        }
        case posY: 
        case negY: {
            float yDir = axis == posY ? 1 : -1;
            Vector3 v0(cube.u.max, yDir, cube.v.min);
            Vector3 v1(cube.u.max, yDir, cube.v.max);
            bool useYMax = cube.u.max > 0.0f ^ axis == posY;
            Vector3 point = Vector3(cube.x.max, 
                                    useYMax ? cube.y.max : cube.y.min, 
                                    cube.z.max);
            return Plane(v1.Cross(v0).Normalize(), point);
        }   
        case posZ: 
        case negZ: {
            float zDir = axis == posZ ? 1 : -1;
            Vector3 v0(cube.u.max, cube.v.min, zDir);
            Vector3 v1(cube.u.max, cube.v.max, zDir);
            bool useZMax = cube.u.max > 0.0f ^ axis == posZ;
            Vector3 point = Vector3(cube.x.max, cube.y.max, 
                                    useZMax ? cube.z.max : cube.z.min);
            return Plane(v1.Cross(v0).Normalize(), point);
        }
        }
    }
        
    inline std::string ToString() const {
        std::ostringstream out;
        out << "[axis: " << axis << ", cube: " << cube.ToString() << "]";
        return out.str();
    }

private:
    inline Vector3 F(const SignedAxis a, const float u, const float v) const {
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

#endif
