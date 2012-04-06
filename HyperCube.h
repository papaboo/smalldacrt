#ifndef _SMALL_HYPER_CUBE_H_
#define _SMALL_HYPER_CUBE_H_

#include "AABB.h"
#include "AABPenteract.h"
#include "Cone.h"
#include "HyperRay.h"
#include "Plane.h"

#include <limits>
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

    inline Plane UpperBoundingPlane(const Axis a) const {

        // Calculate the normal of the bounding plane
        Vector3 normal;
        switch(axis) {
        case posX: 
        case negX: {
            float xDir = axis == posX ? 1 : -1;
            switch(a) {
            case X:
                // Upper bounding plane is infinitely far away
                // cout << "    " << xDir << " X - X" << endl;
                return Plane(Vector3(-xDir, 0, 0), Vector3(xDir * std::numeric_limits<float>::max(), 0, 0));
            case Y: case U: {
                // cout << "    " << xDir << " X - Y or U" << endl;
                Vector3 v0(xDir, cube.u.max, cube.v.min);
                Vector3 v1(xDir, cube.u.max, cube.v.max);
                normal = v0.Cross(v1).Normalize() * xDir;
                break;
            }
            case Z: case V: {
                // cout << "    " << xDir << " X - Z or V" << endl;
                Vector3 v0(xDir, cube.u.max, cube.v.max);
                Vector3 v1(xDir, cube.u.min, cube.v.max);
                normal = v0.Cross(v1).Normalize() * xDir;
                break;
            }
            }
            break;
        }

        case posY: 
        case negY: {
            float yDir = axis == posY ? 1 : -1;
            switch(a) {
            case Y:
                // Upper bounding plane is infinitely far away
                // cout << "    " << yDir << " Y - Y" << endl;
                return Plane(Vector3(0, -yDir, 0), Vector3(0, yDir * std::numeric_limits<float>::max(), 0));
            case X: case U: {
                // cout << "    " << yDir << " Y - X or U" << endl;
                Vector3 v0(cube.u.max, yDir, cube.v.max);
                Vector3 v1(cube.u.max, yDir, cube.v.min);
                normal = v0.Cross(v1).Normalize() * yDir;
                break;
            }
            case Z: case V: {
                // cout << "    " << yDir << " Y - Z or V" << endl;
                Vector3 v0(cube.u.min, yDir, cube.v.max);
                Vector3 v1(cube.u.max, yDir, cube.v.max);
                normal = v0.Cross(v1).Normalize() * yDir;
                break;
            }
            }
            break;
        }

        case posZ: 
        case negZ: {
            float zDir = axis == posZ ? 1 : -1;
            switch(a) {
            case Z:
                // Upper bounding plane is infinitely far away
                // cout << "    " << zDir << " Z - Z" << endl;
                return Plane(Vector3(0, 0, -zDir), Vector3(0,0 , zDir * std::numeric_limits<float>::max()));
            case X: case U: {
                // cout << "    " << zDir << " Z - X or U" << endl;
                Vector3 v0(cube.u.max, cube.v.min, zDir);
                Vector3 v1(cube.u.max, cube.v.max, zDir);
                normal = v0.Cross(v1).Normalize() * zDir;
                break;
            }
            case Y: case V: {
                // cout << "    " << zDir << " Z - Y or V" << endl;
                Vector3 v0(cube.u.max, cube.v.max, zDir);
                Vector3 v1(cube.u.min, cube.v.max, zDir);
                normal = v0.Cross(v1).Normalize() * zDir;
                break;
            }
            }
            break;
        }

        }

        // Use the planes normal to find a point on the AABB
        Vector3 point = Vector3(normal.x < 0 ? cube.x.max : cube.x.min,
                                normal.y < 0 ? cube.y.max : cube.y.min,
                                normal.z < 0 ? cube.z.max : cube.z.min);
        
        return Plane(normal, point);
    }

    inline Plane LowerBoundingPlane(const Axis a) const {

        // Calculate the normal of the bounding plane
        Vector3 normal;
        switch(axis) {
        case posX: 
        case negX: {
            float xDir = axis == posX ? 1 : -1;
            switch(a) {
            case X:
                // Lower bounding plane is at the beginning of the cube
                return Plane(Vector3(xDir, 0, 0), Vector3(xDir < 0 ? cube.x.max : cube.x.min, cube.y.min, cube.z.min));
            case Y: case U: {
                Vector3 v0(xDir, cube.u.min, cube.v.max);
                Vector3 v1(xDir, cube.u.min, cube.v.min);
                normal = v0.Cross(v1).Normalize() * xDir;
                break;
            }
            case Z: case V: {
                Vector3 v0(xDir, cube.u.min, cube.v.min);
                Vector3 v1(xDir, cube.u.max, cube.v.min);
                normal = v0.Cross(v1).Normalize() * xDir;
                break;
            }
            }
            break;
        }

        case posY: 
        case negY: {
            float yDir = axis == posY ? 1 : -1;
            switch(a) {
            case Y:
                // Lower bounding plane is at the beginning of the cube
                return Plane(Vector3(0, yDir, 0), Vector3(cube.x.min, yDir < 0 ? cube.y.max : cube.y.min, cube.z.min));
            case X: case U: {
                Vector3 v0(cube.u.min, yDir, cube.v.min);
                Vector3 v1(cube.u.min, yDir, cube.v.max);
                normal = v0.Cross(v1).Normalize() * yDir;
                break;
            }
            case Z: case V: {
                Vector3 v0(cube.u.max, yDir, cube.v.min);
                Vector3 v1(cube.u.min, yDir, cube.v.min);
                normal = v0.Cross(v1).Normalize() * yDir;
                break;
            }
            }
            break;
        }

        case posZ: 
        case negZ: {
            float zDir = axis == posZ ? 1 : -1;
            switch(a) {
            case Z:
                // Lower bounding plane is at the beginning of the cube
                return Plane(Vector3(0, 0, zDir), Vector3(cube.x.min, cube.y.min, zDir < 0 ? cube.z.max : cube.z.min));
            case X: case U: {
                Vector3 v0(cube.u.min, cube.v.max, zDir);
                Vector3 v1(cube.u.min, cube.v.min, zDir);
                normal = v0.Cross(v1).Normalize() * zDir;
                break;
            }
            case Y: case V: {
                Vector3 v0(cube.u.min, cube.v.min, zDir);
                Vector3 v1(cube.u.max, cube.v.min, zDir);
                normal = v0.Cross(v1).Normalize() * zDir;
                break;
            }
            }
            break;
        }

        }

        // Use the planes normal to find a point on the AABB
        Vector3 point = Vector3(normal.x < 0 ? cube.x.max : cube.x.min,
                                normal.y < 0 ? cube.y.max : cube.y.min,
                                normal.z < 0 ? cube.z.max : cube.z.min);
        
        return Plane(normal, point);
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
