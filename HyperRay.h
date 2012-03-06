#ifndef _SMALL_HYPER_RAY_H_
#define _SMALL_HYPER_RAY_H_

#include <math.h>

#include "Ray.h"
#include "Sphere.h"

enum SignedAxis {posX = 0, negX = 1, posY = 2, negY = 3, posZ = 4, negZ = 5};

struct HyperRay { // For lack of a better name
    Vector5 point;
    SignedAxis axis;

    HyperRay() : point(Vector5(0,0,0,0,0)), axis(posX) {}

    HyperRay(const Vector5& p, const SignedAxis a) : point(p), axis(a) {}

    HyperRay(const Ray& charles) {
        point.x = charles.origin.x;
        point.y = charles.origin.y;
        point.z = charles.origin.z;
        
        Vector3 absDir = Vector3(fabs(charles.dir.x), fabs(charles.dir.y), fabs(charles.dir.z));
        if (absDir.x > absDir.y && absDir.x > absDir.z) { // x is dominant
            point.u = charles.dir.y / absDir.x;
            point.v = charles.dir.z / absDir.x;
            axis = charles.dir.x > 0.0f ? posX : negX;
        } else if (absDir.y > absDir.z) { // y is dominant
            point.u = charles.dir.x / absDir.y;
            point.v = charles.dir.z / absDir.y;
            axis = charles.dir.y > 0.0f ? posY : negY;
        } else { // z is dominant
            point.u = charles.dir.x / absDir.z;
            point.v = charles.dir.y / absDir.z;
            axis = charles.dir.z > 0.0f ? posZ : negZ;
        }
    }

    inline float Intersect(const Sphere& s) const {
        const float eps = 1e-4;
        const Vector3 dir = s.position - Vector3(point.x, point.y, point.z);
        const float b = Dot(dir, Direction().Normalize());
        float det = b*b - Dot(dir, dir) + s.radius*s.radius;
        if (det < 0) return 0; else det = sqrt(det);
        float t;
        return (t=b-det)>eps ? t : ((t=b+det)>eps ? t : 0);
    }

    inline Vector3 Origin() const {
        return Vector3(point.x, point.y, point.z);
    }

    inline Vector3 Direction() const {
        switch(axis) {
        case posX:
            return Vector3(1.0f, point.u, point.v);
        case negX:
            return Vector3(-1.0f, point.u, point.v);
        case posY:
            return Vector3(point.u, 1.0f, point.v);
        case negY:
            return Vector3(point.u, -1.0f, point.v);
        case posZ:
            return Vector3(point.u, point.v, 1.0f);
        case negZ:
            return Vector3(point.u, point.v, -1.0f);
        }        
    }

    inline Ray ToRay() const {
        return Ray(Origin(), 
                   Direction().Normalize());
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[axis: ";
        switch (axis) {
        case posX:
            out << "posX"; break;
        case negX:
            out << "negX"; break;
        case posY:
            out << "posY"; break;
        case negY:
            out << "negY"; break;
        case posZ:
            out << "posZ"; break;
        case negZ:
            out << "negZ"; break;
        };
        out << ", point: " << point.ToString() + "]";
        return out.str();
    }

};

#endif
