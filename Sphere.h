#ifndef _SMALL_SPHERE_H_
#define _SMALL_SPHERE_H_

#include "Vector.h"

enum ReflectionType {DIFFUSE, SPECULAR, REFRACTING};

struct Sphere {
    float radius;
    Vector3 position, emission, color;
    ReflectionType reflection;

    Sphere() {}

    Sphere(const float r, const Vector3& p, const Vector3& e = Vector3(0,0,0), 
           const Vector3& c = Vector3(0,0,1), const ReflectionType rt = DIFFUSE) 
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

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[radius: " << radius << ", position: " << position.ToString() << ", color: " << color.ToString() << "]";
        return out.str();
    }
};

#endif
