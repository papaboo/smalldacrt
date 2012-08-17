#ifndef _SMALL_SPHERE_CONE_H_
#define _SMALL_SPHERE_CONE_H_

#include <iomanip>
#include <math.h>
#include <string>
#include <iostream>

#include "Ray.h"
#include "Sphere.h"
#include "Vector.h"

struct SphereCone {

    Vector3 apex;
    float spreadAngle;
    Vector3 dir;
    float radius;

    /**
     * Construct a cone from it's apex, direction and spreadangle. The direction
     * is assumed to be normalized.
     *
     * TODO Complain if reflex cone
     */
    SphereCone(const Vector3& apex, const Vector3& dir, const float spreadAngle, const float radius)
        : apex(apex), dir(dir), spreadAngle(spreadAngle), radius(radius) {}
        

    inline bool DoesIntersect(const Sphere& sphere) const {

        const float sinToAngle = std::sin(spreadAngle);
        const float cosToAngleSqr = std::cos(spreadAngle) * cos(spreadAngle);
        
        return DoesIntersect(sphere, 1.0f / sinToAngle, cosToAngleSqr);
    }

    /**
     * http://www.geometrictools.com/Documentation/IntersectionSphereCone.pdf
     */
    inline bool DoesIntersect(const Sphere& sphere, const float invSinToAngle, 
                              const float cosToAngleSqr) const {
        float totalRadius = sphere.radius + radius;
        const Vector3 U = apex - dir * (totalRadius * invSinToAngle);
        Vector3 D = sphere.position - U;
        float dSqr = Dot(D,D);
        float e = Dot(dir, D);
        
        if (e > 0.0f && e*e >= dSqr * cosToAngleSqr) {
            D = sphere.position - apex;
            dSqr = Dot(D,D);
            e = -Dot(dir, D);
            const float sinSqr = 1.0f - cosToAngleSqr;
            if (e > 0 && e*e >= dSqr * sinSqr)
                return dSqr <= totalRadius * totalRadius;
            else
                return true;
        }
        
        return false;
    }

    inline bool Includes(const Ray& ray) const {
        // Test if the rays direction is included in the cone.
        if (Dot(dir, ray.dir) < spreadAngle) return false;
        
        // Is the rays origin included in the cone.
        return Dot((ray.origin - apex).Normalize(), ray.dir) > spreadAngle;
    }

    /**
     * @param p is the number of decimals that should be written to the string.
     */
    inline std::string ToString(const int p = 2) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(p) << "[apex: " << apex.ToString(p) << ", angle: " << spreadAngle << ", direction: " << dir.ToString(p) << ", radius: " << radius << "]";
        return out.str();
    }

};

#endif
