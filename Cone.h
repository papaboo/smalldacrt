#ifndef _SMALL_CONE_H_
#define _SMALL_CONE_H_

#include <iomanip>
#include <math.h>
#include <string>
#include <iostream>

#include "Vector.h"
#include "Ray.h"
#include "Sphere.h"

struct Cone {

    Vector3 apex;
    float spreadAngle;
    Vector3 dir;

    /**
     * Construct a cone from it's apex, direction and spreadangle. The direction
     * is assumed to be normalized.
     */
    Cone(const Vector3& apex, const Vector3& dir, const float spreadAngle)
        : apex(apex), dir(dir), spreadAngle(spreadAngle) {}

    /**
     * http://www.geometrictools.com/Documentation/IntersectionSphereCone.pdf
     */
    inline bool DoesIntersect(const Sphere& sphere) const {
        // @TODO Handle reflex cones by inversion
        
        const Vector3 U = apex - dir * (sphere.radius / sin(spreadAngle));
        const float cosSqr = cos(spreadAngle) * cos(spreadAngle);
        Vector3 D = sphere.position - U;
        float dSqr = Dot(D,D);
        float e = Dot(dir, D);
        
        if (e > 0.0f && e*e >= dSqr * cosSqr) {
            D = sphere.position - apex;
            dSqr = Dot(D,D);
            e = -Dot(dir, D);
            const float sinSqr = 1.0f - cosSqr;
            if (e > 0 && e*e >= dSqr * sinSqr)
                return dSqr <= sphere.radius * sphere.radius;
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
        out << std::fixed << std::setprecision(p) << "[apex: " << apex.ToString(p) << ", angle: " << spreadAngle << ", direction: " << dir.ToString(p) << "]";
        return out.str();
    }

};

inline Cone Union(const Cone& lhs, const Cone& rhs) {
    return lhs;
}

#endif
