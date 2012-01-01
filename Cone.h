#ifndef _SMALL_CONE_H_
#define _SMALL_CONE_H_

#include <math.h>

#include "Vector.h"
#include "Ray.h"
#include "Sphere.h"

struct Cone {

    Vector3 apex;
    float spreadAngle;
    Vector3 direction;
    float t;

    Cone(const Vector3& apex, const float spreadAngle, const Vector3& direction)
        : apex(apex), spreadAngle(spreadAngle), direction(direction), t(0) {}

    /**
     * http://www.geometrictools.com/Documentation/IntersectionSphereCone.pdf
     */
    inline bool DoesIntersect(const Sphere& sphere) const {
        Vector3 U = apex - direction * (sphere.radius / sin(spreadAngle));
        Vector3 D = sphere.position - U;
        float dSqr = Dot(D,D);
        float e = Dot(direction, D);
        float cosSqr = cos(spreadAngle) * cos(spreadAngle);
        
        if (e > 0.0f && e*e >= dSqr * cosSqr) {
            D = sphere.position - apex;
            dSqr = Dot(D,D);
            e = -Dot(direction, D);
            float sinSqr = sin(spreadAngle) * sin(spreadAngle);
            if (e > 0 && e*e >= dSqr * sinSqr)
                return dSqr <= sphere.radius * sphere.radius;
            else
                return true;
        }
        
        return false;
    }

};

inline Cone Union(const Cone& lhs, const Cone& rhs) {
    return lhs;
}

#endif
