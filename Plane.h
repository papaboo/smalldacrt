#ifndef _SMALL_PLANE_H_
#define _SMALL_PLANE_H_

#include "Vector.h"

#include <string>
#include <iostream>

/**
 * The plane is defined the the plane equation Ax + By + Cz + D = 0. The normal
 * of the plane is given by [A, B, C] and the distance is D. The normal is
 * assumed to be normalized on construction, so the plane is in Hessian normal
 * form.
 */
class Plane {
private:
    Vector3 normal;
    float distance;

public:

    /**
     * Plane constructor
     *
     * @param normal. The normal of the plane. Assumed to be normalized by the caller.
     * @distance. The distance to origo modulated by the length of the normal
     * (if not normalized). D in the plane equation.
     */
    Plane(Vector3 normal = Vector3(1,0,0), float distance = 0)
        : normal(normal), distance(distance) {}

    /**
     * Plane constructor
     *
     * @param normal. The normal of the plane. Assumed to be normalized by the caller.
     * @point. A point in the plane.
     */
    Plane(Vector3 normal, Vector3 point)
        : normal(normal), distance(-Dot(normal, point)) {}


    inline const Vector3 GetNormal() const { return normal; }
    inline const float GetDistance() const { return distance; }

    /**
     * Calculates the distance from the point p to the plane. If the planes
     * normal is not normalized then the distance needs to be divided by the
     * length of the normal.
     *
     * @return The distance between the plane and p.
     */
    inline float DistanceTo(const Vector3& p) {
        return Dot(normal, p) + distance;
    }

    inline std::string ToString(const int precision = 2) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << "[normal: " << normal.ToString(precision) << ", distance: " << distance << "]";
        return out.str();
    }

};

#endif //_SMALL_PLANE_H_
