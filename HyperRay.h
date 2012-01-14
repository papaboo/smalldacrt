#ifndef _SMALL_HYPER_RAY_H_
#define _SMALL_HYPER_RAY_H_

#include "Ray.h"

enum Axis {posX = 0, negX = 1, posY = 2, negY = 3, posZ = 4, negZ = 5};

struct HyperRay { // For lack of a better name
    Vector5 point;
    Axis axis;
    HyperRay(const Vector5& p, Axis a) : point(p), axis(a) {}
    HyperRay(const Ray& charles) {
        point.x = charles.origin.x;
        point.y = charles.origin.y;
        point.z = charles.origin.z;
        
        Vector3 absDir = Vector3(std::fabs(charles.dir.x), std::fabs(charles.dir.y), std::fabs(charles.dir.z));
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
