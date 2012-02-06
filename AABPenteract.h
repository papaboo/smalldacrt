#ifndef _SMALL_AAB_PENTERACT_H_
#define _SMALL_AAB_PENTERACT_H_

#include "HyperRay.h"

struct Bound1D {
    float min, max;

    Bound1D() : min(0.0f), max(0.0f) {}

    Bound1D(const float min, const float max) 
        : min(min), max(max) {}

    inline float Middle() const {
        return (min + max) * 0.5f;
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[min: " << min << ", max: " << max << "]";
        return out.str();
    }
};

struct AABPenteract {
    
    Bound1D x, y, z, u, v;

    AABPenteract() 
        : x(Bound1D()), y(Bound1D()), z(Bound1D()), u(Bound1D()), v(Bound1D()) {}

    AABPenteract(const Vector5& p)
        : x(Bound1D(p.x, p.x)), y(Bound1D(p.y, p.y)), z(Bound1D(p.z, p.z)), 
          u(Bound1D(p.u, p.u)), v(Bound1D(p.v, p.v)) {}
    
    AABPenteract(const Bound1D x, const Bound1D y, const Bound1D z, 
                 const Bound1D u, const Bound1D v) 
        : x(x), y(y), z(z), u(u), v(v) {}

    inline void Extent(const Vector5& p) {
        x.min = std::min(x.min, p.x); x.max = std::max(x.max, p.x);
        y.min = std::min(y.min, p.y); y.max = std::max(y.max, p.y);
        z.min = std::min(z.min, p.z); z.max = std::max(z.max, p.z);
        u.min = std::min(u.min, p.u); u.max = std::max(u.max, p.u);
        v.min = std::min(v.min, p.v); v.max = std::max(v.max, p.v);
    }

    inline bool Contains(const HyperRay& ray) const {
        return x.min <= ray.point.x && ray.point.x <= x.max && 
            y.min <= ray.point.y && ray.point.y <= y.max && 
            z.min <= ray.point.z && ray.point.z <= z.max && 
            u.min <= ray.point.u && ray.point.u <= u.max && 
            v.min <= ray.point.v && ray.point.v <= v.max;
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[x" << x.ToString() << ", y" << y.ToString() << ", z" << z.ToString() << ", u" << u.ToString() << ", v" << v.ToString() << "]";
        return out.str();
    }
};

#endif
