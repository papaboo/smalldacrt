#ifndef _SMALL_AAB_PENTERACT_H_
#define _SMALL_AAB_PENTERACT_H_

struct Bound1D {
    float min, max;

    Bound1D() : min(0.0f), max(0.0f) {}

    Bound1D(const float min, const float max) 
        : min(min), max(max) {}

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

    AABPenteract(const Bound1D x, const Bound1D y, const Bound1D z, 
                 const Bound1D u, const Bound1D v) 
        : x(x), y(y), z(z), u(u), v(v) {}

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[x" << x.ToString() << ", y" << y.ToString() << ", z" << z.ToString() << ", u" << u.ToString() << ", v" << v.ToString() << "]";
        return out.str();
    }
};

#endif
