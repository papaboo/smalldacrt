#ifndef _SMALL_RAY_H_
#define _SMALL_RAY_H_

struct Ray {
    Vector3 origin;
    Vector3 dir;

    Ray() : origin(Vector3()), dir(Vector3()) {}

    Ray(const Vector3& o, const Vector3& d) 
        : origin(o), dir(d) {}

    inline Vector3 PositionAt(float t) {
        return origin + dir * t;
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[origin: " << origin.ToString() << ", direction: " << dir.ToString() + "]";
        return out.str();
    }
};

#endif
