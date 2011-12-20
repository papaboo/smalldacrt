// Vector

#ifndef _SMALL_VECTOR_H_
#define _SMALL_VECTOR_H_

#include <string>
#include <sstream>

template <class T>
struct Vec3 {
    T x, y, z;

    Vec3() 
        : x(0), y(0), z(0) {}

    Vec3(T pX, T pY, T pZ) 
        : x(pX), y(pY), z(pZ) {}

    template<class C>
    Vec3(Vec3<C> o) 
        : x(o.x), y(o.y), z(o.z) {}

    inline Vec3<T> operator+(const T rhs) const {
        return Vec3<T>(x + rhs, y + rhs, z + rhs);
    }
    inline Vec3<T> operator+(const Vec3<T>& rhs) const {
        return Vec3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
    }

    inline void operator+=(const Vec3<T>& rhs) {
        x += rhs.x; y += rhs.y; z += rhs.z;
    }

    inline Vec3<T> operator-(const T rhs) const {
        return Vec3<T>(x - rhs, y - rhs, z - rhs);
    }
    inline Vec3<T> operator-(const Vec3<T>& rhs) const {
        return Vec3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
    }

    inline Vec3<T> operator*(const float rhs) const {
        return Vec3<T>(x * rhs, y * rhs, z * rhs);
    }

    inline Vec3<T> operator*(const Vec3<T>& rhs) const {
        return Vec3<T>(x * rhs.x, y * rhs.y, z * rhs.z);
    }

    inline Vec3<T> operator/(const float rhs) const {
        return Vec3<T>(x / rhs, y / rhs, z / rhs);
    }
    inline Vec3<T> operator/(const Vec3<T> rhs) const {
        return Vec3<T>(x / rhs.x, y / rhs.y, z / rhs.z);
    }

    inline float Length() {
        return sqrt(x * x + y * y + z * z);
    }

    inline Vec3<T>& Normalize(){ 
        return *this = *this / Length();
    }

    inline float Dot(const Vec3<T>& rhs) {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    
    inline Vec3<T> Cross(const Vec3<T>& rhs) {
        return Vec3<T>(y * rhs.z - z * rhs.y, 
                       z * rhs.x - x * rhs.z,
                       x * rhs.y - y * rhs.x);
    }

    inline std::string ToString() {
        std::ostringstream out;
        out << "[" << x << ", " << y << ", " << z << "]";
        return out.str();
    }
};

template <class T>
inline float Dot(const Vec3<T>& lhs, const Vec3<T>& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

typedef Vec3<float> Vector3;
typedef Vec3<double> Color;

#endif
