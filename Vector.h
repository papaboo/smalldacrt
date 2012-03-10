// Vector

#ifndef _SMALL_VECTOR_H_
#define _SMALL_VECTOR_H_

#include <iomanip>
#include <string>
#include <iostream>

template <class T>
struct Vec3 {
    T x, y, z;

    Vec3() 
        : x(0), y(0), z(0) {}

    Vec3(const T pX, const T pY, const T pZ) 
        : x(pX), y(pY), z(pZ) {}

    template<class C>
    Vec3(const Vec3<C>& o) 
        : x(o.x), y(o.y), z(o.z) {}

    inline Vec3<T>& operator=(const Vec3<T>& rhs) {
        x = rhs.x; y = rhs.y; z = rhs.z;
        return *this;
    }

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

    inline T Length() const {
        return sqrt(x * x + y * y + z * z);
    }

    inline T LengthSquared() const {
        return x * x + y * y + z * z;
    }

    inline Vec3<T>& Normalize(){ 
        return *this = *this / Length();
    }

    inline T Dot(const Vec3<T>& rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    
    inline Vec3<T> Cross(const Vec3<T>& rhs) const {
        return Vec3<T>(y * rhs.z - z * rhs.y, 
                       z * rhs.x - x * rhs.z,
                       x * rhs.y - y * rhs.x);
    }

    inline std::string ToString(const int precision = 2) const {
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << "[" << x << ", " << y << ", " << z << "]";
        return out.str();
    }
};

typedef Vec3<float> Vector3;
typedef Vec3<double> Color;

template <class T>
struct Vec5 {
    T x, y, z, u, v;

    Vec5() 
        : x(0), y(0), z(0), u(0), v(0) {}

    Vec5(const T x, const T y, const T z, const T u, const T v) 
        : x(x), y(y), z(z), u(u), v(v) {}

    Vec5(const Vec3<T> xyz, const T u, const T v) 
        : x(xyz.x), y(xyz.y), z(xyz.z), u(u), v(v) {}

    template<class C>
    Vec5(const Vec5<C>& o) 
        : x(o.x), y(o.y), z(o.z), u(o.u), v(o.v) {}
    
    inline std::string ToString() const {
        std::ostringstream out;
        out << "[" << x << ", " << y << ", " << z << ", " << u << ", " << v << "]";
        return out.str();
    }
};

typedef Vec5<float> Vector5;

template <class T>
inline T Dot(const Vec3<T>& lhs, const Vec3<T>& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

#endif
