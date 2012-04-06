#ifndef _SMALL_MATH_H_
#define _SMALL_MATH_H_

template <class T>
inline T Clamp01(T v) { 
    return v < T(0) ? T(0) : v > T(1) ? T(1) : v;
}

template <class T>
T Lerp(const T& from, const T& to, const float t) {
    return (1.0f - t) * from + t * to;
}

inline float Rand01() {
    return (float)rand() / (float)RAND_MAX;
}

#endif
