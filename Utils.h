#ifndef _SMALL_UTILS_H_
#define _SMALL_UTILS_H_

#include <math.h>
#include "AABPenteract.h"
#include "Cone.h"

template <class T>
inline T Clamp01(T v) { 
    return v < T(0) ? T(0) : v > T(1) ? T(1) : v;
}

inline float Rand01() {
    return (float)rand() / (float)RAND_MAX;
}

inline int ToByte(float v) {
    return int(pow(Clamp01(v),1/2.2)*255+.5);
}

#endif
