#ifndef _SMALL_AABB_H_
#define _SMALL_AABB_H_

#include <algorithm>

struct AABB {
    enum Dimension {X, Y, Z};

    Vector3 min;
    Vector3 max;

    AABB() 
        : min(Vector3(1e30f, 1e30f, 1e30f)), max(Vector3(-1e30f, -1e30f, -1e30f)) {}

    AABB(const Vector3& min, const Vector3& max) 
        : min(min), max(max) {}

    AABB(const Sphere& s)
        : min(s.position - s.radius), max(s.position + s.radius) {}

    inline void Extend(const Sphere& s) {
        min.x = std::min(min.x, s.position.x - s.radius);
        min.y = std::min(min.y, s.position.y - s.radius);
        min.z = std::min(min.z, s.position.z - s.radius);

        max.x = std::max(max.x, s.position.x + s.radius);
        max.y = std::max(max.y, s.position.y + s.radius);
        max.z = std::max(max.z, s.position.z + s.radius);
    }

    inline Vector3 Center() const {
        return (min + max) * 0.5f;
    }

    inline bool ClosestIntersection(const Ray& ray, float &tHit) const {
        Vector3 minTs = (min - ray.origin) / ray.dir;
        Vector3 maxTs = (max - ray.origin) / ray.dir;
        
        float nearT = std::min(minTs.x, maxTs.x);
        nearT = std::max(nearT, std::min(minTs.y, maxTs.y));
        nearT = std::max(nearT, std::min(minTs.z, maxTs.z));
        
        float farT = std::max(minTs.x, maxTs.x);
        farT = std::min(farT, std::max(minTs.y, maxTs.y));
        farT = std::min(farT, std::max(minTs.z, maxTs.z));
     
        tHit = nearT;

        return nearT <= farT && 0 < farT;
    }

    inline Vector3 ClosestPointOnSurface(const Vector3& p) const {
        return Vector3(std::min(max.x, std::max(min.x, p.x)),
                       std::min(max.y, std::max(min.y, p.y)),
                       std::min(max.z, std::max(min.z, p.z)));
    }

    inline float DistanceTo(const Vector3& p) const {
        Vector3 pSurface = ClosestPointOnSurface(p);
        return (pSurface - p).Length();
    }

    inline Dimension GetLargestDimension() {
        Vector3 size = max - min;
        return (size.x > size.y && size.x > size.z) ? X : (size.y > size.z ? Y : Z);
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[min: " << min.ToString() << ", max: " << max.ToString() + "]";
        return out.str();
    }
};

inline AABB Intersection(const AABB& lhs, const AABB& rhs) {
    return AABB(Vector3(std::max(lhs.min.x, rhs.min.x),
                        std::max(lhs.min.y, rhs.min.y),
                        std::max(lhs.min.z, rhs.min.z)),
                Vector3(std::min(lhs.max.x, rhs.max.x),
                        std::min(lhs.max.y, rhs.max.y),
                        std::min(lhs.max.z, rhs.max.z)));
}
inline AABB Union(const AABB& lhs, const AABB& rhs) {
    return AABB(Vector3(std::min(lhs.min.x, rhs.min.x),
                        std::min(lhs.min.y, rhs.min.y),
                        std::min(lhs.min.z, rhs.min.z)),
                Vector3(std::max(lhs.max.x, rhs.max.x),
                        std::max(lhs.max.y, rhs.max.y),
                        std::max(lhs.max.z, rhs.max.z)));
}

#endif
