#ifndef _SMALL_AABB_H_
#define _SMALL_AABB_H_

struct AABB {
    Vector3 min;
    Vector3 max;
    
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

    inline float Intersect(const Ray& ray) const {
        Vector3 minTs = (min - ray.origin) / ray.dir;
        Vector3 maxTs = (max - ray.origin) / ray.dir;
        
        float nearT = std::min(minTs.x, maxTs.x);
        nearT = std::max(nearT, std::min(minTs.y, maxTs.y));
        nearT = std::max(nearT, std::min(minTs.z, maxTs.z));

        float farT = std::max(minTs.x, maxTs.x);
        farT = std::min(farT, std::max(minTs.y, maxTs.y));
        farT = std::min(farT, std::max(minTs.z, maxTs.z));
        
        return farT < nearT ? -1e30 : (nearT <= 0 ? farT : nearT);
    }

    inline std::string ToString() const {
        std::ostringstream out;
        out << "[min: " << min.ToString() << ", max: " << max.ToString() + "]";
        return out.str();
    }
};

#endif