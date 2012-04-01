// smallaabb. A rayTracer using an aabb hierarchy with an implementation based
// on smallpt, a path tracer by Kevin Beason. (But written slightly more human
// readable)

// Compile ./make smallaabb
// Usage: ./small 4 16 && xv image.ppm

#include <math.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>

#include <vector>
#include <algorithm>

using std::vector;
using std::cout;
using std::endl;

#include "Vector.h"
#include "Ray.h"
#include "Sphere.h"
#include "AABB.h"
#include "Utils.h"
#include "Scenes.h"

//const int WIDTH = 32, HEIGHT = 32;
const int WIDTH = 512, HEIGHT = 512;
int sqrtSamples;
int samples;

inline int Index(const int x, const int y, const int sub) {
    return (x + y * WIDTH) * samples + sub;
}
inline int Index(const int x, const int y, const int subX, const int subY) {
    return (x + y * WIDTH) * samples + subX + subY * sqrtSamples;
}

struct BVHNode {
    enum Type { INNER = 0, LEAF = 1, DUMMY = 2};

    // unsigned int child;
    union {
        unsigned int child;
        unsigned int firstPrimitive;
    };
    AABB aabb;
    unsigned char primRange;
    unsigned char type;
    char pad[2];
    
    inline static BVHNode Inner(const AABB& aabb, const unsigned int leftChild) {
        BVHNode node;
        node.aabb = aabb;
        node.child = leftChild;
        node.type = INNER;
        return node;
    }

    inline static BVHNode Leaf(const AABB& aabb, const unsigned int primitive, const unsigned char range) {
        BVHNode node;
        node.aabb = aabb;
        node.firstPrimitive = primitive;
        node.type = LEAF;
        node.primRange = range;
        return node;
    }

    inline static BVHNode Dummy() {
        BVHNode node;
        node.type = DUMMY;
        return node;
    }

    inline Type GetType() const {
        return (BVHNode::Type)type;
    }

    inline unsigned int GetLeftChild() const {
        return child;
    }
    inline unsigned int GetRightChild() const {
        return child+1;
    }

    inline unsigned int GetFirstPrimitive() const {
        return firstPrimitive;
    }
    inline unsigned char GetPrimitiveRange() const {
        return primRange;
    }

    inline std::string ToString() const {
        std::ostringstream out;
        switch (GetType()){
        case INNER:
            out << "[Type: " << ReadableType(GetType()) << ", AABB: " << aabb.ToString();
            out  << ", children: [" << GetLeftChild() << ", " << GetRightChild() << "]";
            break;
        case LEAF:
            out << "[Type: " << ReadableType(GetType()) << ", AABB: " << aabb.ToString();
            out  << ", primitives: [" << GetFirstPrimitive() << " -> " << (unsigned int)GetPrimitiveRange() << "]";;
            break;
        case DUMMY:
            out << "DUMMY";
        }
        return out.str();
    }

private:
    inline std::string ReadableType(Type t) const {
        switch (t) {
        case INNER:
            return "Inner";
        case LEAF:
            return "Leaf";
        case DUMMY:
            return "Dummy";
        }
    }
};

struct PartitionSpheresByX {
    float x;
    PartitionSpheresByX(const float x) : x(x) {}
    bool operator()(Sphere s) { return s.position.x < x; }
};

struct PartitionSpheresByY {
    float y;
    PartitionSpheresByY(const float y) : y(y) {}
    bool operator()(Sphere s) { return s.position.y < y; }
};

struct PartitionSpheresByZ {
    float z;
    PartitionSpheresByZ(const float z) : z(z) {}
    bool operator()(Sphere s) { return s.position.z < z; }
};

void CreateBVH(const AABB& parentAABB, const unsigned int nodeIndex, vector<BVHNode>& nodes, 
               std::vector<Sphere>& spheres, const std::vector<Sphere>::iterator sphereBegin, const std::vector<Sphere>::iterator sphereEnd) {
    
    AABB nodeAABB = CalcAABB(sphereBegin, sphereEnd);
    AABB aabb = Intersection(parentAABB, nodeAABB);

    unsigned int range = sphereEnd - sphereBegin;
    if (range < 6)
        // Create leaf
        nodes[nodeIndex] = BVHNode::Leaf(nodeAABB, sphereBegin - spheres.begin(), range);
    else {
        // Create nodes

        unsigned int childIndex = nodes.size();
        nodes[nodeIndex] = BVHNode::Inner(nodeAABB, childIndex);
        
        // Find splitting plane
        AABB::Dimension largestDim = aabb.GetLargestDimension();

        std::vector<Sphere>::iterator spherePivot;
        AABB leftAABB = aabb, rightAABB = aabb;
        switch(largestDim) {
        case AABB::X:
            leftAABB.max.x = rightAABB.min.x = (aabb.max.x + aabb.min.x) * 0.5f;
            spherePivot = std::partition(sphereBegin, sphereEnd, 
                                         PartitionSpheresByX(leftAABB.max.x));
            break;
        case AABB::Y:
            leftAABB.max.y = rightAABB.min.y = (aabb.max.y + aabb.min.y) * 0.5f;
            spherePivot = std::partition(sphereBegin, sphereEnd, 
                                         PartitionSpheresByY(leftAABB.max.y));
            break;
        case AABB::Z:
            leftAABB.max.z = rightAABB.min.z = (aabb.max.z + aabb.min.z) * 0.5f;
            spherePivot = std::partition(sphereBegin, sphereEnd, 
                                         PartitionSpheresByZ(leftAABB.max.z));
            break;
        }

        if (spherePivot == sphereEnd) 
            // All spheres where partitioned to the left side. Have another go
            // at partitioning with the smaller aabb.
            CreateBVH(leftAABB, nodeIndex, nodes, spheres, sphereBegin, sphereEnd);
        else if (spherePivot == sphereBegin) 
            // All spheres where partitioned to the right side.
            CreateBVH(rightAABB, nodeIndex, nodes, spheres, sphereBegin, sphereEnd);
        else {
            // Reserve room for children
            nodes.push_back(BVHNode::Dummy());
            nodes.push_back(BVHNode::Dummy());
            
            // Create left tree;
            CreateBVH(leftAABB, childIndex, nodes, spheres, sphereBegin, spherePivot);
            CreateBVH(rightAABB, childIndex+1, nodes, spheres, spherePivot, sphereEnd);
        }
    }
}

void PrintHierarchy(const vector<BVHNode>& nodes, const int id = 0, const int level = 0) {
    for (int i = 0; i < level; ++i) cout << "  ";
    cout << nodes[id].ToString() << endl;
    if (nodes[id].GetType() == BVHNode::INNER) {
        PrintHierarchy(nodes, nodes[id].GetLeftChild(), level+1);
        PrintHierarchy(nodes, nodes[id].GetRightChild(), level+1);
    }
}

vector<Ray> CreateRays() {
    Ray cam(Vector3(50,52,295.6), Vector3(0,-0.042612,-1).Normalize()); // cam pos, dir
    Vector3 cx = Vector3(WIDTH * 0.5135 / HEIGHT, 0, 0);
    Vector3 cy = (cx.Cross(cam.dir)).Normalize() * 0.5135;

    vector<Ray> rays = vector<Ray>(WIDTH * HEIGHT * samples);
    for (int y = 0; y < HEIGHT; y++){
        unsigned short Xi[3] = {0, 0, y*y*y};
        for (unsigned short x = 0; x < WIDTH; x++) {
            
            // subpixel grid
            for (int subY = 0; subY < sqrtSamples; ++subY)
                for (int subX = 0; subX < sqrtSamples; ++subX) {
                    // Samples
                    double r1 = 2 * erand48(Xi);
                    float dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                    double r2 = 2 * erand48(Xi);
                    float dy = r2 < 1 ? sqrt(r2) - 1: 1 - sqrt(2 - r2);
                    
                    Vector3 rayDir = cx * (((subX + 0.5 + dx) / sqrtSamples + x) / WIDTH - 0.5) 
                        + cy * (((subY + 0.5 + dy) / sqrtSamples + y) / HEIGHT - 0.5) + cam.dir;
                    rays[Index(x,y,subX,subY)] = Ray(cam.origin + rayDir * 140, rayDir.Normalize());
                }
        }
    }
    
    return rays;
}

inline float Exhaustive(const Ray charles, float t, const BVHNode& node, 
                        const vector<Sphere> spheres, unsigned int &sphereId) {

    for (unsigned int p = node.GetFirstPrimitive(); 
         p < node.GetFirstPrimitive() + node.GetPrimitiveRange(); ++p) {

        const Sphere sphere = spheres[p];
        const float tSphere = sphere.Intersect(charles);
        if (0 < tSphere && tSphere < t) {
            sphereId = p;
            t = tSphere;
        }
    }

    return t;
}

static int levels = 0;

    
/**
 * Recursively intersects Ray Charles with his node and returns the distance to
 * the closest intersection and a stores the id of the sphere in sphereId.
 */
inline float Intersect(const Ray charles, float t, 
                       const BVHNode& node, const vector<BVHNode>& nodes, 
                       const vector<Sphere> spheres, unsigned int &sphereId) {

    // ++levels;
    // for (int i = 0; i < levels; ++i) cout << "  ";
    // cout << "Intersect: [t: " << t << ", n: " << node.ToString() << "]" << endl;

    if (node.GetType() == BVHNode::LEAF) {
        // Intersect leaf
        // --levels;
        return Exhaustive(charles, t, node, spheres, sphereId);
    } else {
        // Traverse further
        const BVHNode left = nodes[node.GetLeftChild()];
        float tLeft;
        if (!left.aabb.ClosestIntersection(charles, tLeft)) tLeft = 1e32f;
        
        const BVHNode right = nodes[node.GetRightChild()];
        float tRight;
        if (!right.aabb.ClosestIntersection(charles, tRight)) tRight = 1e32f;
    
        if (tLeft < tRight) { // Intersect left first
            // for (int i = 0; i < levels; ++i) cout << "  ";
            // cout << " +->[tLeft: " << tLeft << " -> " << left.aabb.ToString() << endl;
            // for (int i = 0; i < levels; ++i) cout << "  ";
            // cout << " +->[tRight: " << tRight << " -> " << right.aabb.ToString() << endl;

            if (tLeft < t) t = Intersect(charles, t, left, nodes, spheres, sphereId);
            if (tRight < t) t = Intersect(charles, t, right, nodes, spheres, sphereId);
        } else { // Intersect right first
            // for (int i = 0; i < levels; ++i) cout << "  ";
            // cout << " +->[tRight: " << tRight << " -> " << right.aabb.ToString() << endl;
            // for (int i = 0; i < levels; ++i) cout << "  ";
            // cout << " +->[tLeft: " << tLeft << " -> " << left.aabb.ToString() << endl;

            if (tRight < t) t = Intersect(charles, t, right, nodes, spheres, sphereId);
            if (tLeft < t) t = Intersect(charles, t, left, nodes, spheres, sphereId);
        }

        // --levels;        
        return t;
    }
}

inline float Intersect(const Ray charles, const vector<BVHNode>& nodes, 
                      const vector<Sphere> spheres, unsigned int &sphereId) {
    sphereId = -1;
    float t = Intersect(charles, 1e30, nodes[0], nodes, spheres, sphereId);
    return sphereId == -1 ? -1.0f : t;
}

Color Shade(const Ray ray, const int depth, const vector<BVHNode>& nodes, const vector<Sphere>& spheres) {
    // id of intersected object
    unsigned int sphereId = 0;

    // cout << "Shade: " << ray.ToString() << endl;
    const float t = Intersect(ray, nodes, spheres, sphereId);
    // cout << endl;
    if (t <= 0.0f)
        return Color(0,0,0); // Background color

    const Sphere& sphere = spheres[sphereId];

    const Vector3 hitPos = ray.origin + ray.dir * t;
    const Vector3 norm = (hitPos - sphere.position).Normalize();
    const Vector3 nl = Dot(norm, ray.dir) < 0 ? norm : norm * -1;

    Color f = sphere.color;
    const float maxRefl = f.x>f.y && f.x>f.z ? f.x : f.y>f.z ? f.y : f.z;
    if (depth > 1) // if depth above 5, then terminate
        return sphere.emission;

    // All objects are diffuse
    const float r1 = 2 * M_PI * Rand01();
    const float r2 = Rand01(); 
    const float r2s = sqrtf(r2);
    // Normal space
    const Vector3 w = nl; 
    const Vector3 u = ((fabsf(w.x) > 0.1f ? Vector3(0,1,0) : Vector3(1,0,0)).Cross(w)).Normalize();
    const Vector3 v = w.Cross(u);
    const Vector3 newRayDir = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrtf(1-r2)).Normalize();
    const Vector3 newPos = hitPos + nl * 0.02f;
    return sphere.emission + f * Shade(Ray(newPos, newRayDir), depth+1, nodes, spheres);
}

int main(int argc, char *argv[]){
    sqrtSamples = argc >= 2 ? atoi(argv[1]) : 1; // # samples
    samples = sqrtSamples * sqrtSamples;
    
    int iterations = argc >= 3 ? atoi(argv[2]) : 1; // # iterations

    vector<Sphere> spheres = Scenes::CornellBox();
    vector<Ray> rays = CreateRays();
    
    vector<BVHNode> nodes = vector<BVHNode>(1);
    AABB startAABB = AABB(Vector3(-1e30f, -1e30f, -1e30f), Vector3(1e30f, 1e30f, 1e30f));
    CreateBVH(startAABB, 0, nodes, spheres, spheres.begin(), spheres.end());
    

    cout << " === Hierarchy ===" << endl;
    PrintHierarchy(nodes);
    cout << endl;
    cout << endl;

    Color* frags = new Color[rays.size()];
    for (int r = 0; r < rays.size(); ++r) {
        if ((r % 1024) == 0) fprintf(stderr,"\rRendering %i/%lu", r, rays.size());
        frags[r] = Shade(rays[r], 0, nodes, spheres);
    }
    cout << endl;
    
    // ASCII color art
    // for (int y = 0; y < HEIGHT; ++y) {
    //     for (int x = 0; x < WIDTH; ++x)
    //         cout << sphereIds[Index(x,y,0)] << " ";
    //     cout << endl;
    // }

    Color* cs = new Color[WIDTH * HEIGHT];
    for (int x = 0; x < WIDTH; ++x)
        for (int y = 0; y < HEIGHT; ++y) {
            Color c = Color(0,0,0);
            for (int s = 0; s < samples; ++s)
                c += frags[Index(x,y,s)];
            cs[x + y * WIDTH] = c / samples;
        }

    SavePPM("aabbimage.ppm", WIDTH, HEIGHT, cs);
    
    return 0;
}
