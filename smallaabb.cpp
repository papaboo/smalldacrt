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

    inline Type GetType() {
        return (BVHNode::Type)type;
    }

    inline unsigned int GetLeftChild() {
        return child;
    }
    inline unsigned int GetRightChild() {
        return child+1;
    }

    inline unsigned int GetFirstPrimitive() {
        return firstPrimitive;
    }
    inline unsigned char GetPrimitiveRange() {
        return primRange;
    }

    inline std::string ToString() {
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
    inline std::string ReadableType(Type t) {
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

static int levels = 0;

void CreateBVH(const AABB& parentAABB, const unsigned int nodeIndex, vector<BVHNode>& nodes, 
               std::vector<Sphere>& spheres, const std::vector<Sphere>::iterator sphereBegin, const std::vector<Sphere>::iterator sphereEnd) {

    for (int i = -1; i < levels; ++i) cout << "  ";
    cout << "Creating node " << nodeIndex << " from spheres [" << sphereBegin - spheres.begin() << ", " << sphereEnd - spheres.begin() << "]" << endl;
    
    AABB aabb = CalcAABB(sphereBegin, sphereEnd);
    aabb = Intersection(parentAABB, aabb);

    unsigned int range = sphereEnd - sphereBegin;
    if (range < 16 || levels > 3) {
        // Create leaf
        nodes[nodeIndex] = BVHNode::Leaf(aabb, spheres.begin() - sphereBegin, range);
        
        for (int i = -1; i < levels; ++i) cout << "  ";
        cout << "Leaf: " << nodes[nodeIndex].ToString() << endl;    
    } else {
        // Create nodes

        unsigned int childIndex = nodes.size();
        nodes[nodeIndex] = BVHNode::Inner(aabb, childIndex);
        for (int i = -1; i < levels; ++i) cout << "  ";
        cout << "Inner: " << nodes[nodeIndex].ToString() << endl;
        
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
            
            ++levels;
            // Create left tree;
            CreateBVH(leftAABB, childIndex, nodes, spheres, sphereBegin, spherePivot);
            CreateBVH(rightAABB, childIndex+1, nodes, spheres, spherePivot, sphereEnd);
            --levels;
        }
    }
}

int main(int argc, char *argv[]){
    int sqrtSamples = argc >= 2 ? atoi(argv[1]) : 1; // # samples
    int samples = sqrtSamples * sqrtSamples;
    
    int iterations = argc >= 3 ? atoi(argv[2]) : 1; // # iterations
    Color* cs = NULL;

    {
        AABB aabb = AABB(Vector3(), Vector3(1,2,3));
        BVHNode root = BVHNode::Inner(aabb, 0);
        BVHNode leaf = BVHNode::Leaf(aabb, 0, 4);
        
        cout << "root: " << root.ToString() << endl;
        cout << "leaf: " << leaf.ToString() << endl;
        cout << "sizeof vector: " << sizeof(Vector3) << endl;
        cout << "sizeof aabb: " << sizeof(AABB) << endl;
        cout << "sizeof node: " << sizeof(BVHNode) << endl;
        cout << "sizeof itr: " << sizeof(std::vector<Sphere>::iterator) << endl;
    }

    // AABB aabb = AABB(Vector3(1,2,3), Vector3(8,5,3));
    // AABB::Dimension dim = aabb.GetLargestDimension();

    vector<Sphere> spheres = Scenes::CornellBox();
    vector<BVHNode> nodes = vector<BVHNode>(1);
    cout << "nodes: " << nodes.size() << endl;
    AABB startAABB = AABB(Vector3(-1e30f, -1e30f, -1e30f), Vector3(1e30f, 1e30f, 1e30f));
    CreateBVH(startAABB, 0, nodes, spheres, spheres.begin(), spheres.end());
    cout << "nodes: " << nodes.size() << endl;
    cout << "root: " << nodes[0].ToString() << endl;    


    return 0;
}
