// Vector

#ifndef _SMALL_SCENES_H_
#define _SMALL_SCENES_H_

#include <vector>

#include "Vector.h"
#include "Sphere.h"
#include "Utils.h"

class Scenes {
public: 
    
    static std::vector<Sphere> CornellBox() {
        //const int SPHERES = 129;
        const int SPHERES = 2048;
        std::vector<Sphere> spheres = std::vector<Sphere>(SPHERES);
        spheres[0] = Sphere(1e5, Vector3(1e5+1,40.8,81.6),   Vector3(),Vector3(.75,.25,.25),DIFFUSE); // Left
        spheres[1] = Sphere(1e5, Vector3(-1e5+99,40.8,81.6), Vector3(),Vector3(.25,.25,.75),DIFFUSE); // Right
        spheres[2] = Sphere(1e5, Vector3(50,40.8, 1e5),      Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Back
        spheres[3] = Sphere(1e5, Vector3(50,40.8,-1e5+170),  Vector3(),Vector3(),           DIFFUSE); // Front
        spheres[4] = Sphere(1e5, Vector3(50, 1e5, 81.6),     Vector3(),Vector3(.75,.75,.75),DIFFUSE); // Bottom
        spheres[5] = Sphere(1e5, Vector3(50,-1e5+81.6,81.6), Vector3(),Vector3(.75,.75,.75),DIFFUSE) ;// Top
        spheres[6] = Sphere(600, Vector3(50,681.6-.27,81.6), Vector3(12,12,12),  Vector3(), DIFFUSE); // Light
        spheres[7] = Sphere(16.5,Vector3(73,16.5,78),        Vector3(),Vector3(1,1,1)*.999, REFRACTING); // Glas
        spheres[8] = Sphere(16.5,Vector3(27,16.5,47),        Vector3(),Vector3(1,1,1)*.999, SPECULAR) ;// Mirror
        
        for (int s = 9; s < SPHERES; ++s) {
            // Create weird random spheres
            float radius = 1.0f + 2.0f * Rand01();
            Vector3 pos = Vector3(Rand01() * 100.0 , Rand01() * 100.0 , Rand01() * 100.0 + 50.0);
            Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
            float reflParam = Rand01();
            ReflectionType rt = reflParam < 0.1f ? REFRACTING : (reflParam > 0.9f ? SPECULAR : DIFFUSE);
        
            spheres[s] = Sphere(radius, pos, Vector3(0.05, 0.05, 0.05),  c, rt);
        }
        
        return spheres;
    }


    static std::vector<Sphere> Test() {
        std::vector<Sphere> spheres;
        
        const Vector3 center = Vector3(50, 40, 80);
        for (int i = 0; i < 10; ++i) {
            for (float x = -1; x < 1.5f; ++x)
                for (float y = -1; y < 1.5f; ++y)
                    for (float z = -1; z < 1.5f; ++z) {
                        Vector3 pos = Vector3(x, y, z) * 15.0f + center;
                        Color c = Color(x * 0.5f + 0.5f, y * 0.5f + 0.5f, z * 0.5f + 0.5f);
                        spheres.push_back(Sphere(5, pos, c * 0.2f + Color(0.1,0.1,0.1), c, DIFFUSE));
                    }
        }

        return spheres;
    }

    static std::vector<Sphere> Snow() {
        const int SPHERES = 8096;
        std::vector<Sphere> spheres = std::vector<Sphere>(SPHERES);
        int s = 0;
        spheres[s++] = Sphere(1e5, Vector3(50, -1e5, 81.6), Vector3(0.01,0.01,0.03),Vector3(.75,.75,.75),DIFFUSE); // Bottom        
        spheres[s++] = Sphere(30, Vector3(-65, 115, -350), Vector3(0.57,0.72,0.87),Vector3(0.0,0.0,.0),DIFFUSE); // Moon
        
        // Snowman
        spheres[s++] = Sphere(12, Vector3(50, 8, 81.6), Vector3(0,0,0), Vector3(.9,.9,.9), DIFFUSE);
        spheres[s++] = Sphere(10, Vector3(50, 28, 81.6), Vector3(0,0,0), Vector3(.9,.9,.9), DIFFUSE);
        Vector3 headCenter = Vector3(50, 43, 81.6);
        spheres[s++] = Sphere(6, headCenter, Vector3(0,0,0), Vector3(.9,.9,.9), DIFFUSE);
        // Eyes
        spheres[s++] = Sphere(0.4, Vector3(48, 45, 87), Vector3(0.5,0,0), Vector3(.0,.0,.0), SPECULAR);
        spheres[s++] = Sphere(0.4, Vector3(52, 45, 87), Vector3(0.5,0,0), Vector3(.0,.0,.0), SPECULAR);
        // Nose
        spheres[s++] = Sphere(1.1, Vector3(50, 43, 88.3), Vector3(0.0,0,0), Vector3(.9,.9,.9), DIFFUSE);
        // Mouth
        // spheres[s++] = Sphere(0.4, Vector3(50, 41, 87), Vector3(2,0,0), Vector3(0.0,.0,.0), DIFFUSE);
        
        for (; s < SPHERES; ++s) {
            // Create snow
            float radius = .05f + .05f * Rand01();
            Vector3 pos = Vector3(10.0f + Rand01() * 80.0 , Rand01() * 80.0 , Rand01() * 100.0 + 50.0);
            //Color light = Rand01() > 0.3f ? Color(0,0,0) : Color(0.5f, 0.5f, 0.5f);
            //Color light = Color(0.1, 0.1, 0.1);
            Color light = Color(0.0, 0.0, 0.0);
            //Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
            Color color = Color(0.95, 0.95, 1.0f);
            ReflectionType rt = DIFFUSE;
        
            spheres[s] = Sphere(radius, pos, light, color, rt);
        }
        
        return spheres;
    }
    
    static std::vector<Sphere> SphereBox() {
        const int SPHERES = 100;
        std::vector<Sphere> spheres = std::vector<Sphere>(SPHERES);
        for (int s = 0; s < SPHERES; ++s) {
            // Create weird random spheres
            float radius = 10.25f + 1.75f * Rand01();
            Vector3 pos = Vector3(10.0f + Rand01() * 80.0 , Rand01() * 80.0 , Rand01() * 100.0 + 50.0);
            Color light = Rand01() > 0.3f ? Color(0,0,0) : Color(0.5f, 0.5f, 0.5f);
            Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
            float reflParam = Rand01();
            ReflectionType rt = reflParam < 0.1f ? REFRACTING : (reflParam > 0.4f ? SPECULAR : DIFFUSE);
            //ReflectionType rt = DIFFUSE;
        
            spheres[s] = Sphere(radius, pos, light, c, rt);
        }
        
        return spheres;
    }


    static std::vector<Sphere> GlasBox() {
        const int SPHERES = 1000;
        std::vector<Sphere> spheres = std::vector<Sphere>(SPHERES);
        for (int s = 0; s < SPHERES; ++s) {
            // Create weird random spheres
            float radius = 1.0f + 3.0f * Rand01();
            Vector3 pos = Vector3(10.0f + Rand01() * 80.0 , Rand01() * 80.0 , Rand01() * 100.0 + 50.0);
            Color light = Color(0.1f, 0.4f, 0.1f);
            Color c = Color(Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f, Rand01() * 0.8f + 0.1f);
            // float reflParam = Rand01();
            // ReflectionType rt = reflParam < 0.1f ? REFRACTING : (reflParam > 0.4f ? SPECULAR : DIFFUSE);
            ReflectionType rt = REFRACTING;
        
            spheres[s] = Sphere(radius, pos, light, c, rt);
        }
        
        return spheres;
    }
    
};

#endif
