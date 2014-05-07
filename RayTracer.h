// *********************************************************
// Ray Tracer Class
// Author : Tamy Boubekeur (boubek@gmail.com).
// Copyright (C) 2012 Tamy Boubekeur.
// All rights reserved.
// *********************************************************

#ifndef RAYTRACER_H
#define RAYTRACER_H

#include <iostream>
#include <vector>
#include <QImage>

#include "Vec3D.h"
#include "Light.h"
#include "Object.h"
#include "Vertex.h"
#include "Ray.h"
#include "Scene.h"

class RayTracer {
public:
    static RayTracer * getInstance ();
    static void destroyInstance ();

    inline const Vec3Df & getBackgroundColor () const { return backgroundColor;}
    inline void setBackgroundColor (const Vec3Df & c) { backgroundColor = c; }
    
    QImage render (const Vec3Df & camPos,
                   const Vec3Df & viewDirection,
                   const Vec3Df & upVector,
                   const Vec3Df & rightVector,
                   float fieldOfView,
                   float aspectRatio,
                   unsigned int screenWidth,
                   unsigned int screenHeight);

    Vec3Df calculBRDFphong(const Light &l, const Vertex &v, const Object &o, const Vec3Df & camPos);
    float ambiantOcclusion(const Vertex &v, const Scene *scene, int nbr_rayon, float radius, float angle);
    //bool intersectScene(const Scene &scene, const Ray &ray, const Object **obj, Vertex& ver, SpaceMgr &mgr);
    bool intersectScene(const Scene &scene, const Ray &ray, const Object **obj, Vertex& ver);

    //Vec3Df pathTracing (const Ray &ray, const Scene *scene, int depth, SpaceMgr &mgr);
    Vec3Df pathTracing (const Ray &ray, const Scene *scene, int depth);

    bool intersectScene2(const Scene &scene, const Ray& ray1, const Object **obj, Vertex &ver);

    void getInterceptedTriangles (const Ray & ray,
                            std::vector<unsigned int> & objects,
                            std::vector<Triangle> & interceptedTriangles,
                            std::vector<Vec3Df> & interceptedTrianglesPoints,
                            std::vector<Vec3Df> & interceptedTrianglesBarys,
                            std::vector<float> & interceptedTrianglesDistance);

protected:
    inline RayTracer () {}
    inline virtual ~RayTracer () {}
    
private:
    Vec3Df backgroundColor;

};


#endif // RAYTRACER_H

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
