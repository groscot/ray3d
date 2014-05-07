// *********************************************************
// Ray Tracer Class
// Author : Tamy Boubekeur (boubek@gmail.com).
// Copyright (C) 2012 Tamy Boubekeur.
// All rights reserved.
// *********************************************************

#include "RayTracer.h"
#include "Ray.h"
#include "Scene.h"
#include <QProgressDialog>
#include <iostream>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <stack>

using namespace std;
#define PI 3.1415926

static RayTracer * instance = NULL;



RayTracer * RayTracer::getInstance () {
    if (instance == NULL)
        instance = new RayTracer ();
    return instance;
}

void RayTracer::destroyInstance () {
    if (instance != NULL) {
        delete instance;
        instance = NULL;
    }
}

inline int clamp (float f, int inf, int sup) {
    int v = static_cast<int> (f);
    return (v < inf ? inf : (v > sup ? sup : v));
}


void RayTracer::getInterceptedTriangles (const Ray & ray2,
                                   std::vector<unsigned int> & objects,
                                   std::vector<Triangle> & interceptedTriangles,
                                   std::vector<Vec3Df> & interceptedTrianglesPoints,
                                   std::vector<Vec3Df> & interceptedTrianglesBarys,
                                   std::vector<float> & interceptedTrianglesDistance) {
    Scene * scene = Scene::getInstance ();
    //vector<unsigned int> triangles = vector<unsigned int>();
    int totalTriangles = 0;

    for (unsigned int k = 0; k < scene->getObjects().size (); k++) {
        const Object & o = scene->getObjects()[k];
        Mesh m = o.getMesh();
        vector<Vertex> vertices = m.getVertices();
        const Ray ray(ray2.getOrigin()-o.getTrans (), ray2.getDirection());

        /** Strategie avec le KDTree :
         * stocker une pile de pointeurs à examiner (en priorité, l'arbre le plus proche du rayon)
         * commencer par empiler le kdtree le plus gros (l'objet)
         * répéter :
         *		dépiler la premier élément
         *		si c'est une feuille, chercher des triangles interceptés
         *			si on en trouve, fini
         *		sinon, calculer la distance des fils gauche/droit
         *			empiler d'abord le plus loin, puis le plus proche
         * tant qu'on n'a pas trouvé de triangle intercepté */

        bool foundTriangles = false;
        std::stack<KDTree*> treeStack;
        treeStack.push(o.getKDTree());
        while (!treeStack.empty() && !foundTriangles)
        {
            // dépiler
            KDTree * tree = treeStack.top();
            treeStack.pop();
            if (tree->isLeaf()) {
                // chercher les triangles interceptés
                vector<unsigned int> inner = tree->getInnerTriangles();
                for (unsigned int i = 0; i < inner.size(); i++) {
                    const Triangle & t = m.getTriangle(inner[i]);

                    const Vec3Df & v0 = vertices[t.getVertex(0)].getPos();
                    const Vec3Df & v1 = vertices[t.getVertex(1)].getPos();
                    const Vec3Df & v2 = vertices[t.getVertex(2)].getPos();

                    Vec3Df intersectionPoint, bary;
                    float intersection = ray.intersect(v0, v1, v2, intersectionPoint, bary);
                    //dzh add
                    intersectionPoint = intersectionPoint + o.getTrans();
                    if (intersection >= 0) {
                        foundTriangles = true;
                        totalTriangles++;
                        interceptedTriangles.push_back(t);
                        objects.push_back(k);
                        interceptedTrianglesPoints.push_back(intersectionPoint);
                        interceptedTrianglesBarys.push_back(bary);
                        interceptedTrianglesDistance.push_back(intersection);
                    }
                }
            } else {
                KDTree * leftTree = tree->getLeft();
                KDTree * rightTree = tree->getRight();
                Vec3Df inter1, inter2;
                bool interLeft = ray.intersect(leftTree->getBbox(), inter1);
                bool interRight = ray.intersect(rightTree->getBbox(), inter2);
                // le plus proche ?
                bool rightFirst = Vec3Df::squaredDistance(inter1, ray.getOrigin()) < Vec3Df::squaredDistance(inter2, ray.getOrigin());
                if(rightFirst) {
                    if (interRight) treeStack.push(rightTree);
                    if (interLeft) treeStack.push(leftTree);
                } else {
                    if (interLeft) treeStack.push(leftTree);
                    if (interRight) treeStack.push(rightTree);
                }

            }
        }
    }
}


// POINT D'ENTREE DU PROJET.
// Le code suivant ray trace uniquement la boite englobante de la scene.
// Il faut remplacer ce code par une veritable raytracer
QImage RayTracer::render (const Vec3Df & camPos,
                          const Vec3Df & direction,
                          const Vec3Df & upVector,
                          const Vec3Df & rightVector,
                          float fieldOfView,
                          float aspectRatio,
                          unsigned int screenWidth,
                          unsigned int screenHeight) {
    QImage image (QSize (screenWidth, screenHeight), QImage::Format_RGB888);
    Scene * scene = Scene::getInstance ();
    const BoundingBox & bbox = scene->getBoundingBox ();
    const Vec3Df & minBb = bbox.getMin ();
    const Vec3Df & maxBb = bbox.getMax ();
    const Vec3Df rangeBb = maxBb - minBb;
    QProgressDialog progressDialog ("Raytracing...", "Cancel", 0, 100);
    progressDialog.show ();
    cout << "camPos" << camPos << endl;
    //const float epsilonDistance = 10; //à vérifier

//    // pour tester
//    float tanX = tan (fieldOfView)*aspectRatio;
//    float tanY = tan (fieldOfView);
//    Vec3Df stepX = (500 - screenWidth/2.f)/screenWidth * tanX * rightVector;
//    Vec3Df stepY = (600 - screenHeight/2.f)/screenHeight * tanY * upVector;
//    Vec3Df step = stepX + stepY;
//    Vec3Df dir = direction + step;
//    dir.normalize ();
//    Ray ray (camPos, dir);
//    Vertex vintersect;
//    const Object *oIntersect = NULL; //the object of the nearest intersection point
//    bool isIntersect= false;
//    isIntersect = intersectScene(*scene,ray,&oIntersect,vintersect);

//    Vertex vintersect2;
//    const Object *oIntersect2 = NULL; //the object of the nearest intersection point
//    bool isIntersect2= false;
//    isIntersect2 = intersectScene2(*scene,ray,&oIntersect2,vintersect2);

//    cout << vintersect.getPos() << ";" << vintersect2.getPos() << endl;


    for (unsigned int i = 0; i < screenWidth; i++) {
        progressDialog.setValue ((100*i)/screenWidth);
        for (unsigned int j = 0; j < screenHeight; j++) {

            //define points in a pixel for antialiasing
            vector< pair<float,float> > ppixels;
            ppixels.push_back(make_pair<float,float>(i-.25,j-.25));
            ppixels.push_back(make_pair<float,float>(i-.25,j));
            ppixels.push_back(make_pair<float,float>(i-.25,j+.25));
            ppixels.push_back(make_pair<float,float>(i,j-.25));
            ppixels.push_back(make_pair<float,float>(i,j));
            ppixels.push_back(make_pair<float,float>(i,j+.25));
            ppixels.push_back(make_pair<float,float>(i+.25,j-.25));
            ppixels.push_back(make_pair<float,float>(i+.25,j));
            ppixels.push_back(make_pair<float,float>(i+.25,j+.25));

            float tanX = tan (fieldOfView)*aspectRatio;
            float tanY = tan (fieldOfView);
            Vec3Df c (backgroundColor);
            //iterate 3*3 rays
            for (vector< pair<float,float> >::iterator pit = ppixels.begin(); pit != ppixels.end(); pit++) {
                Vec3Df stepX = ((*pit).first - screenWidth/2.f)/screenWidth * tanX * rightVector;
                Vec3Df stepY = ((*pit).second - screenHeight/2.f)/screenHeight * tanY * upVector;
                Vec3Df step = stepX + stepY;
                Vec3Df dir = direction + step;
                dir.normalize ();
                Ray ray (camPos, dir);
                //précalculer AO
                const BoundingBox & sceneBBox = scene->getBoundingBox ();
                float radius = 0.05 * sceneBBox.getRadius();
                Vertex vintersect;
                const Object *oIntersect = NULL; //the object of the nearest intersection point
                bool isIntersect= false;
                isIntersect = intersectScene(*scene,ray,&oIntersect,vintersect);
                float termAO = 1;
                if (isIntersect){
                    termAO =  ambiantOcclusion(vintersect,scene,8,radius,PI);
//                    //only AO
//                    //c += termAO*oIntersect->getMaterial().getColor()*255.f;
                    //add by dzh mirror effect
//                    if (oIntersect->getMaterial().getMirror()){
//                        //attention when calculate this direction : i am blocked because of this, i got an inversed direction
//                        Vec3Df dirReflect = -2 * (Vec3Df::dotProduct(vintersect.getNormal(), ray.getDirection())) * vintersect.getNormal() + ray.getDirection();
//                        dirReflect.normalize();
//                        Ray rayReflect(vintersect.getPos(),dirReflect);
//                        //verifier l'intersection du rayReflect avec la scene
//                        const Object *mirrorObject = NULL;
//                        Vertex mirrorVertex;
//                        Vec3Df colorMi(0.f,0.f,0.f);
//                        //parcourir tous les triangles sur tous les objets, obtenir le point intersect
//                        if(intersectScene(*scene,rayReflect,&mirrorObject,mirrorVertex))
//                            colorMi=mirrorObject->getMaterial().getColor();
//                        c += colorMi * 127.f;
//                    }
                 }
                c += termAO*pathTracing(ray,scene,1)*255.f;

            }

            c/=ppixels.size();
            image.setPixel (i, j, qRgb (clamp (c[0], 0, 255), clamp (c[1], 0, 255), clamp (c[2], 0, 255)));
        }
    }

    progressDialog.setValue (100);
    return image;
}

//avec KDTree
//obj --> nearest object which intersects with the scene
// ver -- > points of intersection
bool RayTracer::intersectScene(const Scene &scene, const Ray& ray1, const Object **obj, Vertex &ver){
    std::vector<Triangle> interceptedTriangles;
    std::vector<Vec3Df> interceptedTrianglesPoints;
    std::vector<Vec3Df> interceptedTrianglesBarys;
    std::vector<float> interceptedTrianglesDistance;
    std::vector<unsigned int> objects;
    //Vec3Df intersectionPoint;
    float smallestIntersectionDistance = 1000000.f;
    getInterceptedTriangles (ray1, objects, interceptedTriangles, interceptedTrianglesPoints, interceptedTrianglesBarys, interceptedTrianglesDistance);
    if(interceptedTriangles.size()==0) return false;
    for (unsigned int a = 0; a < interceptedTriangles.size(); a++) {
                    const Object & o = scene.getObjects()[objects[a]];
                    //material = o.getMaterial();
                    Ray ray (ray1.getOrigin()-o.getTrans (), ray1.getDirection());
                    vector<Vertex> vertices = o.getMesh().getVertices();
                    const Triangle & t = interceptedTriangles[a];
                    Vec3Df bary = interceptedTrianglesBarys[a];
                    Vec3Df intersectionPoint = interceptedTrianglesPoints[a];
                    float intersectionDistance = interceptedTrianglesDistance[a]*ray.getDirection().getLength();
                    if (intersectionDistance < smallestIntersectionDistance){
                        // Surface normal
                        Vec3Df normal0 = vertices[t.getVertex(0)].getNormal();
                        Vec3Df normal1 = vertices[t.getVertex(1)].getNormal();
                        Vec3Df normal2 = vertices[t.getVertex(2)].getNormal();
                        Vec3Df normal = Vec3Df::interpolateNormals(normal0, normal1, normal2, bary);
                        normal.normalize();
                        ver = Vertex (intersectionPoint, normal);
                        *obj=&o;
                    }

        }
    return true;
}

//SANS KDTree
 bool RayTracer::intersectScene2(const Scene &scene, const Ray& ray, const Object **obj, Vertex &ver){
    bool isIntersect = false;
    Vec3Df intersectionPoint;
    float smallestIntersectionDistance = 1000000.f;
    for (unsigned int k = 0; k < scene.getObjects().size (); k++) {
        const Object & o = scene.getObjects()[k];
        Ray ray1(ray.getOrigin() - o.getTrans(), ray.getDirection());
        // Iteration sur tous les triangles du Mesh
        Mesh m = o.getMesh();
        vector<Triangle> triangles = m.getTriangles();
        vector<Vertex> vertices = m.getVertices();

        for (unsigned int i = 0; i < triangles.size(); i++) {
            Triangle t = triangles[i];
            float intersection = ray1.intersect(vertices[t.getVertex(0)].getPos(), vertices[t.getVertex(1)].getPos(), vertices[t.getVertex(2)].getPos(), intersectionPoint);
            if (intersection >= 0) {
                isIntersect = true;
                float intersectionDistance = intersection*ray.getDirection().getLength();
                if (intersectionDistance < smallestIntersectionDistance) {
                    //calcul normal of intersectionPoint by interpolation of barycentre coordinate
                    Vec3Df np = intersectionPoint[0]*vertices[t.getVertex(0)].getNormal() + intersectionPoint[1]*vertices[t.getVertex(1)].getNormal() + intersectionPoint[2]*vertices[t.getVertex(2)].getNormal();
                    np.normalize();
                    //calcul coordonnees of intersectionPoint
                    // remember to add o.getTrans to get real coordinate
                    Vec3Df p1 = o.getTrans() + intersectionPoint[0]*vertices[t.getVertex(0)].getPos() + intersectionPoint[1]*vertices[t.getVertex(1)].getPos() + intersectionPoint[2]*vertices[t.getVertex(2)].getPos();
                    ver = Vertex (p1, np);
                    *obj = &o;
                    smallestIntersectionDistance = intersectionDistance;
                }
            }
        }
    }
    return isIntersect;

}


// vertex v is the world coordinate
Vec3Df RayTracer::calculBRDFphong(const Light &l, const Vertex &v, const Object &o, const Vec3Df &camPos){
    const Vec3Df lpos = l.getPos();
    const Material m = o.getMaterial();
    float kd = m.getDiffuse();
    float ks = m.getSpecular();
    Vec3Df wi, r, wo;
    wi = lpos - v.getPos();
    wi.normalize();
    //wo = camPos - o.getTrans() - v.getPos();
    wo = camPos - v.getPos();
    wo.normalize();
    r = 2 * (Vec3Df::dotProduct(v.getNormal(), wi)) * v.getNormal() - wi;
    r.normalize();
    float vald = max(Vec3Df::dotProduct(v.getNormal(), wi),0.f);
    float vals = max(Vec3Df::dotProduct(r, wo), 0.f);
    float shininess=20;
    float f = kd*vald + ks*pow(vals,shininess);
    return f*o.getMaterial().getColor()*l.getColor();
}

//path tracing
Vec3Df RayTracer::pathTracing (const Ray &ray, const Scene *scene, int depth){
    const float epsilonDistance = 20; //à vérifier
    Vec3Df color (0.f,0.f,0.f);
    Vertex vintersect;
    const Object *oIntersect = NULL; //the object of the nearest intersection point
    bool isIntersect= false;
    isIntersect = intersectScene(*scene,ray,&oIntersect,vintersect);
    if (isIntersect){
//        //add by dzh mirror effect avant path tracing
//        if (oIntersect->getMaterial().getMirror()){
//            //attention when calculate this direction : i am blocked because of this, i got an inversed direction
//            Vec3Df dirReflect = -2 * (Vec3Df::dotProduct(vintersect.getNormal(), ray.getDirection())) * vintersect.getNormal() + ray.getDirection();
//            dirReflect.normalize();
//            Ray rayReflect(vintersect.getPos(),dirReflect);
//            //verifier l'intersection du rayReflect avec la scene
//            const Object *mirrorObject = NULL;
//            Vertex mirrorVertex;
//            Vec3Df colorMi(0.f,0.f,0.f);
//            //parcourir tous les triangles sur tous les objets, obtenir le point intersect
//            if(intersectScene(*scene,rayReflect,&mirrorObject,mirrorVertex))
//                colorMi=mirrorObject->getMaterial().getColor();
//            color += colorMi * 255.f;
//        }



        for (unsigned int k1 = 0; k1 < scene->getLights().size (); k1++){
            const Light &l = scene->getLights()[k1];
            //area lighting, 9 points in a disk
            Vec3Df lpoint = l.getPos();
            vector<Vec3Df> lpoints;
            lpoints.push_back(lpoint);

            //radius de disque to change
            const BoundingBox & sceneBBox = scene->getBoundingBox ();
            float r = 0.2 * sceneBBox.getRadius();
            //float r = l.getRadius();

            //iterate for every shawdow ray ---> change numbre src
            for (int i_src = 0; i_src < 8; i_src++){
                float rr = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * r * r;
                float angle = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * PI * 2;
                float xr = sqrt(rr) * cos(angle);
                float yr = sqrt(rr) * sin(angle);
                lpoints.push_back(Vec3Df(lpoint+Vec3Df(xr,yr,0)));
            }
            float proportion = 0.f;
            for (vector<Vec3Df>::iterator pid = lpoints.begin(); pid!=lpoints.end(); pid++ ){
                Vec3Df dirRayShadow = (*pid) - vintersect.getPos();
                dirRayShadow.normalize();
                Ray rayShadow(vintersect.getPos(),dirRayShadow);
                //verifier si rayShadow intersecte d'autre triangles
                const Object * obj1 = NULL;
                Vertex ver1;
                if(!intersectScene(*scene,rayShadow,&obj1,ver1) || Vec3Df::squaredDistance(ver1.getPos(),rayShadow.getOrigin())>epsilonDistance ){
                    proportion+=1.f/(float)lpoints.size();
                }
            }
            color += proportion*calculBRDFphong(l, vintersect, (*oIntersect), ray.getOrigin());
        }
        color /= scene->getLights().size ();

        if (0 != depth) {
            --depth;
            Vec3Df dir(0.f,0.f,0.f);
            //--------------------------------effet mirror
            if (oIntersect->getMaterial().getMirror()){
                dir = -2 * (Vec3Df::dotProduct(vintersect.getNormal(), ray.getDirection())) * vintersect.getNormal() + ray.getDirection();
                dir.normalize();
            }
            else{
                //choisir aleatoirement un rayon
                float u1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) ;
                float u2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
                float phi = 2 * PI * u1;
                float theta = acos (2 * u2 -1);
                dir = Vec3Df (sin(theta)*cos(phi),sin(theta)*sin(phi), cos(theta));
                if (Vec3Df::dotProduct(dir,vintersect.getNormal()) < 0) dir = -dir;

                //cosine distribution around the normal
                //The sum of the given direction v and a vector uniformly distributed over a sphere leads to a cosine distribution.
                // dir = dir + vintersect.getNormal();
            }
            Ray nextRay(vintersect.getPos(),dir);
            Vec3Df colorL = pathTracing(nextRay,scene,depth);
            Light l;
            l.setPos(vintersect.getPos()+dir);
            l.setColor(colorL);
            color += calculBRDFphong(l,vintersect,*oIntersect,ray.getOrigin());
        }
    }
    return color;
}


// angle : angle du cone de distribution des rayons
float RayTracer::ambiantOcclusion(const Vertex &v, const Scene *scene, int nbr_rayon, float radius, float angle){
    //vector < Vec3Df > rays;
    float ao = 0;
    Vec3Df normal = v.getNormal();
    Vec3Df p = v.getPos();
    //float theta_normal = atan(normal[2]);
    for (int i = 0; i < nbr_rayon; ++i){
        //float theta = theta_normal + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * angle;
        //float phi = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * PI * 2;
        //Vec3Df dir (sin(theta)*cos(phi),sin(theta)*sin(phi), cos(theta));
        float u1 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) ;
        float u2 = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        float phi = 2 * PI * u1;
        float theta = acos (2 * u2 -1);
        Vec3Df dir = Vec3Df (sin(theta)*cos(phi),sin(theta)*sin(phi), cos(theta));
        if (Vec3Df::dotProduct(dir,v.getNormal()) < 0) dir = -dir;
        Ray ray(p,dir);
        /*
         *if ray intersect with any triangle in a sphere of (p, radius), then d = 0; else, d = 1;
         * a method to implement :
         * for  every object, do intersect with bbox
         *      if (intersect and distance < radius)
         *          if(       ray intersect with every triangle of this object )
         *                 if (intersectDistance < radius) :  d =0 and end the loop
         *
         */
        int d = 1;



        ///////////////////methode avant KDTree
//        Vec3Df intersectionPoint;
//        for (unsigned int k = 0; k < scene->getObjects().size (); k++) {
//            const Object & o = scene->getObjects()[k];
//            bool hasIntersection = ray.intersect (o.getBoundingBox (),
//                                                  intersectionPoint);
//            if (hasIntersection) {
//                float intersectionDistance = Vec3Df::squaredDistance (intersectionPoint + o.getTrans (),
//                                                                      p);
//                //iterate triangles
//                if (intersectionDistance <  radius ){
//                    Mesh m = o.getMesh();
//                    vector<Triangle> triangles = m.getTriangles();
//                    vector<Vertex> vertices = m.getVertices();
//                    for (unsigned int i = 0; i < triangles.size(); i++) {
//                        Triangle t = triangles[i];
//                        float intersection = ray.intersect(vertices[t.getVertex(0)].getPos(), vertices[t.getVertex(1)].getPos(), vertices[t.getVertex(2)].getPos(), intersectionPoint);
//                        if (intersection >= 0 && intersection < radius) {
//                            d = 0;
//                            break;
//                        }

//                    }
//                }

//            }
//            if( d == 0) break;
//        }

        ///////////////////methode apres KDTree
        ///
        std::vector<Triangle> interceptedTriangles;
        std::vector<Vec3Df> interceptedTrianglesPoints;
        std::vector<Vec3Df> interceptedTrianglesBarys;
        std::vector<float> interceptedTrianglesDistance;
        std::vector<unsigned int> objects;
        //Vec3Df intersectionPoint;

        getInterceptedTriangles (ray, objects, interceptedTriangles, interceptedTrianglesPoints, interceptedTrianglesBarys, interceptedTrianglesDistance);
        if(interceptedTriangles.size()==0) d=1;
        for (unsigned int a = 0; a < interceptedTriangles.size(); a++) {
                        float intersectionDistance = interceptedTrianglesDistance[a]*ray.getDirection().getLength();
                        if (intersectionDistance <  radius ){
                            d = 0;
                            break;
                        }
            }

        ao += d * Vec3Df::dotProduct(dir,normal);
    }
    ao/=nbr_rayon;
    return ao;
}



