// *********************************************************
// Scene Class
// Author : Tamy Boubekeur (boubek@gmail.com).
// Copyright (C) 2010 Tamy Boubekeur.
// All rights reserved.
// *********************************************************

#include "Scene.h"

using namespace std;

static Scene * instance = NULL;

Scene * Scene::getInstance () {
    if (instance == NULL)
        instance = new Scene ();
    return instance;
}

void Scene::destroyInstance () {
    if (instance != NULL) {
        delete instance;
        instance = NULL;
    }
}

Scene::Scene () {
    buildDefaultScene ();
    updateBoundingBox ();
}

Scene::~Scene () {
}

void Scene::updateBoundingBox () {
    if (objects.empty ())
        bbox = BoundingBox ();
    else {
        bbox = objects[0].getBoundingBox ();
        for (unsigned int i = 1; i < objects.size (); i++)
            bbox.extendTo (objects[i].getBoundingBox ());
    }
}

// Changer ce code pour creer des scenes originales
void Scene::buildDefaultScene () {
//    Mesh groundMesh;
//    groundMesh.loadOFF ("models/ground.off");
//    Material groundMat;
//    Object ground (groundMesh, groundMat);
//    objects.push_back (ground);

//    Mesh wall1Mesh;
//    wall1Mesh.loadOFF ("models/wall1.off");
//    Material wall1Mat (1.f, 1.f, Vec3Df (1.f, 0.f, 0.f));

//    Object wall1 (wall1Mesh , wall1Mat);
//    objects.push_back (wall1);
//    Mesh wall2Mesh;
//    wall2Mesh.loadOFF ("models/wall2.off");
//    Material wall2Mat (1.f, 1.f, Vec3Df (0.2f,0.2f,0.2f));
//    Object wall2 (wall2Mesh , wall2Mat);
//    objects.push_back (wall2);

//    Mesh wall3Mesh;
//    wall3Mesh.loadOFF ("models/wall3.off");
//    Material wall3Mat (1.f, 1.f, Vec3Df (0.2f,0.2f,0.2f));
//     wall3Mat.setMirror(true);
//    Object wall3 (wall3Mesh , wall3Mat);
//    objects.push_back (wall3);

//    Mesh sphereMesh;
//    sphereMesh.loadOFF ("models/sphere.off");
//    Material sphereMat (1.f, 1.f, Vec3Df (1.f, .6f, .5f));
//    Object sphere (sphereMesh , sphereMat);
//    sphere.setTrans (Vec3Df (0.2f, 0.f, 1.5));
//    objects.push_back (sphere);

//    Mesh cellMesh;
//    cellMesh.loadOFF ("models/cell.off");
//    Material cellMat (1.f, 1.f, Vec3Df (0.5f,0.5f,0.5f));
//    Object cell (cellMesh , cellMat);
//    objects.push_back (cell);
	
    Mesh ramMesh;
    ramMesh.loadOFF ("models/Safe.off");
    Material ramMat (1.f, 1.f, Vec3Df (1.f, .6f, .2f));
    Object ram (ramMesh, ramMat);
    //ram.setTrans (Vec3Df (1.f, 0.5f, 0.f));
    objects.push_back (ram);
	/*
	Mesh guitarMesh;
    guitarMesh.loadOFF ("models/guitar.off");
    Material guitarMat (1.f, 1.f, Vec3Df (1.f, .6f, .2f));
    Object guitar (guitarMesh, guitarMat);
    guitar.setTrans (Vec3Df (1.f, 0.5f, 0.f));
    objects.push_back (guitar);
	*/
//    Mesh rhinoMesh;
//    rhinoMesh.loadOFF ("models/rhino.off");
//    Material rhinoMat (1.0f, 0.2f, Vec3Df (0.6f, 0.6f, 0.7f));
//    Object rhino (rhinoMesh, rhinoMat);
//    rhino.setTrans (Vec3Df (-1.f, -1.0f, 0.4f));
//    objects.push_back (rhino);
//    Mesh gargMesh;
//    gargMesh.loadOFF ("models/gargoyle.off");
//    Material gargMat (0.7f, 0.4f, Vec3Df (0.5f, 0.8f, 0.5f));
//    Object garg (gargMesh, gargMat);
//    garg.setTrans (Vec3Df (-1.f, 1.0f, 0.1f));
//    objects.push_back (garg);
    //Light l (Vec3Df (2.0f, 2.0f, 8.0f), Vec3Df (1.0f, 1.0f, 1.0f), 1.0f);
     Light l (Vec3Df (120.0f, 120.0f, 120.0f), Vec3Df (1.0f, 1.0f, 1.0f), 1.0f);
     lights.push_back (l);
}
