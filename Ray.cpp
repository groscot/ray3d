// *********************************************************
// Ray Class
// Author : Tamy Boubekeur (boubek@gmail.com).
// Copyright (C) 2010 Tamy Boubekeur.
// All rights reserved.
// *********************************************************

#include "Ray.h"

using namespace std;

static const unsigned int NUMDIM = 3, RIGHT = 0, LEFT = 1, MIDDLE = 2;

bool Ray::intersect (const BoundingBox & bbox, Vec3Df & intersectionPoint) const {
    const Vec3Df & minBb = bbox.getMin ();
    const Vec3Df & maxBb = bbox.getMax ();
    bool inside = true;
    unsigned int  quadrant[NUMDIM];
    register unsigned int i;
    unsigned int whichPlane;
    Vec3Df maxT;
    Vec3Df candidatePlane;
    
    for (i=0; i<NUMDIM; i++)
        if (origin[i] < minBb[i]) {
            quadrant[i] = LEFT;
            candidatePlane[i] = minBb[i];
            inside = false;
        } else if (origin[i] > maxBb[i]) {
            quadrant[i] = RIGHT;
            candidatePlane[i] = maxBb[i];
            inside = false;
        } else	{
            quadrant[i] = MIDDLE;
        }

    if (inside)	{
        intersectionPoint = origin;
        return (true);
    }

    for (i = 0; i < NUMDIM; i++)
        if (quadrant[i] != MIDDLE && direction[i] !=0.)
            maxT[i] = (candidatePlane[i]-origin[i]) / direction[i];
        else
            maxT[i] = -1.;

    whichPlane = 0;
    for (i = 1; i < NUMDIM; i++)
        if (maxT[whichPlane] < maxT[i])
            whichPlane = i;

    if (maxT[whichPlane] < 0.) return (false);
    for (i = 0; i < NUMDIM; i++)
        if (whichPlane != i) {
            intersectionPoint[i] = origin[i] + maxT[whichPlane] *direction[i];
            if (intersectionPoint[i] < minBb[i] || intersectionPoint[i] > maxBb[i])
                return (false);
        } else {
            intersectionPoint[i] = candidatePlane[i];
        }
    return (true);			
}


// Ray.cpp

float Ray::intersect (const Vec3Df & p0, const Vec3Df & p1, const Vec3Df & p2, Vec3Df & intersectionPoint) const {
    Vec3Df e0 = p1-p0;
    Vec3Df e1 = p2-p0;
    Vec3Df n = Vec3Df::crossProduct(e0,e1);
    n.normalize();
    Vec3Df q = Vec3Df::crossProduct(direction,e1);
    float a = Vec3Df::dotProduct(e0,q);

    if (Vec3Df::dotProduct(n,direction) >= 0 || a < epsilon)
        return -1;

    Vec3Df s = (origin-p0)/a;
    Vec3Df r = Vec3Df::crossProduct(s,e0);

    intersectionPoint[1] = Vec3Df::dotProduct(s,q);
    intersectionPoint[2] = Vec3Df::dotProduct(r,direction);
    intersectionPoint[0] = 1 - intersectionPoint[2] - intersectionPoint[1];

    if (intersectionPoint[0] < 0 || intersectionPoint[1] < 0 || intersectionPoint[2] < 0)
        return -1;

    float t = Vec3Df::dotProduct(e1,r);

    if (t < 0)
        return -1;
    else
        return t;
}

float Ray::intersect (const Vec3Df & p0, const Vec3Df & p1, const Vec3Df & p2, Vec3Df & intersectionPoint, Vec3Df & bary) const {
    Vec3Df e0 = p1-p0;
    Vec3Df e1 = p2-p0;
    Vec3Df n = Vec3Df::crossProduct(e0,e1);
    Vec3Df dir (this->direction);
    n.normalize();
    Vec3Df q = Vec3Df::crossProduct(dir,e1);
    float a = Vec3Df::dotProduct(e0,q);

    if (Vec3Df::dotProduct(n,dir) >= 0 || std::abs(a) < epsilon)
        return -1;

    Vec3Df s = (this->origin-p0)/a;
    Vec3Df r = Vec3Df::crossProduct(s,e0);

    bary[1] = Vec3Df::dotProduct(s,q);
    bary[2] = Vec3Df::dotProduct(r,dir);
    bary[0] = 1 - bary[2] - bary[1];

    if (bary[0] < 0 || bary[1] < 0 || bary[2] < 0)
        return -1;

    float t = Vec3Df::dotProduct(e1,r);
    intersectionPoint = Vec3Df(p0 * bary[0] + p1 * bary[1] + p2 * bary[2])/(bary[0] + bary[1] + bary[2]);

    if (t < 0)
        return -1;
    else
        return t;
}

bool Ray::hasIntersection (const Triangle & triangle, const Mesh & mesh) const {
    unsigned int v0 = triangle.getVertex(0);
    unsigned int v1 = triangle.getVertex(1);
    unsigned int v2 = triangle.getVertex(2);

    Vec3Df p0 = mesh.getVertices().at(v0).getPos();
    Vec3Df p1 = mesh.getVertices().at(v1).getPos();
    Vec3Df p2 = mesh.getVertices().at(v2).getPos();

    float t;
    Vec3Df intersectionPoint  = Vec3Df();
    Vec3Df bary = Vec3Df();

    t = this->intersect(p0, p1, p2, intersectionPoint, bary);

    if (t >= 0)
        return true;
    else
        return false;
}

