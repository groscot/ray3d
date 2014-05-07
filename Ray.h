// *********************************************************
// Ray Class
// Author : Tamy Boubekeur (boubek@gmail.com).
// Copyright (C) 2010 Tamy Boubekeur.
// All rights reserved.
// *********************************************************

#ifndef RAY_H
#define RAY_H

#include <iostream>
#include <vector>

#include "Vec3D.h"
#include "BoundingBox.h"
#include "Object.h"

class Ray {
public:
    inline Ray () {}
    inline Ray (const Vec3Df & origin, const Vec3Df & direction)
        : origin (origin), direction (direction) {}
    inline virtual ~Ray () {}

    inline const Vec3Df & getOrigin () const { return origin; }
    inline Vec3Df & getOrigin () { return origin; }
    inline const Vec3Df & getDirection () const { return direction; }
    inline Vec3Df & getDirection () { return direction; }

    bool intersect (const BoundingBox & bbox, Vec3Df & intersectionPoint) const;


    /// Renvoie la valeur de t, agit par effet de bord sur les coordonnees du point d'intersection
    float intersect (const Vec3Df & p0, const Vec3Df & p1, const Vec3Df & p2, Vec3Df & intersectionPoint) const;

    //
    float intersect (const Vec3Df & p0, const Vec3Df & p1, const Vec3Df & p2, Vec3Df & intersectionPoint, Vec3Df & bary) const;

    bool hasIntersection (const Triangle & triangle, const Mesh & mesh) const;
    static const float epsilon = 0.00001;

private:
    Vec3Df origin;
    Vec3Df direction;
};


#endif // RAY_H

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
