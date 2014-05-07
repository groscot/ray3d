/*
LibBoubek/BoundingBox.h
Copyright (c) 2003-2008, Tamy Boubekeur

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* The name of the source code copyright owner cannot be used to endorse or
  promote products derived from this software without specific prior
  written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <vector>
#include <algorithm>

#include "Vec3D.h"
#include "Triangle.h"
#include "Mesh.h"

const float BOUNDINGBOX_EPSILON = 0.0001f;

class BoundingBox {
public:
    BoundingBox () : minBb (Vec3Df (0.0f, 0.0f, 0.0f)), maxBb (Vec3Df (0.0f, 0.0f, 0.0f)) {}
    BoundingBox (const Vec3Df & p) : minBb (p), maxBb (p) {}
    BoundingBox (const Vec3Df & min, const Vec3Df & max) : minBb (min), maxBb (max) {}

    inline void init (const Vec3Df & p) {
        minBb = maxBb = p;
    }
    inline void init (float x, float y, float z) {
        init (Vec3Df (x, y, z));
    }
    inline float getWidth () const {
        return (maxBb[0] - minBb[0]);
    }
    inline float getHeight () const {
        return (maxBb[1] - minBb[1]);
    }
    inline float getLength () const {
        return (maxBb[2] - minBb[2]);
    }
    inline float getSize () const {
        return std::max (getWidth (), std::max (getHeight (), getLength ()));
    }
    inline float getRadius () const {
        return Vec3Df::distance (minBb, maxBb) / 2.0;
    }
    inline void extendTo (const Vec3Df & p) {
        for (unsigned int i = 0; i < 3; i++) {
            if (p[i] > maxBb[i]) maxBb[i] = p[i];
            if (p[i] < minBb[i]) minBb[i] = p[i];
        }
    }
    inline void extendTo (const BoundingBox & b) {
        extendTo (b.minBb);
        extendTo (b.maxBb);
    }
    inline bool contains (const Vec3Df & p) const {
        for (unsigned int i = 0; i < 3; i++)
            if (!(p[i] >= minBb[i] && p[i] <= maxBb[i]))
                return false;
        return true;
    }
    inline bool contains (const BoundingBox & b) const {
        for (unsigned int i = 0; i < 3; i++)
            if (fabs (getMiddle (i) - b.getMiddle (i)) - BOUNDINGBOX_EPSILON > (getWHL (i) + b.getWHL (i)) / 2.0)
                return false;
        return true;
    }
	inline bool contains (const Triangle & t, const Mesh & m) const {
        unsigned int v;
		Vec3Df bary;
		Vec3Df milieu1, milieu2, milieu3;
		// sommets
        for (unsigned int i = 0; i < 3; i++) {
            v = t.getVertex(i);
			bary += m.getVertices().at(v).getPos();
			if (contains(m.getVertices().at(v).getPos()))
				return true;
		}
		bary /= 3.f;
		
		// milieux
		milieu1 = (m.getVertices().at(0).getPos()+m.getVertices().at(1).getPos())/2.f;
		milieu2 = (m.getVertices().at(0).getPos()+m.getVertices().at(2).getPos())/2.f;
		milieu3 = (m.getVertices().at(1).getPos()+m.getVertices().at(2).getPos())/2.f;
		
		if (contains(m.getVertices().at(0).getPos()*0.33f+m.getVertices().at(0).getPos()*0.67f)) return true;
		if (contains(m.getVertices().at(0).getPos()*0.67f+m.getVertices().at(0).getPos()*0.33f)) return true;
		
		if (contains(m.getVertices().at(0).getPos()*0.33f+m.getVertices().at(1).getPos()*0.67f)) return true;
		if (contains(m.getVertices().at(0).getPos()*0.67f+m.getVertices().at(1).getPos()*0.33f)) return true;
		
		if (contains(m.getVertices().at(1).getPos()*0.33f+m.getVertices().at(2).getPos()*0.67f)) return true;
		if (contains(m.getVertices().at(1).getPos()*0.67f+m.getVertices().at(2).getPos()*0.33f)) return true;
		
		if (contains(milieu1)) return true;
		if (contains(milieu2)) return true;
		if (contains(milieu3)) return true;
		
		// barycentre
		if (contains(bary))
			return true;
		
        return false;
    }
    inline bool scaleContains (const Vec3Df & p, float scale) const {
        float limitW = scale * getWidth () / 2.0;
        float limitH = scale * getHeight () / 2.0;
        float limitL = scale * getLength () / 2.0;
        Vec3Df c = getCenter ();
        if ((fabs (c[0] - p[0]) <= limitW) && (fabs (c[1] - p[1]) <= limitH) && (fabs (c[2] - p[2]) <= limitL))
            return true;
        return false;
    }
    inline void scale (float factor) {
        Vec3Df center = getCenter ();
        Vec3Df diffMin, diffMax;
        diffMin.fromTo (center, minBb);
        diffMax.fromTo (center, maxBb);
        diffMin *= factor;
        diffMax *= factor;
        minBb = center + diffMin;
        maxBb = center + diffMax;
    }
    inline Vec3Df getCenter () const {
        return (minBb + maxBb) / 2;
    }
    inline const Vec3Df & getMin () const {
        return minBb;
    }
    inline const Vec3Df & getMax () const {
        return maxBb;
    }
    inline void subdivide (std::vector<BoundingBox> & splitBoundingBoxArray) const {
        Vec3Df med = (minBb + maxBb) / 2;
        float x_2 = (maxBb[0] - minBb [0]) / 2;
        float y_2 = (maxBb[1] - minBb [1]) / 2;
        float z_2 = (maxBb[2] - minBb [2]) / 2;
        splitBoundingBoxArray.resize (8);
        splitBoundingBoxArray[0] = BoundingBox (minBb, med);
        splitBoundingBoxArray[1] = BoundingBox (minBb + Vec3Df (x_2, 0.0, 0.0), med + Vec3Df (x_2, 0.0, 0.0));
        splitBoundingBoxArray[2] = BoundingBox (minBb + Vec3Df (0.0, y_2, 0.0), med + Vec3Df (0.0, y_2, 0.0));
        splitBoundingBoxArray[3] = BoundingBox (minBb + Vec3Df (x_2, y_2, 0.0), med + Vec3Df (x_2, y_2, 0.0));
        splitBoundingBoxArray[4] = BoundingBox (minBb + Vec3Df (0.0, 0.0, z_2), med + Vec3Df (0.0, 0.0, z_2));
        splitBoundingBoxArray[5] = BoundingBox (minBb + Vec3Df (x_2, 0.0, z_2), med + Vec3Df (x_2, 0.0, z_2));
        splitBoundingBoxArray[6] = BoundingBox (minBb + Vec3Df (0.0, y_2, z_2), med + Vec3Df (0.0, y_2, z_2));
        splitBoundingBoxArray[7] = BoundingBox (minBb + Vec3Df (x_2, y_2, z_2), med + Vec3Df (x_2, y_2, z_2));
    }
	inline void split (BoundingBox & leftBoundingBox, BoundingBox & rightBoundingBox, int axis) const {
        Vec3Df med1 = maxBb;
        Vec3Df med2 = minBb;
        med1[axis] = (maxBb[axis] + minBb [axis]) / 2;
        med2[axis] = med1[axis];
        leftBoundingBox = BoundingBox (minBb, med1);
        rightBoundingBox = BoundingBox (med2, maxBb);
    }
    inline void split (BoundingBox & leftBoundingBox, BoundingBox & rightBoundingBox, int axis, float alpha) const {
        Vec3Df med1 = maxBb;
        Vec3Df med2 = minBb;
        med1[axis] = alpha*maxBb[axis] + (1.f-alpha)*minBb[axis];
        med2[axis] = med1[axis];
		if (alpha < 0.00001f) {
			std::cout << "aaa";
			exit(-1);
		}
		if (maxBb[axis] == minBb[axis]) {
			std::cout << "bbb";
			exit(-1);
		}
        leftBoundingBox = BoundingBox (minBb, med1);
        rightBoundingBox = BoundingBox (med2, maxBb);
    }
    bool intersectRay (const Vec3Df & origin, const Vec3Df & direction, Vec3Df & intersection) const;

private:
    inline float getWHL (unsigned int i) const {
        return (maxBb[i] - minBb[i]);
    }
    inline float getMiddle (unsigned int i) const {
        return ((minBb[i] + maxBb[i]) / 2.0);
    }
    static inline bool isIn (float x, float min, float max) {
        return (x >= min && x <= max);
    }

    Vec3Df minBb, maxBb;
};

#endif // BOUNDINGBOX_H

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
