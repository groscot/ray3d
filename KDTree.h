//
//  KDTree.h
//  raytracer
//
//  Created by Raphaël Groscot on 02/05/14.
//  Copyright (c) 2014 Raphaël Groscot. All rights reserved.
//

#ifndef KDTREE_H
#define KDTREE_H

#include "BoundingBox.h"
#include "Triangle.h"
#include "Mesh.h"
//#include "Ray.h"
#include "BoundingBox.h"

#include <vector>

using namespace std;

/// KDtree for ONE object
class KDTree
{
public:
    // Constructors and destructor
    KDTree ();
    KDTree (const BoundingBox & bbox, int state = 0, int level = 0);
    KDTree (const BoundingBox & box, const Mesh & mesh);
    /*inline virtual ~KDTree () {
        delete left_child;
        left_child = NULL;
        delete right_child;
        right_child = NULL;
        innerTriangles.clear();
    }*/

    // Getters
    inline KDTree * getLeft() const { return left_child; }
    inline KDTree * getRight() const { return right_child; }
    inline bool isLeaf() const { return leaf; }
    inline int getNextState() const { return (state+1)%3; }
    inline std::vector<unsigned int> getInnerTriangles() const { return innerTriangles; }

    inline const BoundingBox getBbox() const { return bbox; }
    inline int getLevel() const { return level; }
    void getRecursiveBboxes(int max_rec, vector<BoundingBox> & bbs);

    int countTriangles () const {
        if (isLeaf()) {
            return innerTriangles.size();
        } else {
            return left_child->countTriangles() + right_child->countTriangles();
        }
    }

    /// Initializes the KDTree for the given Object
    void expand(const std::vector<unsigned int> & triangles, const Mesh & mesh);

private:
    BoundingBox bbox;
    int state;
    int level;
    KDTree * left_child;
    KDTree * right_child;
    std::vector<unsigned int> innerTriangles;
    bool leaf;
};

#endif
