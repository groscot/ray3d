//
//  KDTree.cpp
//  raytracer
//
//  Created by Raphaël Groscot on 02/05/14.
//  Copyright (c) 2014 Raphaël Groscot. All rights reserved.
//

#include "KDTree.h"

#include <vector>
#include <algorithm>
#include "BoundingBox.h"

using namespace std;

static const int MAX_RECURSION_LEVEL = 16;
const static int MAX_LEAF_ELEMENTS = 200;
static const bool SHOW_OUTPUT = false;

KDTree::KDTree () : level(0) {
    //innerTriangles = std::vector<unsigned int>();
}

KDTree::KDTree (const BoundingBox & bbox, int state, int level) : bbox (bbox), state (state), level (level) {
    //innerTriangles = std::vector<unsigned int>();
}

KDTree::KDTree (const BoundingBox & box, const Mesh & mesh) {
    state = 0;
    level = 0;
    bbox = box;
    //innerTriangles = std::vector<unsigned int>();
    std::cout << "+ KDTree" << std::endl;
    std::vector<unsigned int> v;
    for (unsigned int i = 0; i < mesh.getTriangles().size(); i++) v.push_back(i);

    expand(v, mesh);
    std::cout << "Finished KDTree!" << std::endl;
}

void KDTree::getRecursiveBboxes(int max_rec, vector<BoundingBox> & bbs) {
    if (isLeaf()) {
        bbs.push_back(bbox);
    } else {
        if (level >= max_rec) {
            bbs.push_back(left_child->getBbox());
            bbs.push_back(right_child->getBbox());
        } else {
            left_child->getRecursiveBboxes(max_rec, bbs);
            right_child->getRecursiveBboxes(max_rec, bbs);
        }
    }
}

void KDTree::expand(const std::vector<unsigned int> & triangles, const Mesh & mesh) {
    //std::cout << triangles.size() << " triangles" << std::endl;
    if (level > MAX_RECURSION_LEVEL || triangles.size() <= MAX_LEAF_ELEMENTS) {
        // We're a leaf!
        leaf = true;

        //cout << (bbox.getMax() - bbox.getMin()).getLength() << endl;

        for (unsigned int k = 0; k < triangles.size(); k++) {
            innerTriangles.push_back(triangles[k]);
            //if(!bbox.contains(mesh.getTriangle(triangles[k]),mesh)) exit(-1);
        }

        if (SHOW_OUTPUT) {
            for (unsigned int i = 0; i < level; i++) std::cout << "|  ";
            std::cout << " --* leaf (" << innerTriangles.size() << ")" << std::endl;
        }

        left_child = NULL;
        right_child = NULL;
    } else {
        // We're a tree!
        leaf = false;

        int next = getNextState();
        BoundingBox leftBbox, rightBbox;

        // Calcul de la mediane
        vector<float> pointsProjetes;
        vector<Vertex> vertices = mesh.getVertices();
        for (unsigned int i = 0; i < triangles.size(); i++) {
            float centre = 0.f;
            for (unsigned int j = 0; j < 3; j++) {
                centre += vertices[mesh.getTriangle(triangles[i]).getVertex(j)].getPos()[state];
            }
            pointsProjetes.push_back(centre/3);

        }
        nth_element(pointsProjetes.begin(), pointsProjetes.begin()+pointsProjetes.size()/2, pointsProjetes.end());
        float med = pointsProjetes.at(pointsProjetes.size()/2-1);
        float min = bbox.getMin()[state];
        float max = bbox.getMax()[state];

        float alpha;
        if ((med - min)/(max - min) < 0.00001f || (med - min)/(max - min) > 1.f - 0.00001f) alpha = 0.5f;// cas degenere
        else alpha = (med - min)/(max - min);
        bbox.split(leftBbox, rightBbox, state, alpha);

        if (SHOW_OUTPUT) {
            for (unsigned int i = 0; i < level; i++) std::cout << "|  ";
            std::cout << "|--+ Branch (" << triangles.size() << ")/" << state << std::endl;
        }

        left_child = new KDTree (leftBbox, next, level+1);
        right_child = new KDTree (rightBbox, next, level+1);


        std::vector<unsigned int> left_triangles;
        std::vector<unsigned int> right_triangles;

        for (unsigned int i = 0; i < triangles.size(); i++) {
            bool none = true;
            if (leftBbox.contains(mesh.getTriangle(triangles[i]), mesh)) {
                left_triangles.push_back(triangles[i]);
                none = false;
            }
            //else
            if (rightBbox.contains(mesh.getTriangle(triangles[i]), mesh)) {
                right_triangles.push_back(triangles[i]);
                none = false;
            }
            if (none) {
                cout << endl << "ERROR level " << level << " (state " << state << ")" << endl;
                cout << bbox.getMin() << " : " << bbox.getMax() << endl;
                cout << leftBbox.getMin() << " : " << leftBbox.getMax() << endl;
                cout << rightBbox.getMin() << " : " << rightBbox.getMax() << endl;
                exit(-1);
            }
        }

        left_child->expand(left_triangles, mesh);
        right_child->expand(right_triangles, mesh);
    }
}
