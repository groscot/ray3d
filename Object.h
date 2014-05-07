
#ifndef OBJECT_H
#define OBJECT_H

#include <iostream>
#include <vector>

#include "Mesh.h"
#include "Material.h"
#include "Ray.h"
#include "BoundingBox.h"
#include "KDTree.h"

class Object {
public:
    inline Object () {}
    inline Object (const Mesh & mesh, const Material & mat) : mesh (mesh), mat (mat) {
        updateBoundingBox ();
		createKDTree();
        std::cout << "done (" << tree->countTriangles() << " triangles)" << std::endl;
    }
    virtual ~Object () {}

    inline const Vec3Df & getTrans () const { return trans;}
    inline void setTrans (const Vec3Df & t) { trans = t; }

    inline const Mesh & getMesh () const { return mesh; }
    inline Mesh & getMesh () { return mesh; }
	
	inline KDTree * getKDTree () const { return tree; }
    
    inline const Material & getMaterial () const { return mat; }
    inline Material & getMaterial () { return mat; }

    inline const BoundingBox & getBoundingBox () const { return bbox; }
    void updateBoundingBox ();
	inline void createKDTree() { tree = new KDTree(bbox, mesh); }
    
private:
    Mesh mesh;
    Material mat;
    BoundingBox bbox;
    Vec3Df trans;
	KDTree* tree;
};


#endif // Scene_H

// Some Emacs-Hints -- please don't remove:
//
//  Local Variables:
//  mode:C++
//  tab-width:4
//  End:
