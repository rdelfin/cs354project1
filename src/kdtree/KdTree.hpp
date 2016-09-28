//
// Created by rdelfin on 9/27/16.
//

#ifndef GLSL_KDTREE_HPP
#define GLSL_KDTREE_HPP


#include "../scene/scene.h"


class KdNode {
public:
    KdNode* left;
    KdNode* right;
    BoundingBox bounds;
    std::vector<Geometry> value;


    KdNode(KdNode* left, KdNode* right, BoundingBox bounds);
    ~KdNode();
};

class KdTree {
public:
    KdTree(Scene* scene);

    void construct();

    ~KdTree();
private:
    BoundingBox getSceneBounds();

    Scene* scene;
    bool created;

    KdNode* root;

};


#endif //GLSL_KDTREE_HPP
