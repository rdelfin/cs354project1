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
    std::vector<Geometry*> objects;


    KdNode(KdNode* left, KdNode* right, BoundingBox bounds);
    ~KdNode();
};

class KdTree {
public:
    KdTree(Scene* scene, int objThreshold, int maxIterations);

    void construct();

    ~KdTree();
private:
    BoundingBox getSceneBounds();
    void setupNode(KdNode* node, int iterationsLeft);
    BoundingBox getNextBound(std::vector<Geometry*> objects, BoundingBox bounds);
    BoundingBox remainingBoundingBox(BoundingBox rest, BoundingBox total);

    Scene* scene;
    bool created;

    int objThreshold;
    int maxIterations;

    KdNode* root;

};


#endif //GLSL_KDTREE_HPP
