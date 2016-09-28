//
// Created by rdelfin on 9/27/16.
//

#include "KdTree.hpp"


KdNode::KdNode(KdNode *left, KdNode *right, BoundingBox bounds)
        :left(left), right(right), bounds(bounds) {

}

KdNode::~KdNode() {
    if(left != nullptr)
        delete left;
    if(right != nullptr)
        delete right;

}

KdTree::KdTree(Scene *scene)
    : scene(scene), created(false), root(nullptr) {

}


void KdTree::construct() {
    if(root == nullptr)
        delete root;

    BoundingBox sceneBound = getSceneBounds();

    root = new KdNode(nullptr, nullptr, sceneBound);

    std::vector<Geometry*> objectsLeft(scene->beginObjects(), scene->endObjects());

    
}

BoundingBox KdTree::getSceneBounds() {
    BoundingBox result;

    auto it = scene->beginObjects();
    if(it == scene->endObjects()) return result;

    result = (*it)->getBoundingBox();

    for(; it != scene->endObjects(); ++it) {
        BoundingBox thisBB = (*it)->getBoundingBox();
        glm::dvec3 pastMax = thisBB.getMax();
        glm::dvec3 pastMin = thisBB.getMax();

        glm::dvec3 currMax = result.getMax();
        glm::dvec3 currMin = result.getMin();

        glm::dvec3 newMax(glm::max(pastMax.x, currMax.x), glm::max(pastMax.y, currMax.y), glm::max(pastMax.z, currMax.z));
        glm::dvec3 newMin(glm::min(pastMin.x, currMin.x), glm::min(pastMin.y, currMin.y), glm::min(pastMin.z, currMin.z));
    }
}


KdTree::~KdTree() {
    delete root;
}
