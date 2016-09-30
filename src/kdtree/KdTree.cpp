//
// Created by rdelfin on 9/27/16.
//

#include "KdTree.hpp"


KdNode:: KdNode(KdNode *left, KdNode *right, BoundingBox bounds)
        :left(left), right(right), bounds(bounds) {

}

KdNode::~KdNode() {
    if(left != nullptr)
        delete left;
    if(right != nullptr)
        delete right;

}

KdTree::KdTree(Scene *scene, int objThreshold, int maxIterations)
    : scene(scene), created(false), root(nullptr), objThreshold(objThreshold), maxIterations(maxIterations) {

}


void KdTree::construct() {
    if(root == nullptr)
        delete root;

    BoundingBox sceneBound = getSceneBounds();

    root = new KdNode(nullptr, nullptr, sceneBound);

    std::vector<Geometry*> objectsLeft(scene->beginObjects(), scene->endObjects());
    setupNode(root, maxIterations);
}

void KdTree::setupNode(KdNode* node, int iterations) {

    BoundingBox childMax = getNextBound(node->objects, node->bounds);
    BoundingBox childMin = remainingBoundingBox(childMax, node->bounds);

    // Convention: Left is min, right is max
    std::vector<Geometry*> leftObj, rightObj;

    for(auto it = node->objects.begin(); it != node->objects.end(); ++it) {
        if(childMax.intersects((*it)->getBoundingBox())) {
            rightObj.push_back(*it);
        }

        if(childMin.intersects((*it)->getBoundingBox())) {
            leftObj.push_back(*it);
        }
    }

    node->left = new KdNode(nullptr, nullptr, childMin);
    node->right = new KdNode(nullptr, nullptr, childMax);

    if(iterations > 0) {
        if (node->right->objects.size() > objThreshold)
            setupNode(node->left, iterations - 1);
        if (node->left->objects.size() > objThreshold)
            setupNode(node->right, iterations - 1);
    }
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


BoundingBox KdTree::getNextBound(std::vector<Geometry *> objects, BoundingBox bounds) {
    BoundingBox result;
    result.setMax(bounds.getMax());
    result.setMin(glm::dvec3(bounds.getMax().x + (bounds.getMax().x - bounds.getMin().x) / 2.0,
                             bounds.getMax().y,
                             bounds.getMax().z));
    return result;
}

BoundingBox KdTree::remainingBoundingBox(BoundingBox rest, BoundingBox total) {
    if(rest.getMax() == total.getMax()) {
        BoundingBox result;
        result.setMin(total.getMin());

        if(result.getMin().x != total.getMin().x) {
            result.setMax(glm::dvec3(rest.getMin().x, rest.getMax().y, rest.getMax().z));
        }
        else if(result.getMin().y != total.getMin().y) {
            result.setMax(glm::dvec3(rest.getMax().x, rest.getMin().y, rest.getMax().z));
        }
        else if(result.getMin().z != total.getMin().z) {
            result.setMax(glm::dvec3(rest.getMax().x, rest.getMax().y, rest.getMin().z));
        } else {
            return BoundingBox();
        }

        return result;

    } else if(rest.getMin() == total.getMin()) {
        BoundingBox result;
        result.setMax(total.getMax());

        if(result.getMax().x != total.getMax().x) {
            result.setMax(glm::dvec3(rest.getMax().x, rest.getMin().y, rest.getMin().z));
        }
        else if(result.getMin().y != total.getMin().y) {
            result.setMax(glm::dvec3(rest.getMin().x, rest.getMax().y, rest.getMin().z));
        }
        else if(result.getMin().z != result.getMax().z) {
            result.setMax(glm::dvec3(rest.getMin().x, rest.getMin().y, rest.getMax().z));
        } else {
            return BoundingBox();
        }

        return result;
    }

    return BoundingBox();
}

KdTree::~KdTree() {
    delete root;
}



