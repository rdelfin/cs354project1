//
// Created by rdelfin on 9/27/16.
//

#include <unordered_map>
#include <boost/functional/hash.hpp>
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
        Geometry* obj = *it;

        // Special case: If object does not have bounding box, it will be left at the root node
        if(!obj->hasBoundingBoxCapability())
            continue;

        if(childMax.intersects(obj->getBoundingBox())) {
            rightObj.push_back(obj);
        }

        if(childMin.intersects(obj->getBoundingBox())) {
            leftObj.push_back(obj);
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

std::vector<Geometry*> KdTree::intersects(const ray &r) {
    // Returns the full list
    std::vector<KdNode*> nodes = recursiveIntersects(r, root);
    std::vector<Geometry*> result;

    for(auto it = nodes.begin(); it != nodes.end(); ++it) {
        KdNode* node = *it;

        for(auto objIt = node->objects.begin(); objIt != node->objects.end(); ++objIt) {
            Geometry* geometry = *objIt;

            if(std::find(result.begin(), result.end(), geometry) == result.end())
                result.push_back(geometry);
        }
    }

    return result;
}

std::vector<KdNode*> KdTree::recursiveIntersects(const ray &r, KdNode* n) {
    if(n->left == nullptr || n->right == nullptr) {
        return {n};
    }


    BoundingBox b = n->bounds;
    BoundingBox minBox = n->left->bounds;
    BoundingBox maxBox = n->right->bounds;

    /** This figures out the five planes used for identifying entry points, exit points, and boxes on which there are
     *  collisions
     */
    IntersectionType type = intersectionSide(r, b, minBox, maxBox);

    if(type == MIN)
        return recursiveIntersects(r, n->left);
    if(type == MAX)
        return recursiveIntersects(r, n->right);
    if(type == BOTH) {
        std::vector<KdNode *> left = recursiveIntersects(r, n->left);
        std::vector<KdNode *> right = recursiveIntersects(r, n->right);

        left.insert(left.end(), right.begin(), right.end());

        return left;
    }
}

KdTree::IntersectionType KdTree::intersectionSide(const ray& r, BoundingBox total, BoundingBox minBox, BoundingBox maxBox) {
    glm::dvec3 splitPlaneN;
    double splitPlaneD;

    glm::dvec3 aPlaneNormal;
    double minPlaneAD;
    double maxPlaneAD;

    glm::dvec3 bPlaneNormal;
    double minPlaneBD;
    double maxPlaneBD;

    // Figure out what all the planes are
    if(minBox.getMax().x != total.getMax().x) {
        splitPlaneN = glm::dvec3(1, 0, 0);
        splitPlaneD = minBox.getMax().x;

        aPlaneNormal = glm::dvec3(0, 1, 0);
        minPlaneAD = total.getMin().y;
        maxPlaneAD = total.getMax().y;

        bPlaneNormal = glm::dvec3(0, 0, 1);
        minPlaneBD = total.getMin().z;
        maxPlaneBD = total.getMax().z;
    } else if(minBox.getMax().y != total.getMax().y) {
        splitPlaneN = glm::dvec3(0, 1, 0);
        splitPlaneD = minBox.getMax().y;

        aPlaneNormal = glm::dvec3(1, 0, 0);
        minPlaneAD = total.getMin().x;
        maxPlaneAD = total.getMax().x;

        bPlaneNormal = glm::dvec3(0, 0, 1);
        minPlaneBD = total.getMin().z;
        maxPlaneBD = total.getMax().z;

    } else if(minBox.getMax().z != total.getMax().z) {
        splitPlaneN = glm::dvec3(0, 0, 1);
        splitPlaneD = minBox.getMax().z;

        aPlaneNormal = glm::dvec3(1, 0, 0);
        minPlaneAD = total.getMin().x;
        maxPlaneAD = total.getMax().x;

        bPlaneNormal = glm::dvec3(0, 1, 0);
        minPlaneBD = total.getMin().y;
        maxPlaneBD = total.getMax().y;
    }

    // Find intersection with all planes
    double splitT = (splitPlaneD - glm::dot(splitPlaneN, r.p)) / glm::dot(splitPlaneN, r.d);
    double minAT = (minPlaneAD - glm::dot(aPlaneNormal, r.p)) / glm::dot(aPlaneNormal, r.d);
    double maxAT = (maxPlaneAD - glm::dot(aPlaneNormal, r.p)) / glm::dot(aPlaneNormal, r.d);
    double minBT = (minPlaneBD - glm::dot(bPlaneNormal, r.p)) / glm::dot(bPlaneNormal, r.d);
    double maxBT = (maxPlaneBD - glm::dot(bPlaneNormal, r.p)) / glm::dot(bPlaneNormal, r.d);

    // Indentify which intersections come first in each pair of parallel planes
    bool minAFirst = minAT < maxAT;
    bool minBFirst = minBT < maxBT;

    // Name said plane intersactions
    double nearBT = minBFirst ? minBT : maxBT;
    double nearAT = minAFirst ? minAT : maxAT;
    double farBT = minBFirst ? maxBT : minBT;
    double farAT = minAFirst ? maxAT : minAT;

    // Identify which intersections identify entry and exit points
    double nearT = nearAT > nearBT ? nearAT : nearBT;
    double farT = farAT > farBT ? farBT : farAT;

    if(nearT < splitT && splitT < farT)
        return BOTH;
    else {
        glm::dvec3 pos = r.at(farT + (farT - nearT) / 2.0);
        if(pos.x <  minBox.getMax().x && pos.y < minBox.getMax().y && pos.z < minBox.getMax().z)
            return MIN;
        else
            return MAX;
    }
}



