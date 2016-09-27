#include <iostream>
#include "cubeMap.h"
#include "ray.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

glm::dvec3 CubeMap::getColor(ray r) const {
    /* Indexing:
     * tMap[0] = +x
     * tMap[1] = -x
     * tMap[2] = +y
     * tMap[3] = -y
     * tMap[4] = +z
     * tMap[5] = -z

    /* We can look at the colision of a ray r(t)=r.d, ignoring the P term. We then look at a simple 2x2x2 cube centered
     * at the origin. We know the face we collide with is the one with the smallest t term. By convention, all these
     * planes will have a, b, or c set to +-1 and d to 1 (creating normals that face outwards). The solution for t is
     * therefore: t = 1/dot(n, r.d) */

    double tVals[6];
    glm::dvec3 normals[6] = {glm::dvec3(1, 0, 0), glm::dvec3(-1, 0, 0), glm::dvec3(0, 1, 0), glm::dvec3(0, -1, 0),
                             glm::dvec3(0, 0, 1), glm::dvec3(0, 0, -1)};

    int minIdx = -1;
    double minT = 0;
    for(int i = 0; i < 6; i++) {
        double t = 1.0 / glm::dot(normals[i], r.d);

        if(t > 0 && (minIdx == -1 || t < minT)) {
            minIdx = i;
            minT = t;
        }
    }

    // Calculate the intersection point. We can simply project this onto the surface
    glm::dvec3 point = r.d*minT;


    // Project vector onto the respective plane
    glm::dvec3 proj = point - glm::dot(point, normals[minIdx])*normals[minIdx];
    glm::dvec2 twoDProj(0, 0);

    if(minIdx == 0 || minIdx == 1)
        twoDProj = glm::dvec2(proj.z, proj.y);
    if(minIdx == 2 || minIdx == 3)
        twoDProj = glm::dvec2(proj.x, proj.z);
    if(minIdx == 4 || minIdx == 5)
        twoDProj = glm::dvec2(proj.x, proj.y);

    glm::dvec2 uvCoord = (twoDProj + glm::dvec2(1, 1)) / 2.0;
    int xCoord = (int)(tMap[minIdx]->getWidth()*uvCoord.x);
    int yCoord = (int)(tMap[minIdx]->getHeight()*uvCoord.y);

    return tMap[minIdx]->getPixelAt(xCoord, yCoord);
}