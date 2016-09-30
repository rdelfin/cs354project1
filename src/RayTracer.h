#ifndef __RAYTRACER_H__
#define __RAYTRACER_H__

#define MAX_THREADS 32

// The main ray tracer.

#include "scene/ray.h"
#include "scene/cubeMap.h"
#include "kdtree/KdTree.hpp"
#include <time.h>
#include <thread>
#include <queue>
#include <glm/vec3.hpp>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include <thread>


typedef std::unordered_map<std::pair<double, double>, glm::dvec3, boost::hash< std::pair<double, double> > > SampleMap;

class Scene;
class Pixel
{
public:
	Pixel(int i, int j, unsigned char* ptr) : ix(i), jy(j), value(ptr) {}

	int ix;
	int jy;
	unsigned char* value;
};


class RayTracer {
public:
    RayTracer();

    ~RayTracer();

    glm::dvec3 tracePixel(int i, int j, unsigned int ctr);

    glm::dvec3 trace(double x, double y, unsigned char *pixel, unsigned int ctr);

    glm::dvec3 traceRay(ray &r, const glm::dvec3 &thresh, int depth, double &length);

    glm::dvec3 getPixel(int i, int j);

    void setPixel(int i, int j, glm::dvec3 color);

    void getBuffer(unsigned char *&buf, int &w, int &h);

    double aspectRatio();

    void traceImage(int w, int h, int bs, double thresh);

    int aaImage(int samples, double aaThresh);

    bool checkRender();

    void traceSetup(int w, int h);

    void setThreshold(double th) { thresh = th; }

    void setaaThreshold(double th) { aaThresh = th; }

    void setThreads(int th) { threads = (unsigned) th; }

    void setSamples(int num) { samples = num; }

    void setCubeMap(CubeMap *m) {
        if (cubemap) delete cubemap;
        cubemap = m;
    }

    void enableKdTree() { kdTreeEnabled = true; }

    void disableKdTree() { kdTreeEnabled = false; }

    void setKdTreeDepth(int depth) { kdTreeDepth = depth; }

    void setKdTreeLeafSize(int size) { kdTreeLeafSize = size; }

    bool loadScene(char *fn);

    bool sceneLoaded() { return scene != 0; }

    bool haveCubeMap() { return cubemap != 0; }

    void setReady(bool ready) { m_bBufferReady = ready; }

    bool isReady() const { return m_bBufferReady; }

    const Scene &getScene() { return *scene; }

    CubeMap *getCubeMap() { return cubemap; }

    bool isKdTreeEnabled() { return kdTreeEnabled; }

    int getKdTreeDepth() { return kdTreeDepth; }

    int getKdTreeLeafSize() { return kdTreeLeafSize; }

private:
    void getSamples(int x, int y, int sampleLevel, SampleMap &oversampleMap);

    glm::dvec3 getAverageColor(int x, int y, int sampleLevel, SampleMap &oversampleMap);

    std::vector<std::thread> threadList;
    std::vector<bool> threadDone;
    std::vector<std::pair<int, int>> threadRayRange;

    void traceThread(unsigned int threadIdx);

public:
	unsigned char *buffer;
	int buffer_width, buffer_height;
	int bufferSize;
	unsigned int threads;
	double thresh;
	double aaThresh;
	int samples;
	Scene* scene;
	CubeMap* cubemap;

	bool m_bBufferReady;
	bool stopTrace;

	KdTree* kdtree;
	bool kdTreeEnabled;
	int kdTreeDepth;
    int kdTreeLeafSize;

};

#endif // __RAYTRACER_H__
