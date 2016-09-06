//
// rayUI.h
//
// The header file for the UI part
//

#ifndef __rayUI_h__
#define __rayUI_h__

// who the hell cares if my identifiers are longer than 255 characters:
#pragma warning(disable : 4786)

//#include <math.h>

#include <string>
//#include "../RayTracer.h"
#define MAX_THREADS 32

using std::string;

class RayTracer;

class TraceUI {
public:
	TraceUI()
		: m_nDepth(0), m_nSize(512), m_nBlockSize(4), m_nThreshold(0), m_nSuperSamples(3), m_nAaThreshold(100), m_nTreeDepth(15), m_nLeafSize(10), m_nFilterWidth(1),
		m_displayDebuggingInfo(false), m_antiAlias(false), m_kdTree(true), m_shadows(true), m_smoothshade(true), m_usingCubeMap(false), m_backface(true),
		raytracer(0)
	{ for (unsigned int i = 0; i < MAX_THREADS; i++) rayCount[i] = 0; }

	virtual int	run() = 0;

	// Send an alert to the user in some manner
	virtual void alert(const string& msg) = 0;

	// setters
	virtual void setRayTracer( RayTracer* r ) { raytracer = r; }
	void setCubeMap(bool b) { m_gotCubeMap = b; }
	void useCubeMap(bool b) { m_usingCubeMap = b; }

	// accessors:
	int	getSize() const { return m_nSize; }
	int	getDepth() const { return m_nDepth; }
	int	getBlockSize() const { return m_nBlockSize; }
	double	getThreshold() const { return (double)m_nThreshold * 0.001; }
	double	getAaThreshold() const { return (double)m_nAaThreshold * 0.001; }
	int	getSuperSamples() const { return m_nSuperSamples; }
	int	getMaxDepth() const { return m_nTreeDepth; }
	int	getLeafSize() const { return m_nLeafSize; }
	int	getFilterWidth() const { return m_nFilterWidth; }
	int	getThreads() const { return m_threads; }
	bool	aaSwitch() const { return m_antiAlias; }
	bool	kdSwitch() const { return m_kdTree; }
	bool	shadowSw() const { return m_shadows; }
	bool	smShadSw() const { return m_smoothshade; }
	bool	bkFaceSw() const { return m_backface; }
	bool	cubeMap() const { return m_usingCubeMap && m_gotCubeMap; }

	// ray counter
	static void addRays(int number, int ctr) { if (ctr >= 0) rayCount[ctr] += number; }
	static void addRay(int ctr) { if (ctr >= 0) rayCount[ctr]++; }
	static int getCount(int ctr) { return ctr < 0 ? -1 : rayCount[ctr]; }
	static int getCount()
	{
		int total = 0;
		for (int i = 0; i < m_threads; i++) total += rayCount[i];
		return total;
	}
	static int resetCount(int ctr)
	{
		if (ctr < 0 ) return -1;
		int temp = rayCount[ctr];
		rayCount[ctr] = 0;
		return temp;
	}
	static int resetCount()
	{
		int total = 0;
		for (int i = 0; i < m_threads; i++)
		{
			total += rayCount[i];
			rayCount[i] = 0;
		}
		return total;
	}

	static int m_threads;  // number of threads to run
	static bool m_debug;

protected:
	RayTracer*	raytracer;

	int	m_nSize;	// Size of the traced image
	int	m_nDepth;	// Max depth of recursion
	int m_nThreshold; // Threshold for interpolation within block
	int m_nBlockSize;	// Blocksize (square, even, power of 2 preferred)
	int m_nSuperSamples;	// Supersampling rate (1-d) for antialiasing
	int m_nAaThreshold;  // Pixel neighborhood difference for supersampling
	int m_nTreeDepth;  // maximum kdTree depth
	int m_nLeafSize;  // target number of objects per leaf
	int m_nFilterWidth;  // width of cubemap filter

	static int rayCount[MAX_THREADS];	// Ray counter

	// Determines whether or not to show debugging information
	// for individual rays.  Disabled by default for efficiency
	// reasons.
	bool m_displayDebuggingInfo;
	bool m_antiAlias; // Is antialiasing on?
	bool m_kdTree;  // use kd-tree?
	bool m_shadows;  // compute shadows?
	bool m_smoothshade;  // turn on/off smoothshading?
	bool m_backface;  // cull backfaces?
	bool m_usingCubeMap;  // render with cubemap
	bool m_gotCubeMap;  // cubemap defined

};

#endif
