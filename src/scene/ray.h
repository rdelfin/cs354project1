//
// ray.h
//
// The low-level classes used by ray tracing: the ray and isect classes.
//

#ifndef __RAY_H__
#define __RAY_H__

// who the hell cares if my identifiers are longer than 255 characters:
#pragma warning(disable : 4786)

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include "material.h"
#include "../ui/TraceUI.h"

class SceneObject;

// A ray has a position where the ray starts, and a direction (which should
// always be normalized!)

class ray {
public:
	enum RayType
	{
		VISIBILITY,
		REFLECTION,
		REFRACTION,
		SHADOW
	};

	ray(const glm::dvec3 &pp,
	    const glm::dvec3 &dd,
	    unsigned char *px,
	    unsigned int i,
	    const glm::dvec3 &w,
	    RayType tt = VISIBILITY)
		: p(pp), d(dd), pixel(px), ctr(i), atten(w), t(tt)
	{ TraceUI::addRay(ctr); }
	ray(const ray& other)
		: p(other.p),
		  d(other.d),
		  pixel(other.pixel),
		  ctr(other.ctr),
		  atten(other.atten),
		  t(other.t)
	{ TraceUI::addRay(ctr); }
	~ray() {}

	ray& operator =( const ray& other ) 
	{
		p = other.p;
		d = other.d;
		pixel = other.pixel;
		ctr = other.ctr;
		atten = other.atten;
		t = other.t;
		return *this;
	}

	glm::dvec3 at( double t ) const
	{ return p + (t*d); }

	glm::dvec3 getPosition() const { return p; }
	glm::dvec3 getDirection() const { return d; }
	unsigned char* getPixel() const { return pixel; }
	glm::dvec3 getAtten() const { return atten; }
	RayType type() const { return t; }

public:
	glm::dvec3 p;
	glm::dvec3 d;
	unsigned int ctr;
	unsigned char* pixel;
	glm::dvec3 atten;
	RayType t;
};


// The description of an intersection point.

class isect
{
public:
    isect() : obj( NULL ), t( 0.0 ), N(), material(0) {}
	isect(const isect& other)
	{
		obj = other.obj;
		t = other.t;
		N = other.N;
		bary = other.bary;
		uvCoordinates = other.uvCoordinates;
		if (other.material) material = new Material(*other.material);
		else material = 0;
	}

	~isect() { delete material; }
    
    isect& operator = (const isect& other) {
        if( this != &other ) {
            obj = other.obj;
            t = other.t;
            N = other.N;
			bary = other.bary;
            uvCoordinates = other.uvCoordinates;
			if( other.material ) {
                if( material ) *material = *other.material;
                else material = new Material(*other.material );
            }
            else {
                delete material;
                material = 0;
            }
        }
        return *this;
    }

    void setObject(const SceneObject *o) { obj = o; }
    void setT(double tt) { t = tt; }
    void setN(const glm::dvec3& n) { N = n; }
    void setMaterial(const Material& m)  { if(material) *material = m; else material = new Material(m); }
    void setUVCoordinates( const glm::dvec2& coords ) { uvCoordinates = coords; }
    void setBary(const glm::dvec3& weights) { bary = weights; }
    void setBary(const double alpha, const double beta, const double gamma)
		{ bary[0] = alpha; bary[1] = beta; bary[2] = gamma; }
    const Material &getMaterial() const;

public:
    const SceneObject *obj;
    double t;
    glm::dvec3 N;
    glm::dvec2 uvCoordinates;
    glm::dvec3 bary;
    Material *material;         // if this intersection has its own material
                                // (as opposed to one in its associated object)
                                // as in the case where the material was interpolated
};

const double RAY_EPSILON = 0.00000001;

#endif // __RAY_H__
