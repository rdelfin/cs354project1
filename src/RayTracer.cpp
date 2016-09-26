// The main ray tracer.

#pragma warning (disable: 4786)

#include "RayTracer.h"
#include "scene/light.h"
#include "scene/material.h"
#include "scene/ray.h"

#include "parser/Tokenizer.h"
#include "parser/Parser.h"

#include "ui/TraceUI.h"
#include <cmath>
#include <algorithm>
#include <glm/glm.hpp>
#include <string.h> // for memset

#include <iostream>
#include <fstream>

using namespace std;
extern TraceUI* traceUI;

// Use this variable to decide if you want to print out
// debugging messages.  Gets set in the "trace single ray" mode
// in TraceGLWindow, for example.
bool debugMode = false;

double eps = 0.000001;

// Trace a top-level ray through pixel(i,j), i.e. normalized window coordinates (x,y),
// through the projection plane, and out into the scene.  All we do is
// enter the main ray-tracing method, getting things started by plugging
// in an initial ray weight of (0.0,0.0,0.0) and an initial recursion depth of 0.

glm::dvec3 RayTracer::trace(double x, double y, unsigned char *pixel, unsigned int ctr)
{
    // Clear out the ray cache in the scene for debugging purposes,
  if (TraceUI::m_debug) scene->intersectCache.clear();

    ray r(glm::dvec3(0,0,0), glm::dvec3(0,0,0), pixel, ctr, glm::dvec3(1,1,1), ray::VISIBILITY);
    scene->getCamera().rayThrough(x,y,r);
    double dummy;
    glm::dvec3 ret = traceRay(r, glm::dvec3(1.0,1.0,1.0), traceUI->getDepth() , dummy);
    ret = glm::clamp(ret, 0.0, 1.0);
    return ret;
}

glm::dvec3 RayTracer::tracePixel(int i, int j, unsigned int ctr)
{
	glm::dvec3 col(0,0,0);

	if( ! sceneLoaded() ) return col;

	double x = double(i)/double(buffer_width);
	double y = double(j)/double(buffer_height);

	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;
	col = trace(x, y, pixel, ctr);

	pixel[0] = (int)( 255.0 * col[0]);
	pixel[1] = (int)( 255.0 * col[1]);
	pixel[2] = (int)( 255.0 * col[2]);
	return col;
}


// Do recursive ray tracing!  You'll want to insert a lot of code here
// (or places called from here) to handle reflection, refraction, etc etc.
glm::dvec3 RayTracer::traceRay(ray& r, const glm::dvec3& thresh, int depth, double& t )
{
	isect i;
	glm::dvec3 colorC;

	if(scene->intersect(r, i)) {
        glm::dvec3 reflectedColor(0.0, 0.0, 0.0);
        glm::dvec3 refractedColor(0.0, 0.0, 0.0);
        Material mat = i.getMaterial();

        if(depth > 0) {

            if(debugMode)
                std::cout << "DEPTH " << depth << std::endl;

            // Reflection
            if(mat.Refl()) {
                glm::dvec3 ref = glm::normalize(r.getDirection() - 2.0 * glm::dot(i.N, r.getDirection()) * i.N);
                ray refRay(r.getPosition() + i.t * r.getDirection() + ref * eps, ref, r.getPixel(), r.ctr, r.getAtten(),
                           ray::REFLECTION);
                reflectedColor = traceRay(refRay, thresh, depth - 1, t);
            }


            // Refraction
            glm::dvec3 n = i.N;
            glm::dvec3 d = -r.getDirection();
            double c = glm::dot(n, d);
            double n1 = (c < 0 ? i.getMaterial().index(i) : 1);
            double n2 = (c < 0 ? 1 : i.getMaterial().index(i));
            double rConst = n1 / n2;
            double radical = 1 - rConst * rConst * (1 - c * c);

            if(debugMode) {
                std::cout << "Ks: (" << mat.ks(i).x << ", " << mat.ks(i).y << ", " << mat.ks(i).z << ")" << std::endl;
                std::cout << "Translucent:  " << (mat.Trans() ? "true" : "false") << std::endl;
                std::cout << "Reflective:  " << (mat.Refl() ? "true" : "false") << std::endl;
                std::cout << "Radical: " << radical << std::endl;
            }


            if (radical >= 0 && mat.Trans()) {
                if(debugMode)
                    std::cout << "NO TOTAL INTERNAL REFRACTION " << std::endl;

                // For exiting rays, c < 0, so we have to use c*-1
                glm::dvec3 altN = n;
                if(c < 0)
                    altN = -altN;

                glm::dvec3 T = glm::normalize(-(rConst * d + sqrt(radical) * altN));
                ray refractedRay(r.getPosition() + (i.t + eps) * r.getDirection(), T, r.getPixel(), r.ctr, r.getAtten(),
                                 ray::REFRACTION);
                refractedColor = traceRay(refractedRay, thresh, depth - 1, t);
            }
        }

		colorC = mat.shade(scene, r, i) + mat.kr(i)*reflectedColor + mat.kt(i)*refractedColor;
	} else {
		// No intersection.  This ray travels to infinity, so we color
		// it according to the background color, which in this (simple) case
		// is just black.
		// 
		// FIXME: Add CubeMap support here.

		colorC = glm::dvec3(0.0, 0.0, 0.0);
	}
	return colorC;
}

RayTracer::RayTracer()
	: scene(0), buffer(0), thresh(0), buffer_width(256), buffer_height(256), m_bBufferReady(false), cubemap (0)
{
}

RayTracer::~RayTracer()
{
	delete scene;
	delete [] buffer;
}

void RayTracer::getBuffer( unsigned char *&buf, int &w, int &h )
{
	buf = buffer;
	w = buffer_width;
	h = buffer_height;
}

double RayTracer::aspectRatio()
{
	return sceneLoaded() ? scene->getCamera().getAspectRatio() : 1;
}

bool RayTracer::loadScene( char* fn ) {
	ifstream ifs( fn );
	if( !ifs ) {
		string msg( "Error: couldn't read scene file " );
		msg.append( fn );
		traceUI->alert( msg );
		return false;
	}
	
	// Strip off filename, leaving only the path:
	string path( fn );
	if( path.find_last_of( "\\/" ) == string::npos ) path = ".";
	else path = path.substr(0, path.find_last_of( "\\/" ));

	// Call this with 'true' for debug output from the tokenizer
	Tokenizer tokenizer( ifs, false );
	Parser parser( tokenizer, path );
	try {
		delete scene;
		scene = 0;
		scene = parser.parseScene();
	} 
	catch( SyntaxErrorException& pe ) {
		traceUI->alert( pe.formattedMessage() );
		return false;
	}
	catch( ParserException& pe ) {
		string msg( "Parser: fatal exception " );
		msg.append( pe.message() );
		traceUI->alert( msg );
		return false;
	}
	catch( TextureMapException e ) {
		string msg( "Texture mapping exception: " );
		msg.append( e.message() );
		traceUI->alert( msg );
		return false;
	}

	if( !sceneLoaded() ) return false;

	return true;
}

void RayTracer::traceSetup(int w, int h)
{
	if (buffer_width != w || buffer_height != h)
	{
		buffer_width = w;
		buffer_height = h;
		bufferSize = buffer_width * buffer_height * 3;
		delete[] buffer;
		buffer = new unsigned char[bufferSize];
        printf("Creating buffer of size %d at %p\n", bufferSize, buffer);
	}
	memset(buffer, 0, w*h*3);
	m_bBufferReady = true;
}

void RayTracer::traceImage(int w, int h, int bs, double thresh)
{
    traceSetup(w, h);

    for(unsigned int x = 0; x < w; x++) {
        for(unsigned int y = 0; y < h; y++) {
            tracePixel(y, x, 0);
        }
    }
}

int RayTracer::aaImage(int samples, double aaThresh)
{
	for(unsigned int x = 0; x < buffer_width; x++) {
		for(unsigned int y = 0; y < buffer_width; y++) {
			glm::dvec2 tl(x - 0.5, y + 0.5);
			glm::dvec2 lr(x + 0.5, y + 0.5);
			glm::dvec2 bl(x - 0.5, y - 0.5);
			glm::dvec2 br(x + 0.5, y - 0.5);
		}
	}
}

bool RayTracer::checkRender()
{
	// FIXME: Return true if tracing is done. (only if multithreading)
	return true;
}

glm::dvec3 RayTracer::getPixel(int i, int j)
{
	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;
	return glm::dvec3((double)pixel[0]/255.0, (double)pixel[1]/255.0, (double)pixel[2]/255.0);
}

void RayTracer::setPixel(int i, int j, glm::dvec3 color)
{
	unsigned char *pixel = buffer + ( i + j * buffer_width ) * 3;

	pixel[0] = (int)( 255.0 * color[0]);
	pixel[1] = (int)( 255.0 * color[1]);
	pixel[2] = (int)( 255.0 * color[2]);
}

