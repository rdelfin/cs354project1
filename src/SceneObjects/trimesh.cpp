#include <cmath>
#include <float.h>
#include <algorithm>
#include <assert.h>
#include <string.h>
#include <iostream>
#include "trimesh.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

using namespace std;

Trimesh::~Trimesh()
{
	for( Materials::iterator i = materials.begin(); i != materials.end(); ++i )
		delete *i;
}

// must add vertices, normals, and materials IN ORDER
void Trimesh::addVertex( const glm::dvec3 &v )
{
    vertices.push_back( v );
}

void Trimesh::addMaterial( Material *m )
{
    materials.push_back( m );
}

void Trimesh::addNormal( const glm::dvec3 &n )
{
    normals.push_back( n );
}

// Returns false if the vertices a,b,c don't all exist
bool Trimesh::addFace( int a, int b, int c )
{
    int vcnt = vertices.size();

    if( a >= vcnt || b >= vcnt || c >= vcnt ) return false;

    TrimeshFace *newFace = new TrimeshFace( scene, new Material(*this->material), this, a, b, c );
    newFace->setTransform(this->transform);
    if (!newFace->degen) faces.push_back( newFace );


    // Don't add faces to the scene's object list so we can cull by bounding box
    // scene->add(newFace);
    return true;
}

const char* Trimesh::doubleCheck()
// Check to make sure that if we have per-vertex materials or normals
// they are the right number.
{
    if( !materials.empty() && materials.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of materials.";
    if( !normals.empty() && normals.size() != vertices.size() )
        return "Bad Trimesh: Wrong number of normals.";

    return 0;
}

bool Trimesh::intersectLocal(ray& r, isect& i) const
{
	typedef Faces::const_iterator iter;
	bool have_one = false;
	for( iter j = faces.begin(); j != faces.end(); ++j )
	{
		isect cur;
		if( (*j)->intersectLocal( r, cur ) )
		{
			if( !have_one || (cur.t < i.t) )
			{
				i = cur;
				have_one = true;
			}
		}
	}
	if( !have_one ) i.setT(1000.0);
	return have_one;
} 

bool TrimeshFace::intersect(ray& r, isect& i) const {
  return intersectLocal(r, i);
}

// Intersect ray r with the triangle abc.  If it hits returns true,
// and put the parameter in t and the barycentric coordinates of the
// intersection in u (alpha) and v (beta).
bool TrimeshFace::intersectLocal(ray& r, isect& i) const
{
    // Calculate points A, B and C
    glm::dvec3 a = parent->vertices[ids[0]];
    glm::dvec3 b = parent->vertices[ids[1]];
    glm::dvec3 c = parent->vertices[ids[2]];

    // Special case when ray and plane are parallel.
    if(glm::dot(normal, r.d) == 0)
        return false;

    // For our ray, p(t) = (P + td), we can solve for t and get: t = -(n*P * d)/(n*d)
    double t = -(glm::dot(normal, r.p) + dist)/(glm::dot(normal, r.d));

    glm::dvec3 p = r.p + i.t*r.d; // Value of p(i.t)

    glm::dvec3 u = c - a;
    glm::dvec3 v = c - b;

    // Cramer's rule solution set:
    glm::dmat3x3 denominator = {{a.x,b.x,c.x},{a.y,b.y,c.y},{1,1,1}};
    glm::dmat3x3 alphaNumerator = denominator;
    glm::dmat3x3 betaNumerator = denominator;

    alphaNumerator[0][0] = betaNumerator[0][1] = p.x;
    alphaNumerator[1][0] = betaNumerator[1][1] = p.y;

    double alpha = glm::determinant(alphaNumerator)/glm::determinant(denominator);
    double beta = glm::determinant(betaNumerator)/glm::determinant(denominator);

    bool intersects = t >=0 && alpha >= 0 && alpha <= 1 && beta >=0 && beta <= 1;

    std::cout << "T: " << t << ", alpha: " << alpha << ", beta: " << beta << std::endl;
    std::cout << "r.P: (" << r.p.x << ", " << r.p.y << ", " << r.p.z << ")" << "r.d: (" << r.d.x << ", " << r.d.y << ", " << r.d.z << ")" << std::endl;
    std::cout << "a: (" << a.x << ", " << a.y << ", " << a.z << "), b: (" << b.x << ", " << b.y << ", " << b.z << "), c: (" << c.x << ", " << c.y << ", " << c.z << ")" << ", n: (" << normal.x << ", " << normal.y << ", " << normal.z << ")" << std::endl;

    if(intersects) {
        i.t = t;
        i.uvCoordinates.x = alpha;
        i.uvCoordinates.y = beta;
        i.material = new Material(*material);
    }

    return intersects;
}

void Trimesh::generateNormals()
// Once you've loaded all the verts and faces, we can generate per
// vertex normals by averaging the normals of the neighboring faces.
{
    int cnt = vertices.size();
    normals.resize( cnt );
    int *numFaces = new int[ cnt ]; // the number of faces assoc. with each vertex
    memset( numFaces, 0, sizeof(int)*cnt );
    
    for( Faces::iterator fi = faces.begin(); fi != faces.end(); ++fi )
    {
		glm::dvec3 faceNormal = (**fi).getNormal();
        
        for( int i = 0; i < 3; ++i )
        {
            normals[(**fi)[i]] += faceNormal;
            ++numFaces[(**fi)[i]];
        }
    }

    for( int i = 0; i < cnt; ++i )
    {
        if( numFaces[i] )
            normals[i]  /= numFaces[i];
    }

    delete [] numFaces;
    vertNorms = true;
}

