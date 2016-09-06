#include <cmath>

#include "Square.h"

using namespace std;


//Test
bool Square::intersectLocal(ray& r, isect& i) const
{
	glm::dvec3 p = r.getPosition();
	glm::dvec3 d = r.getDirection();

	if( d[2] == 0.0 ) {
		return false;
	}

	double t = -p[2]/d[2];

	if( t <= RAY_EPSILON ) {
		return false;
	}

	glm::dvec3 P = r.at( t );

	if( P[0] < -0.5 || P[0] > 0.5 ) {	
		return false;
	}

	if( P[1] < -0.5 || P[1] > 0.5 ) {	
		return false;
	}

	i.obj = this;
	i.setMaterial(this->getMaterial());
	i.t = t;
	if( d[2] > 0.0 ) {
		i.N = glm::dvec3( 0.0, 0.0, -1.0 );
	} else {
		i.N = glm::dvec3( 0.0, 0.0, 1.0 );
	}

	i.setUVCoordinates( glm::dvec2(P[0] + 0.5, P[1] + 0.5) );
	return true;
}
