/*
 *  utility.cpp
 *  flip3D
 *
 */

#include "fluid_utility.h"
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
using namespace std;
namespace imagesci{
namespace fluid{

FLOAT hypot2( FLOAT a, FLOAT b, FLOAT c ) {
    return a*a + b*b + c*c;
}

FLOAT length2(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).lengthSqr();
}

FLOAT length(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).length();
}

void my_rand_shuffle( std::vector<ipos> &waters ) {
	random_shuffle( waters.begin(), waters.end() );
}

unsigned long getMicroseconds() {
	struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec*1000000 + tv.tv_usec;
}
FLOAT smooth_kernel( FLOAT r2, FLOAT h ) {
    return fmax( 1.0-r2/(h*h), 0.0 );
}
FLOAT sharp_kernel( FLOAT r2, FLOAT h ) {
    return fmax( h*h/fmax(r2,1.0e-5) - 1.0, 0.0 );
}
double dumptime() {
	static unsigned prevMicroSec = getMicroseconds();
	unsigned curMicroSec = getMicroseconds();
	double res = (curMicroSec - prevMicroSec)/1000000.0;
	prevMicroSec = curMicroSec;
	return res;
}


#define RE			1.4
#define FOR_EVERY_PARTICLE for( int n=0; n<particles.size(); n++ ) { particle *p = particles[n];

void mapP2G( sorter *sort, vector<particlePtr>& particles, MACGrid<float>& grid, int gn ) {

	// Compute Mapping
	OPENMP_FOR FOR_EVERY_CELL(gn+1) {

		// Variales for Particle Sorter
		vector<particlePtr> neighbors;

		// Map X Grids
		if( j < gn && k < gn) {
			FLOAT px[3] = { i, j+0.5, k+0.5 };
			FLOAT sumw = 0.0;
			FLOAT sumx = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,1,2,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					FLOAT x = fmax(0,fmin(gn,gn*p->p[0]));
					FLOAT y = fmax(0,fmin(gn,gn*p->p[1]));
					FLOAT z = fmax(0,fmin(gn,gn*p->p[2]));
					FLOAT pos[3] = { x, y, z };
					FLOAT w = p->m * sharp_kernel(length2(pos,px),RE);
					sumx += w*p->u[0];
					sumw += w;
				}
			}
			grid[0][i][j][k] = sumw ? sumx/sumw : 0.0;
		}

		// Map Y Grids
		if( i < gn && k < gn ) {
			FLOAT py[3] = { i+0.5, j, k+0.5 };
			FLOAT sumw = 0.0;
			FLOAT sumy = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,2,1,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					FLOAT x = fmax(0,fmin(gn,gn*p->p[0]));
					FLOAT y = fmax(0,fmin(gn,gn*p->p[1]));
					FLOAT z = fmax(0,fmin(gn,gn*p->p[2]));
					FLOAT pos[3] = { x, y, z };
					FLOAT w = p->m * sharp_kernel(length2(pos,py),RE);
					sumy += w*p->u[1];
					sumw += w;
				}
			}
			grid[1][i][j][k] = sumw ? sumy/sumw : 0.0;
		}

		// Map Z Grids
		if( i < gn && j < gn ) {
			FLOAT pz[3] = { i+0.5, j+0.5, k };
			FLOAT sumw = 0.0;
			FLOAT sumz = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,2,2,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					FLOAT x = fmax(0,fmin(gn,gn*p->p[0]));
					FLOAT y = fmax(0,fmin(gn,gn*p->p[1]));
					FLOAT z = fmax(0,fmin(gn,gn*p->p[2]));
					FLOAT pos[3] = { x, y, z };
					FLOAT w = p->m * sharp_kernel(length2(pos,pz),RE);
					sumz += w*p->u[2];
					sumw += w;
				}
			}
			grid[2][i][j][k] = sumw ? sumz/sumw : 0.0;
		}
	} END_FOR
}

void mapG2P(std::vector<particlePtr>& particles,MACGrid<float> grid, int gn ) {
	OPENMP_FOR FOR_EVERY_PARTICLE {
		fetchVelocity( p->p, p->u, grid, gn );
	} END_FOR;
}
FLOAT linear ( RegularGrid<float>& q, FLOAT x, FLOAT y, FLOAT z, int w, int h, int d ) {
	x = fmax(0.0,fmin(w,x));
	y = fmax(0.0,fmin(h,y));
	z = fmax(0.0,fmin(d,z));
	int i = min((int)x,w-2);
	int j = min((int)y,h-2);
	int k = min((int)z,h-2);
	return	(k+1-z)*(((i+1-x)*q[i][j][k]+(x-i)*q[i+1][j][k])*(j+1-y) + ((i+1-x)*q[i][j+1][k]+(x-i)*q[i+1][j+1][k])*(y-j)) +
			(z-k)*(((i+1-x)*q[i][j][k+1]+(x-i)*q[i+1][j][k+1])*(j+1-y) + ((i+1-x)*q[i][j+1][k+1]+(x-i)*q[i+1][j+1][k+1])*(y-j));
}

void fetchVelocity(openvdb::Vec3f& p,openvdb::Vec3f& u, MACGrid<float>& grid, int gn ) {
	u[0] = linear( grid[0], gn*p[0], gn*p[1]-0.5, gn*p[2]-0.5, gn+1, gn, gn );
	u[1] = linear( grid[1], gn*p[0]-0.5, gn*p[1], gn*p[2]-0.5, gn, gn+1, gn );
	u[2] = linear( grid[2], gn*p[0]-0.5, gn*p[1]-0.5, gn*p[2], gn, gn, gn+1 );
}
#define SPRING		50.0

void resample( sorter *sort, openvdb::Vec3f& p,openvdb::Vec3f& u, FLOAT re ) {
	// Variables for Neighboring Particles
	std::vector<particle *> neighbors;
	int cell_size = sort->getCellSize();
	FLOAT wsum = 0.0;
	openvdb::Vec3f save(u);
	u[0] = u[1] = u[2] = 0.0;

	// Gather Neighboring Particles
	neighbors = sort->getNeigboringParticles_cell(fmax(0,fmin(cell_size-1,cell_size*p[0])),
												  fmax(0,fmin(cell_size-1,cell_size*p[1])),
												  fmax(0,fmin(cell_size-1,cell_size*p[2])),1,1,1);
	for( int n=0; n<neighbors.size(); n++ ) {
		particle *np = neighbors[n];
		if( np->type == FLUID ) {
			FLOAT dist2 = length2(p,np->p);
			FLOAT w = np->m * sharp_kernel(dist2,re);
			u += w * np->u;
			wsum += w;
		}
	}
	if( wsum ) {
		u /= wsum;
	} else {
		u = save;
	}
}

void correct( sorter *sort, std::vector<particlePtr>& particles, FLOAT dt, FLOAT re ) {
	// Variables for Neighboring Particles
	int cell_size = sort->getCellSize();
	sort->sort(particles);

	// Compute Pseudo Moved Point
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			particle *p = particles[n];
			openvdb::Vec3f spring(0.0);
			FLOAT x = max(0.0f,min((float)cell_size,cell_size*p->p[0]));
			FLOAT y = max(0.0f,min((float)cell_size,cell_size*p->p[1]));
			FLOAT z = max(0.0f,min((float)cell_size,cell_size*p->p[2]));
			std::vector<particle *> neighbors = sort->getNeigboringParticles_cell(x,y,z,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *np = neighbors[n];
				if( p != np ) {
					FLOAT dist = length(p->p,np->p);
					FLOAT w = SPRING * np->m * smooth_kernel(dist*dist,re);
					if( dist > 0.1*re ) {
						spring += w * (p->p-np->p) / dist * re;
					} else {
						if( np->type == FLUID ) {
							spring += 0.01*re/dt*(rand()%101)/100.0;
						} else {
							spring += 0.05*re/dt*np->n;
						}
					}
				}
			}
			p->tmp[0] = p->p + dt*spring;
		}
	}
	// Resample New Velocity
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			particle *p = particles[n];
			p->tmp[1] = p->u;
			resample( sort, p->tmp[0], p->tmp[1], re );

		}
	}

	// Update
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			particle *p = particles[n];
			p->p = p->tmp[0];
			p->u = p->tmp[1];
		}
	}
}


}
}
