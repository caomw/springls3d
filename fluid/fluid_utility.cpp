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
#define RE			1.4

using namespace std;
namespace imagesci{
namespace fluid{
float hypot2( float a, float b, float c ) {
    return a*a + b*b + c*c;
}

float length2(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).lengthSqr();
}

float length(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
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
float smooth_kernel( float r2, float h ) {
    return fmax( 1.0-r2/(h*h), 0.0 );
}
float sharp_kernel( float r2, float h ) {
    return fmax( h*h/fmax(r2,1.0e-5) - 1.0, 0.0 );
}
double dumptime() {
	static unsigned prevMicroSec = getMicroseconds();
	unsigned curMicroSec = getMicroseconds();
	double res = (curMicroSec - prevMicroSec)/1000000.0;
	prevMicroSec = curMicroSec;
	return res;
}



void mapP2G( sorter *sort, std::vector<ParticlePtr>& particles, MACGrid<float>& grid, int gn ) {

	// Compute Mapping
	OPENMP_FOR FOR_EVERY_CELL(gn+1) {

		// Variales for Particle Sorter
		vector<particle*> neighbors;

		// Map X Grids
		if( j < gn && k < gn) {
			openvdb::Vec3f px(i, j+0.5, k+0.5);
			float sumw = 0.0;
			float sumx = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,1,2,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					float x = fmax(0,fmin(gn,gn*p->p[0]));
					float y = fmax(0,fmin(gn,gn*p->p[1]));
					float z = fmax(0,fmin(gn,gn*p->p[2]));
					float pos[3] = { x, y, z };
					float w = p->m * sharp_kernel(length2(pos,px),RE);
					sumx += w*p->u[0];
					sumw += w;
				}
			}
			grid[0][i][j][k] = sumw ? sumx/sumw : 0.0;
		}

		// Map Y Grids
		if( i < gn && k < gn ) {
			openvdb::Vec3f py( i+0.5, j, k+0.5);
			float sumw = 0.0;
			float sumy = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,2,1,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					float x = fmax(0,fmin(gn,gn*p->p[0]));
					float y = fmax(0,fmin(gn,gn*p->p[1]));
					float z = fmax(0,fmin(gn,gn*p->p[2]));
					float pos[3] = { x, y, z };
					float w = p->m * sharp_kernel(length2(pos,py),RE);
					sumy += w*p->u[1];
					sumw += w;
				}
			}
			grid[1][i][j][k] = sumw ? sumy/sumw : 0.0;
		}

		// Map Z Grids
		if( i < gn && j < gn ) {
			openvdb::Vec3f pz(i+0.5, j+0.5, k);
			float sumw = 0.0;
			float sumz = 0.0;
			neighbors = sort->getNeigboringParticles_wall(i,j,k,2,2,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *p = neighbors[n];
				if( p->type == FLUID ) {
					float x = fmax(0,fmin(gn,gn*p->p[0]));
					float y = fmax(0,fmin(gn,gn*p->p[1]));
					float z = fmax(0,fmin(gn,gn*p->p[2]));
					float pos[3] = { x, y, z };
					float w = p->m * sharp_kernel(length2(pos,pz),RE);
					sumz += w*p->u[2];
					sumw += w;
				}
			}
			grid[2][i][j][k] = sumw ? sumz/sumw : 0.0;
		}
	} END_FOR
}

void mapG2P(std::vector<ParticlePtr>& particles,MACGrid<float>& grid, int gn ) {
	OPENMP_FOR for(ParticlePtr& p:particles){
		fetchVelocity( p->p, p->u, grid, gn );
	} OPENMP_END;
}
float linear ( RegularGrid<float>& q, float x, float y, float z, int w, int h, int d ) {
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

void resample( sorter *sort, openvdb::Vec3f& p,openvdb::Vec3f& u, float re ) {
	// Variables for Neighboring Particles
	std::vector<particle*> neighbors;
	int cell_size = sort->getCellSize();
	float wsum = 0.0;
	openvdb::Vec3f save(u);
	u[0] = u[1] = u[2] = 0.0;

	// Gather Neighboring Particles
	neighbors = sort->getNeigboringParticles_cell(fmax(0,fmin(cell_size-1,cell_size*p[0])),
												  fmax(0,fmin(cell_size-1,cell_size*p[1])),
												  fmax(0,fmin(cell_size-1,cell_size*p[2])),1,1,1);
	for( int n=0; n<neighbors.size(); n++ ) {
		particle *np = neighbors[n];
		if( np->type == FLUID ) {
			float dist2 = length2(p,np->p);
			float w = np->m * sharp_kernel(dist2,re);
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

void correct( sorter *sort, std::vector<ParticlePtr>& particles, float dt, float re ) {
	// Variables for Neighboring Particles
	int cell_size = sort->getCellSize();
	sort->sort(particles);

	// Compute Pseudo Moved Point
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			particle *p = particles[n].get();
			openvdb::Vec3f spring(0.0);
			float x = max(0.0f,min((float)cell_size,cell_size*p->p[0]));
			float y = max(0.0f,min((float)cell_size,cell_size*p->p[1]));
			float z = max(0.0f,min((float)cell_size,cell_size*p->p[2]));
			std::vector<particle*> neighbors = sort->getNeigboringParticles_cell(x,y,z,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *np = neighbors[n];
				if( p != np ) {
					float dist = length(p->p,np->p);
					float w = SPRING * np->m * smooth_kernel(dist*dist,re);
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
			particle *p = particles[n].get();
			p->tmp[1] = p->u;
			resample( sort, p->tmp[0], p->tmp[1], re );

		}
	}

	// Update
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			particle *p = particles[n].get();
			p->p = p->tmp[0];
			p->u = p->tmp[1];
		}
	}
}

static double implicit_func( vector<particle *> &neighbors,openvdb::Vec3f& p, float density, int gn) {
	double phi = 8.0*density/gn;
	for( int m=0; m<neighbors.size(); m++ ) {
		particle &np = *neighbors[m];
		if( np.type == WALL ) {
			if( length(np.p,p) < density/gn ) return 4.5*density/gn;
			continue;
		}
		double d = length(np.p,p);
		if( d < phi ) {
			phi = d;
		}
	}
	return phi - density/gn;
}

double implicit_func( sorter *sort, openvdb::Vec3f& p, float density ) {
	int gn = sort->getCellSize();
	vector<particle *> neighbors = sort->getNeigboringParticles_cell(
			fmax(0,fmin(gn-1,gn*p[0])),
			fmax(0,fmin(gn-1,gn*p[1])),
			fmax(0,fmin(gn-1,gn*p[2])),2,2,2
			);
	return implicit_func( neighbors, p, density, gn );
}
}
}
