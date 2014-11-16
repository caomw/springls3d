/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 *  This implementation of a PIC/FLIP fluid simulator is derived from:
 *
 *  Ando, R., Thurey, N., & Tsuruno, R. (2012). Preserving fluid sheets with adaptively sampled anisotropic particles.
 *  Visualization and Computer Graphics, IEEE Transactions on, 18(8), 1202-1214.
 */
#include "fluid_utility.h"
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#define RE			1.4
#define SPRING		50.0

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

void my_rand_shuffle( std::vector<openvdb::Coord> &waters ) {
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



void mapParticlesToGrid( ParticleLocator *sort, std::vector<ParticlePtr>& particles, MACGrid<float>& grid, int gn ) {

	// Compute Mapping
	OPENMP_FOR FOR_EVERY_CELL(gn+1) {

		// Variales for Particle Sorter
		vector<FluidParticle*> neighbors;

		// Map X Grids
		if( j < gn && k < gn) {
			openvdb::Vec3f px(i, j+0.5, k+0.5);
			float sumw = 0.0;
			float sumx = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,1,2,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(gn*p->mLocation[0],0.0f,(float)gn);
					float y = clamp(gn*p->mLocation[1],0.0f,(float)gn);
					float z = clamp(gn*p->mLocation[2],0.0f,(float)gn);
					openvdb::Vec3f pos(x, y, z);
					float w = p->mMass * sharp_kernel(length2(pos,px),RE);
					sumx += w*p->mVelocity[0];
					sumw += w;
				}
			}
			grid[0](i,j,k) = sumw ? sumx/sumw : 0.0;
		}

		// Map Y Grids
		if( i < gn && k < gn ) {
			openvdb::Vec3f py( i+0.5, j, k+0.5);
			float sumw = 0.0;
			float sumy = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,2,1,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = fmax(0,fmin(gn,gn*p->mLocation[0]));
					float y = fmax(0,fmin(gn,gn*p->mLocation[1]));
					float z = fmax(0,fmin(gn,gn*p->mLocation[2]));
					float pos[3] = { x, y, z };
					float w = p->mMass * sharp_kernel(length2(pos,py),RE);
					sumy += w*p->mVelocity[1];
					sumw += w;
				}
			}
			grid[1](i,j,k) = sumw ? sumy/sumw : 0.0;
		}

		// Map Z Grids
		if( i < gn && j < gn ) {
			openvdb::Vec3f pz(i+0.5, j+0.5, k);
			float sumw = 0.0;
			float sumz = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,2,2,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = fmax(0,fmin(gn,gn*p->mLocation[0]));
					float y = fmax(0,fmin(gn,gn*p->mLocation[1]));
					float z = fmax(0,fmin(gn,gn*p->mLocation[2]));
					float pos[3] = { x, y, z };
					float w = p->mMass * sharp_kernel(length2(pos,pz),RE);
					sumz += w*p->mVelocity[2];
					sumw += w;
				}
			}
			grid[2](i,j,k) = sumw ? sumz/sumw : 0.0;
		}
	} END_FOR
}


float linear ( RegularGrid<float>& q, float x, float y, float z, int w, int h, int d ) {
	x = clamp(x,0.0f,(float)w);
	y = clamp(y,0.0f,(float)h);
	z = clamp(z,0.0f,(float)d);
	int i = min((int)x,w-2);
	int j = min((int)y,h-2);
	int k = min((int)z,d-2);
	return	(k+1-z)*(((i+1-x)*q(i,j,k)+(x-i)*q(i+1,j,k))*(j+1-y) + ((i+1-x)*q(i,j+1,k)+(x-i)*q(i+1,j+1,k))*(y-j)) +
			(z-k)*(((i+1-x)*q(i,j,k+1)+(x-i)*q(i+1,j,k+1))*(j+1-y) + ((i+1-x)*q(i,j+1,k+1)+(x-i)*q(i+1,j+1,k+1))*(y-j));
}

void fetchVelocity(openvdb::Vec3f& p,openvdb::Vec3f& u, MACGrid<float>& grid, int gn ) {
	u[0] = linear( grid[0], gn*p[0], gn*p[1]-0.5, gn*p[2]-0.5, gn+1, gn, gn );
	u[1] = linear( grid[1], gn*p[0]-0.5, gn*p[1], gn*p[2]-0.5, gn, gn+1, gn );
	u[2] = linear( grid[2], gn*p[0]-0.5, gn*p[1]-0.5, gn*p[2], gn, gn, gn+1 );
}

void resampleParticles( ParticleLocator *sort, openvdb::Vec3f& p,openvdb::Vec3f& u, float re ) {
	// Variables for Neighboring Particles
	std::vector<FluidParticle*> neighbors;
	int cell_size = sort->getCellSize();
	float wsum = 0.0;
	openvdb::Vec3f save(u);
	u[0] = u[1] = u[2] = 0.0;

	// Gather Neighboring Particles
	neighbors = sort->getNeigboringCellParticles(fmax(0,fmin(cell_size-1,cell_size*p[0])),
												  fmax(0,fmin(cell_size-1,cell_size*p[1])),
												  fmax(0,fmin(cell_size-1,cell_size*p[2])),1,1,1);
	for( int n=0; n<neighbors.size(); n++ ) {
		FluidParticle *np = neighbors[n];
		if( np->mObjectType == FLUID ) {
			float dist2 = length2(p,np->mLocation);
			float w = np->mMass * sharp_kernel(dist2,re);
			u += w * np->mVelocity;
			wsum += w;
		}
	}
	if( wsum ) {
		u /= wsum;
	} else {
		u = save;
	}
}

void correctParticles( ParticleLocator *sort, std::vector<ParticlePtr>& particles, float dt, float re ) {
	// Variables for Neighboring Particles
	int cell_size = sort->getCellSize();
	sort->update(particles);

	// Compute Pseudo Moved Point
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			openvdb::Vec3f spring(0.0);
			float x = max(0.0f,min((float)cell_size,cell_size*p->mLocation[0]));
			float y = max(0.0f,min((float)cell_size,cell_size*p->mLocation[1]));
			float z = max(0.0f,min((float)cell_size,cell_size*p->mLocation[2]));
			std::vector<FluidParticle*> neighbors = sort->getNeigboringCellParticles(x,y,z,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *np = neighbors[n];
				if( p != np ) {
					float dist = length(p->mLocation,np->mLocation);
					float w = SPRING * np->mMass * smooth_kernel(dist*dist,re);
					if( dist > 0.1*re ) {
						spring += w * (p->mLocation-np->mLocation) / dist * re;
					} else {
						if( np->mObjectType == FLUID ) {
							spring += 0.01*re/dt*(rand()%101)/100.0;
						} else {
							spring += 0.05*re/dt*np->mNormal;
						}
					}
				}
			}
			p->mTmp[0] = p->mLocation + dt*spring;
		}
	}
	// Resample New Velocity
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			p->mTmp[1] = p->mVelocity;
			resampleParticles( sort, p->mTmp[0], p->mTmp[1], re );

		}
	}

	// Update
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			p->mLocation = p->mTmp[0];
			p->mVelocity = p->mTmp[1];
		}
	}
}
void mapGridToParticles(std::vector<ParticlePtr>& particles,MACGrid<float>& grid, int gn ) {
	OPENMP_FOR for(int n=0;n<particles.size();n++){
		ParticlePtr& p=particles[n];
		fetchVelocity( p->mLocation, p->mVelocity, grid, gn );
	}
}
static double implicit_func( vector<ParticlePtr> &neighbors,openvdb::Vec3f& p, float density, int gn) {
	double phi = 8.0*density/gn;
	for( int m=0; m<neighbors.size(); m++ ) {
		FluidParticle &np = *neighbors[m];
		if( np.mObjectType == WALL ) {
			if( length(np.mLocation,p) < density/gn ) return 4.5*density/gn;
			continue;
		}
		double d = length(np.mLocation,p);
		if( d < phi ) {
			phi = d;
		}
	}
	return phi - density/gn;
}
static double implicit_func( vector<FluidParticle*> &neighbors,openvdb::Vec3f& p, float density, int gn) {
	double phi = 8.0*density/gn;
	for( int m=0; m<neighbors.size(); m++ ) {
		FluidParticle &np = *neighbors[m];
		if( np.mObjectType == WALL ) {
			if( length(np.mLocation,p) < density/gn ) return 4.5*density/gn;
			continue;
		}
		double d = length(np.mLocation,p);
		if( d < phi ) {
			phi = d;
		}
	}
	return phi - density/gn;
}
double implicit_func( ParticleLocator *sort, openvdb::Vec3f& p, float density ) {
	int gn = sort->getCellSize();
	vector<FluidParticle *> neighbors = sort->getNeigboringCellParticles(
			clamp((int)(p[0]*gn),0,gn-1),
			clamp((int)(p[1]*gn),0,gn-1),
			clamp((int)(p[2]*gn),0,gn-1),2,2,2
			);
	return implicit_func( neighbors, p, density, gn );
}
}
}
