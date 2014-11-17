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

const float RELAXATION_KERNEL_WIDTH=1.4;
const float SPRING_CONSTANT=50.0;

using namespace std;

namespace imagesci{
namespace fluid{
float LengthSquared( float a, float b, float c ) {
    return a*a + b*b + c*c;
}

float DistanceSquared(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).lengthSqr();
}

float Distance(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).length();
}

void ShuffleCoordinates( std::vector<openvdb::Coord> &waters ) {
	random_shuffle( waters.begin(), waters.end() );
}


float SmoothKernel( float r2, float h ) {
    return max( 1.0-r2/(h*h), 0.0 );
}
float SharpKernel( float r2, float h ) {
    return max( h*h/fmax(r2,1.0e-5) - 1.0, 0.0 );
}




void MapParticlesToGrid( ParticleLocator *sort, std::vector<ParticlePtr>& particles, MACGrid<float>& grid) {

	// Compute Mapping
	openvdb::Coord dims(grid.rows(),grid.cols(),grid.slices());
	OPENMP_FOR FOR_EVERY_CELL(dims[0]+1,dims[1]+1,dims[2]+1) {
		// Variales for Particle Sorter
		vector<FluidParticle*> neighbors;

		// Map X Grids
		if( j <dims[1] && k < dims[2]) {
			openvdb::Vec3f px(i, j+0.5, k+0.5);
			float sumw = 0.0;
			float sumx = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,1,2,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					openvdb::Vec3f pos(x, y, z);
					float w = p->mMass * SharpKernel(DistanceSquared(pos,px),RELAXATION_KERNEL_WIDTH);
					sumx += w*p->mVelocity[0];
					sumw += w;
				}
			}
			grid[0](i,j,k) = sumw ? sumx/sumw : 0.0;
		}

		// Map Y Grids
		if( i < dims[0] && k < dims[2] ) {
			openvdb::Vec3f py( i+0.5, j, k+0.5);
			float sumw = 0.0;
			float sumy = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,2,1,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					float pos[3] = { x, y, z };
					float w = p->mMass * SharpKernel(DistanceSquared(pos,py),RELAXATION_KERNEL_WIDTH);
					sumy += w*p->mVelocity[1];
					sumw += w;
				}
			}
			grid[1](i,j,k) = sumw ? sumy/sumw : 0.0;
		}

		// Map Z Grids
		if( i < dims[0] && j < dims[1] ) {
			openvdb::Vec3f pz(i+0.5, j+0.5, k);
			float sumw = 0.0;
			float sumz = 0.0;
			neighbors = sort->getNeigboringWallParticles(i,j,k,2,2,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					float pos[3] = { x, y, z };
					float w = p->mMass * SharpKernel(DistanceSquared(pos,pz),RELAXATION_KERNEL_WIDTH);
					sumz += w*p->mVelocity[2];
					sumw += w;
				}
			}
			grid[2](i,j,k) = sumw ? sumz/sumw : 0.0;
		}
	} END_FOR
}

float linear ( RegularGrid<float>& q, float x, float y, float z) {
	 int w=q.rows();
	 int h=q.cols();
	 int d=q.slices();
	x = clamp(x,0.0f,(float)w);
	y = clamp(y,0.0f,(float)h);
	z = clamp(z,0.0f,(float)d);
	int i = min((int)x,w-2);
	int j = min((int)y,h-2);
	int k = min((int)z,d-2);
	return	(k+1-z)*(((i+1-x)*q(i,j,k)+(x-i)*q(i+1,j,k))*(j+1-y) + ((i+1-x)*q(i,j+1,k)+(x-i)*q(i+1,j+1,k))*(y-j)) +
			(z-k)*(((i+1-x)*q(i,j,k+1)+(x-i)*q(i+1,j,k+1))*(j+1-y) + ((i+1-x)*q(i,j+1,k+1)+(x-i)*q(i+1,j+1,k+1))*(y-j));
}
void fetchVelocity(openvdb::Vec3f& p,openvdb::Vec3f& u, MACGrid<float>& grid) {

	u[0] = linear( grid[0], grid.rows()*p[0], grid.cols()*p[1]-0.5, grid.slices()*p[2]-0.5);
	u[1] = linear( grid[1], grid.rows()*p[0]-0.5, grid.cols()*p[1], grid.slices()*p[2]-0.5);
	u[2] = linear( grid[2], grid.rows()*p[0]-0.5, grid.cols()*p[1]-0.5, grid.slices()*p[2]);
}

void resampleParticles( ParticleLocator *sort, openvdb::Vec3f& p,openvdb::Vec3f& u, float re ) {
	// Variables for Neighboring Particles
	std::vector<FluidParticle*> neighbors;
	openvdb::Coord cell_size = sort->getCellSize();
	float wsum = 0.0;
	openvdb::Vec3f save(u);
	u[0] = u[1] = u[2] = 0.0;
	int i = clamp((int)(p[0]*cell_size[0]),0,cell_size[0]-1);
	int j = clamp((int)(p[1]*cell_size[1]),0,cell_size[1]-1);
	int k = clamp((int)(p[2]*cell_size[2]),0,cell_size[2]-1);
	// Gather Neighboring Particles
	neighbors = sort->getNeigboringCellParticles(i,j,k,1,1,1);
	for(FluidParticle *np:neighbors) {
		if( np->mObjectType == FLUID ) {
			float dist2 = DistanceSquared(p,np->mLocation);
			float w = np->mMass * SharpKernel(dist2,re);
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
	openvdb::Coord cell_size = sort->getCellSize();
	sort->update(particles);

	// Compute Pseudo Moved Point
	OPENMP_FOR FOR_EVERY_PARTICLE(particles) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			openvdb::Vec3f spring(0.0);
			int i = clamp((int)(p->mLocation[0]*cell_size[0]),0,cell_size[0]-1);
			int j = clamp((int)(p->mLocation[1]*cell_size[1]),0,cell_size[1]-1);
			int k = clamp((int)(p->mLocation[2]*cell_size[2]),0,cell_size[2]-1);
			std::vector<FluidParticle*> neighbors = sort->getNeigboringCellParticles(i,j,k,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *np = neighbors[n];
				if( p != np ) {
					float dist = Distance(p->mLocation,np->mLocation);
					float w = SPRING_CONSTANT * np->mMass * SmoothKernel(dist*dist,re);
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
	OPENMP_FOR FOR_EVERY_PARTICLE(particles) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			p->mTmp[1] = p->mVelocity;
			resampleParticles( sort, p->mTmp[0], p->mTmp[1], re );

		}
	}

	// Update
	OPENMP_FOR FOR_EVERY_PARTICLE(particles) {
		if( particles[n]->mObjectType == FLUID ) {
			FluidParticle *p = particles[n].get();
			p->mLocation = p->mTmp[0];
			p->mVelocity = p->mTmp[1];
		}
	}
}
void MapGridToParticles(std::vector<ParticlePtr>& particles,MACGrid<float>& grid) {
	OPENMP_FOR FOR_EVERY_PARTICLE(particles){
		ParticlePtr& p=particles[n];
		fetchVelocity( p->mLocation, p->mVelocity, grid);
	}
}
static double implicit_func( vector<FluidParticle*> &neighbors,openvdb::Vec3f& p, float density,float voxelSize) {
	double phi = 8.0*density*voxelSize;
	for( int m=0; m<neighbors.size(); m++ ) {
		FluidParticle &np = *neighbors[m];
		if( np.mObjectType == WALL ) {
			if( Distance(np.mLocation,p) < density*voxelSize) return 4.5*density*voxelSize;
			continue;
		}
		double d = Distance(np.mLocation,p);
		if( d < phi ) {
			phi = d;
		}
	}
	return phi - density*voxelSize;
}
double implicit_func( ParticleLocator *sort, openvdb::Vec3f& p, float density ) {
	openvdb::Coord cell_size = sort->getCellSize();
	vector<FluidParticle *> neighbors = sort->getNeigboringCellParticles(
			clamp((int)(p[0]*cell_size[0]),0,cell_size[0]-1),
			clamp((int)(p[1]*cell_size[1]),0,cell_size[1]-1),
			clamp((int)(p[2]*cell_size[2]),0,cell_size[2]-1),2,2,2
			);
	return implicit_func( neighbors, p, density,sort->getVoxelSize());
}
}
}
