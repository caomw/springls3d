/*
 *  sorter.cpp
 *  flip3D
 */

#include "fluid_sorter.h"
#include "fluid_utility.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace std;
namespace imagesci{
namespace fluid{
ParticleLocator::ParticleLocator( int gn ):cells(gn,gn,gn) {
	this->mGridSize = gn;
}
ParticleLocator::~ParticleLocator() {
}

void ParticleLocator::update( std::vector<ParticlePtr >& particles ) {
	// Clear All Cells
	FOR_EVERY_CELL(mGridSize) {
		cells[i][j][k].clear();
	} END_FOR
	// Store Into The Cells
	for(ParticlePtr& p:particles) {
		int i = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[0]));
		int j = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[1]));
		int k = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[2]));
		cells[i][j][k].push_back(p);
	}
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringWallParticles( int i, int j, int k, int w, int h, int d ) {
	std::vector<FluidParticle*> res;
	for( int si=i-w; si<=i+w-1; si++ ) for( int sj=j-h; sj<=j+h-1; sj++ ) for( int sk=k-d; sk<=k+d-1; sk++ ) {
		if( si < 0 || si > mGridSize-1 || sj < 0 || sj > mGridSize-1 || sk < 0 || sk > mGridSize-1 ) continue;
		for( int a=0; a<cells[si][sj][sk].size(); a++ ) { 
			ParticlePtr p = cells[si][sj][sk][a];
			res.push_back(p.get());
		}
	}
	return res;
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringCellParticles( int i, int j, int k, int w, int h, int d ) {
	std::vector<FluidParticle*> res;
	for( int si=i-w; si<=i+w; si++ ) for( int sj=j-h; sj<=j+h; sj++ ) for( int sk=k-d; sk<=k+d; sk++ ) {
		if( si < 0 || si > mGridSize-1 || sj < 0 || sj > mGridSize-1 || sk < 0 || sk > mGridSize-1 ) continue;
		for( int a=0; a<cells[si][sj][sk].size(); a++ ) { 
			ParticlePtr p = cells[si][sj][sk][a];
			res.push_back(p.get());
		}
	}
	return res;
}

int	 ParticleLocator::getParticleCount( int i, int j, int k ) {
	return cells[i][j][k].size();
}

float ParticleLocator::getLevelSetValue( int i, int j, int k, RegularGrid<float>& halfwall, float density ) {
	float accm = 0.0;
	for( int a=0; a<cells[i][j][k].size(); a++ ) { 
		if( cells[i][j][k][a]->mObjectType == FLUID ) {
			accm += cells[i][j][k][a]->mDensity;
		} else {
			return 1.0;
		}
	}
	float n0 = 1.0/(density*density*density);
	return 0.2*n0-accm;
}

void ParticleLocator::markAsWater(RegularGrid<char>& A, RegularGrid<float>& halfwall, float density ) {
	FOR_EVERY_CELL(mGridSize) {
		A[i][j][k] = AIR;
		for( int a=0; a<cells[i][j][k].size(); a++ ) { 
			if( cells[i][j][k][a]->mObjectType == WALL ) {
				A[i][j][k] = WALL;
			}
		}
		if( A[i][j][k] != WALL ) A[i][j][k] = getLevelSetValue( i, j, k, halfwall, density ) < 0.0 ? FLUID : AIR;
	} END_FOR
}
void ParticleLocator::deleteAllParticles() {
	FOR_EVERY_CELL(mGridSize) {
		for( int a=0; a<cells[i][j][k].size(); a++ ) { 
			cells[i][j][k].clear();
		}
	} END_FOR
}
}}
