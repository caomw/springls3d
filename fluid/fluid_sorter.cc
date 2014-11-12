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
		cells(i,j,k).clear();
	} END_FOR
	// Store Into The Cells
	for(ParticlePtr& p:particles) {
		int i = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[0]));
		int j = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[1]));
		int k = fmax(0,fmin(mGridSize-1,mGridSize*p->mLocation[2]));
		cells(i,j,k).push_back(p);
	}
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringWallParticles( int i, int j, int k, int w, int h, int d ) {
	std::vector<FluidParticle*> res;
	for( int si=i-w; si<=i+w-1; si++ ) for( int sj=j-h; sj<=j+h-1; sj++ ) for( int sk=k-d; sk<=k+d-1; sk++ ) {
		if( si < 0 || si > mGridSize-1 || sj < 0 || sj > mGridSize-1 || sk < 0 || sk > mGridSize-1 ) continue;
		for( int a=0; a<cells(si,sj,sk).size(); a++ ) {
			ParticlePtr p = cells(si,sj,sk)[a];
			res.push_back(p.get());
		}
	}
	return res;
}

std::vector<FluidParticle*> ParticleLocator::getNeigboringCellParticles( int i, int j, int k, int w, int h, int d ) {
	std::vector<FluidParticle*> res;
	for( int si=i-w; si<=i+w; si++ ) for( int sj=j-h; sj<=j+h; sj++ ) for( int sk=k-d; sk<=k+d; sk++ ) {
		if( si < 0 || si > mGridSize-1 || sj < 0 || sj > mGridSize-1 || sk < 0 || sk > mGridSize-1 ) continue;
		for( int a=0; a<cells(si,sj,sk).size(); a++ ) {
			ParticlePtr p = cells(si,sj,sk)[a];
			res.push_back(p.get());
		}
	}
	return res;
}

int	 ParticleLocator::getParticleCount( int i, int j, int k ) {
	return cells(i,j,k).size();
}

float ParticleLocator::getLevelSetValue( int i, int j, int k, RegularGrid<float>& halfwall, float density ) {
	float accm = 0.0;
	for( int a=0; a<cells(i,j,k).size(); a++ ) {
		if( cells(i,j,k)[a]->mObjectType == FLUID ) {
			accm += cells(i,j,k)[a]->mDensity;
		} else {
			return 1.0;
		}
	}
	float n0 = 1.0/(density*density*density);
	return 0.2*n0-accm;
}

void ParticleLocator::markAsWater(RegularGrid<char>& A, RegularGrid<float>& halfwall, float density ) {
	FOR_EVERY_CELL(mGridSize) {
		A(i,j,k) = AIR;
		for( int a=0; a<cells(i,j,k).size(); a++ ) {
			if( cells(i,j,k)[a]->mObjectType == WALL ) {
				A(i,j,k) = WALL;
			}
		}
		if( A(i,j,k) != WALL ) A(i,j,k) = getLevelSetValue( i, j, k, halfwall, density ) < 0.0 ? FLUID : AIR;
	} END_FOR
}
void ParticleLocator::deleteAllParticles() {
	FOR_EVERY_CELL(mGridSize) {
		for( int a=0; a<cells(i,j,k).size(); a++ ) {
			cells(i,j,k).clear();
		}
	} END_FOR
}
}}
