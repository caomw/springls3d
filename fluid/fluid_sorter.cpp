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
sorter::sorter( int gn ):cells(gn,gn,gn) {
	this->gn = gn;
}
sorter::~sorter() {
}

void sorter::sort( std::vector<particlePtr > &particles ) {
	// Clear All Cells
	FOR_EVERY_CELL(gn) {
		cells[i][j][k].clear();
	} END_FOR
	// Store Into The Cells
	for( int n=0; n<particles.size(); n++ ) { 
		particlePtr p = particles[n];
		FLOAT pos[3];
		for( int k=0; k<3; k++ ) {
            pos[k] = p->p[k];
		}
		int i = fmax(0,fmin(gn-1,gn*pos[0]));
		int j = fmax(0,fmin(gn-1,gn*pos[1]));
		int k = fmax(0,fmin(gn-1,gn*pos[2]));
		cells[i][j][k].push_back(p);


	}
}

std::vector<particlePtr> sorter::getNeigboringParticles_wall( int i, int j, int k, int w, int h, int d ) {
	std::vector<particlePtr> res;
	for( int si=i-w; si<=i+w-1; si++ ) for( int sj=j-h; sj<=j+h-1; sj++ ) for( int sk=k-d; sk<=k+d-1; sk++ ) {
		if( si < 0 || si > gn-1 || sj < 0 || sj > gn-1 || sk < 0 || sk > gn-1 ) continue;
		for( int a=0; a<cells[si][sj][sk].size(); a++ ) { 
			particlePtr p = cells[si][sj][sk][a];
			res.push_back(p);
		}
	}
	return res;
}

std::vector<particlePtr> sorter::getNeigboringParticles_cell( int i, int j, int k, int w, int h, int d ) {
	std::vector<particlePtr> res;
	for( int si=i-w; si<=i+w; si++ ) for( int sj=j-h; sj<=j+h; sj++ ) for( int sk=k-d; sk<=k+d; sk++ ) {
		if( si < 0 || si > gn-1 || sj < 0 || sj > gn-1 || sk < 0 || sk > gn-1 ) continue;
		for( int a=0; a<cells[si][sj][sk].size(); a++ ) { 
			particlePtr p = cells[si][sj][sk][a];
			res.push_back(p);
		}
	}
	return res;
}

int	 sorter::getNumParticleAt( int i, int j, int k ) {
	return cells[i][j][k].size();
}

FLOAT sorter::levelset( int i, int j, int k, FLOAT ***halfwall, FLOAT density ) {
	FLOAT accm = 0.0;
	for( int a=0; a<cells[i][j][k].size(); a++ ) { 
		if( cells[i][j][k][a]->type == FLUID ) {
			accm += cells[i][j][k][a]->dens;
		} else {
			return 1.0;
		}
	}
	FLOAT n0 = 1.0/(density*density*density);
	return 0.2*n0-accm;
}

void sorter::markWater(RegularGrid<char>& A, RegularGrid<float> halfwall, FLOAT density ) {
	FOR_EVERY_CELL(gn) {
		A[i][j][k] = AIR;
		for( int a=0; a<cells[i][j][k].size(); a++ ) { 
			if( cells[i][j][k][a]->type == WALL ) {
				A[i][j][k] = WALL;
			}
		}
		if( A[i][j][k] != WALL ) A[i][j][k] = levelset( i, j, k, halfwall, density ) < 0.0 ? FLUID : AIR;
	} END_FOR
}
void sorter::deleteAllParticles() {
	FOR_EVERY_CELL(gn) {
		for( int a=0; a<cells[i][j][k].size(); a++ ) { 
			cells[i][j][k].clear();
		}
	} END_FOR
}
}}
