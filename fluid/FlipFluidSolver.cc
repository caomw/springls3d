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
 */

#include "FlipFluidSolver.h"
#include "fluid_utility.h"
#include <openvdb/openvdb.h>
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
namespace fluid {
const float FlipFluidSolver::ALPHA           =0.95f;
const float FlipFluidSolver::DT              =0.6e-2f;
const float FlipFluidSolver::DENSITY         =0.5f;
const float FlipFluidSolver::GRAVITY        = 9.8f;
FlipFluidSolver::FlipFluidSolver(int gridSize):
		mGridSize(gridSize),
		mA(Coord(gridSize),Coord(0),0.0f),
		mL(Coord(gridSize),Coord(0),0.0f),
		mPress(Coord(gridSize),Coord(0),0.0f),
		mVol(Coord(gridSize),Coord(0),0.0f),
		mVolSave(Coord(gridSize),Coord(0),0.0f),
		mWallNormal(Coord(gridSize),Coord(0),openvdb::Vec4f(0.0f)),
		max_dens(1.0){
	WALL_THICK=1.0f/gridSize;
}
void FlipFluidSolver::computeDensity() {
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {

		// Find Neighbors
		int gn = sort->getCellSize();
		if( particles[n]->type == WALL ) {
			particles[n]->dens = 1.0;
			continue;
		}

		Vec3f& p = particles[n]->p;
		std::vector<particlePtr> neighbors = sort->getNeigboringParticles_cell(fmax(0,fmin(gn-1,gn*p[0])),
																		 fmax(0,fmin(gn-1,gn*p[1])),
																		 fmax(0,fmin(gn-1,gn*p[2])),1,1,1);
		FLOAT wsum = 0.0;
		for( int m=0; m<neighbors.size(); m++ ) {
			particle np = *neighbors[m];
			if( np.type == WALL ) continue;
			FLOAT d2 = length2(np.p,p);
			FLOAT w = np.m*smooth_kernel(d2, 4.0f*DENSITY/mGridSize);
			wsum += w;
		}
		particles[n]->dens = wsum / max_dens;
	}
}
void FlipFluidSolver::placeWalls() {
	Object obj;

	// Left Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;			obj.p[1][0] = WALL_THICK;
	obj.p[0][1] = 0.0;			obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;			obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Right Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 1.0-WALL_THICK;	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;				obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;				obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Floor Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GRAY;
	obj.visible = 0;
	obj.p[0][0] = 0.0;	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;	obj.p[1][1] = WALL_THICK;
	obj.p[0][2] = 0.0;	obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Ceiling Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;				obj.p[1][0] = 1.0;
	obj.p[0][1] = 1.0-WALL_THICK;	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;				obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Front Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;	obj.p[1][2] = WALL_THICK;
	objects.push_back(obj);

	// Back Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;				obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;				obj.p[1][1] = 1.0;
	obj.p[0][2] = 1.0-WALL_THICK;	obj.p[1][2] = 1.0;
	objects.push_back(obj);
}
void FlipFluidSolver::damBreakTest() {
	Object obj;

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = 0.2;	obj.p[1][0] = 0.4;
	obj.p[0][1] = WALL_THICK;	obj.p[1][1] = 0.4;
	obj.p[0][2] = 0.2;	obj.p[1][2] = 0.8;

	objects.push_back(obj);

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = WALL_THICK;	obj.p[1][0] = 1.0-WALL_THICK;
	obj.p[0][1] = WALL_THICK;	obj.p[1][1] = 0.06;
	obj.p[0][2] = WALL_THICK;	obj.p[1][2] = 1.0-WALL_THICK;

	objects.push_back(obj);
}
void  FlipFluidSolver::placeObjects() {
	// Place Object Wall
	placeWalls();

	// profileTest();
    // waterDropTest();
	// cliffPourTest();
	damBreakTest();
	// spherePourTest();
}
void FlipFluidSolver::init(){
		FOR_EVERY_X_FLOW(mGridSize) {
			mVolSave[0][i][j][k] = mVol[0][i][j][k] = 0.0;
		} END_FOR

		FOR_EVERY_Y_FLOW(mGridSize) {
			mVolSave[1][i][j][k] = mVol[1][i][j][k] = 0.0;
		} END_FOR

		FOR_EVERY_Z_FLOW(mGridSize) {
			mVolSave[2][i][j][k] = mVol[2][i][j][k] = 0.0;
		} END_FOR

		FOR_EVERY_CELL(mGridSize) {
			mA(i,j,k) = AIR;
	        mPress(i,j,k) = 0.0;
		} END_FOR

		sort = std::unique_ptr<sorter>(new sorter(mGridSize));

		placeObjects();

			// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
			FLOAT h = DENSITY/mGridSize;
			FOR_EVERY_CELL(10) {
				particle *p = new particle;
				p->p[0] = (i+0.5)*h;
				p->p[1] = (j+0.5)*h;
				p->p[2] = (k+0.5)*h;
				p->type = FLUID;
				p->m = 1.0;
				particles.push_back(std::unique_ptr<particle>(p));
			} END_FOR

			sort->sort(particles);
			max_dens = 1.0;


			/*
			computeDensity();
			max_dens = 0.0;
			for( int n=0; n<particles.size(); n++ ) {
				particle *p = particles[n];
				max_dens = fmax(max_dens,p->dens);
				delete p;
			}
			particles.clear();

			// Place Fluid Particles And Walls
			double w = DENSITY*WALL_THICK;
			for( int i=0; i < N/DENSITY; i++ ) {
				for( int j=0; j < N/DENSITY; j++ ) {
					for( int k=0; k < N/DENSITY; k++ ) {
						double x = i*w+w/2.0;
						double y = j*w+w/2.0;
						double z = k*w+w/2.0;

						if( x > WALL_THICK && x < 1.0-WALL_THICK &&
							y > WALL_THICK && y < 1.0-WALL_THICK &&
							z > WALL_THICK && z < 1.0-WALL_THICK ) {
								pushParticle( x, y, z, FLUID );
						}
					}
				}
			}

			// Place Wall Particles And Walls
			w = 1.0/N;
			for( int i=0; i < N; i++ ) {
				for( int j=0; j < N; j++ ) {
					for( int k=0; k < N; k++ ) {
						double x = i*w+w/2.0;
						double y = j*w+w/2.0;
						double z = k*w+w/2.0;
						pushParticle( x, y, z, WALL );
					}
				}
			}

			// Remove Particles That Stuck On Wal Cells
			sort->sort(particles);
			sort->markWater(A,wall_normal[3],DENSITY);

			for( vector<particle *>::iterator iter=particles.begin(); iter!=particles.end(); ) {
				particle &p = **iter;
				if( p.type == WALL ) {
					iter++;
					continue;
				}
				int i = fmin(N-1,fmax(0,p.p[0]*N));
				int j = fmin(N-1,fmax(0,p.p[1]*N));
				int k = fmin(N-1,fmax(0,p.p[2]*N));
				if( A[i][j][k] == WALL ) {
					delete *iter;
					iter = particles.erase(iter);
				} else {
					iter ++;
				}
			// Comput Normal for Walls
			compute_wall_normal();

*/
}
FlipFluidSolver::~FlipFluidSolver() {
	// TODO Auto-generated destructor stub
}
}
} /* namespace imagesci */
