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

#include "FluidSimulation.h"
#include "fluid_utility.h"
#include "laplace_solver.h"
#include <openvdb/openvdb.h>
using namespace openvdb;
using namespace openvdb::tools;
using namespace std;
namespace imagesci {
namespace fluid {
const float FluidSimulation::mPicFlipBlendWeight = 0.95f;
const float FluidSimulation::mTimeStep = 0.6e-2f;
const float FluidSimulation::mDensityIsoLevel = 0.5f;
const float FluidSimulation::GRAVITY = 9.8f;
FluidSimulation::FluidSimulation(int gridSize,MotionScheme scheme) :
		Simulation("Fluid Simulation",scheme),
		mStuckParticleCount(0),
		pourTime(-1),
		mPourPosition(0.0),
		mPourRadius(0.12),
		mGridSize(gridSize),
		mLevelSet(Coord(gridSize),Coord(0),0),
		mLabel(Coord(gridSize), Coord(0), 0.0f),mDivergence(Coord(gridSize), Coord(0), 0.0f), mLaplacian(
				Coord(gridSize), Coord(0), 0.0f), mPressure(Coord(gridSize),
				Coord(0), 0.0f), mVelocity(Coord(gridSize), Coord(0), 0.0f), mVelocityLast(
				Coord(gridSize), Coord(0), 0.0f), mWallWeight(Coord(gridSize),
				Coord(0), 0.0f), mWallNormal(Coord(gridSize), Coord(0),
				openvdb::Vec3f(0.0f)), mMaxDensity(1.0) {
	mWallThickness = 1.0f / gridSize;
}
void FluidSimulation::computeDensity() {
	OPENMP_FOR for (int n = 0; n < mParticles.size(); n++) {

		// Find Neighbors
		int gn = mSorter->getCellSize();
		if (mParticles[n]->type == WALL) {
			mParticles[n]->dens = 1.0;
			continue;
		}

		Vec3f& p = mParticles[n]->p;
		std::vector<particle*> neighbors = mSorter->getNeigboringParticles_cell(
				fmax(0, fmin(gn - 1, gn * p[0])),
				fmax(0, fmin(gn - 1, gn * p[1])),
				fmax(0, fmin(gn - 1, gn * p[2])), 1, 1, 1);
		float wsum = 0.0;
		for (int m = 0; m < neighbors.size(); m++) {
			particle& np = *neighbors[m];
			if (np.type == WALL)
				continue;
			float d2 = length2(np.p, p);
			float w = np.m * smooth_kernel(d2, 4.0f * mDensityIsoLevel / mGridSize);
			wsum += w;
		}
		mParticles[n]->dens = wsum / mMaxDensity;
	}
}
void FluidSimulation::placeWalls() {
	Object obj;

	// Left Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = mWallThickness;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	mSimulationObjects.push_back(obj);

	// Right Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 1.0 - mWallThickness;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	mSimulationObjects.push_back(obj);

	// Floor Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GRAY;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = mWallThickness;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	mSimulationObjects.push_back(obj);

	// Ceiling Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 1.0 - mWallThickness;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	mSimulationObjects.push_back(obj);

	// Front Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = mWallThickness;
	mSimulationObjects.push_back(obj);

	// Back Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 1.0 - mWallThickness;
	obj.p[1][2] = 1.0;
	mSimulationObjects.push_back(obj);
}
void FluidSimulation::damBreakTest() {
	Object obj;

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = 0.2;
	obj.p[1][0] = 0.4;
	obj.p[0][1] = mWallThickness;
	obj.p[1][1] = 0.4;
	obj.p[0][2] = 0.2;
	obj.p[1][2] = 0.8;

	mSimulationObjects.push_back(obj);

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = mWallThickness;
	obj.p[1][0] = 1.0 - mWallThickness;
	obj.p[0][1] = mWallThickness;
	obj.p[1][1] = 0.06;
	obj.p[0][2] = mWallThickness;
	obj.p[1][2] = 1.0 - mWallThickness;

	mSimulationObjects.push_back(obj);
}
void FluidSimulation::placeObjects() {
	// Place Object Wall
	placeWalls();

	// profileTest();
	// waterDropTest();
	// cliffPourTest();
	damBreakTest();
	// spherePourTest();
}
void FluidSimulation::reposition(vector<int>& indices, vector<ParticlePtr> particles ) {
	if( indices.empty() ) return;
	int gn = mSorter->getCellSize();

	// First Search for Deep Water
	vector<ipos> waters;
	while( waters.size() < indices.size() ) {
		FOR_EVERY_CELL(gn) {
			if( i > 0 && mLabel[i-1][j][k] != FLUID ) continue;
			if( i < mGridSize-1 && mLabel[i+1][j][k] != FLUID ) continue;
			if( j > 0 && mLabel[i][j-1][k] != FLUID ) continue;
			if( j < mGridSize-1 && mLabel[i][j+1][k] != FLUID ) continue;
			if( k > 0 && mLabel[i][j][k-1] != FLUID ) continue;
			if( k < mGridSize-1 && mLabel[i][j][k+1] != FLUID ) continue;
			if( mLabel[i][j][k] != FLUID ) continue;

			ipos aPos = { i, j, k };
			waters.push_back(aPos);
			if( waters.size() >= indices.size() ) {
				i = mGridSize; j = mGridSize; k = mGridSize;
			}
		} END_FOR;
		if( waters.empty() ) return;
	}

	// Shuffle
	my_rand_shuffle(waters);

	float h = 1.0/gn;
	for( int n=0; n<indices.size(); n++ ) {
		particle &p = *particles[indices[n]];
		p.p[0] = h*(waters[n].i+0.25+0.5*(rand()%101)/100);
		p.p[1] = h*(waters[n].j+0.25+0.5*(rand()%101)/100);
		p.p[2] = h*(waters[n].k+0.25+0.5*(rand()%101)/100);
	}

	mSorter->sort(particles);

	for( int n=0; n<indices.size(); n++ ) {
		particle &p = *particles[indices[n]];
		Vec3f u(0.0);
		resample( mSorter.get(), p.p,u, h );
		p.u = u;
	}
}
void FluidSimulation::pushParticle(double x, double y, double z, char type) {
	Object *inside_obj = NULL;
	for (int n = 0; n < mSimulationObjects.size(); n++) {
		Object &obj = mSimulationObjects[n];

		bool found = false;
		float thickness = 3.0 / mGridSize;
		if (obj.shape == BOX) {
			if (x > obj.p[0][0] && x < obj.p[1][0] && y > obj.p[0][1]
					&& y < obj.p[1][1] && z > obj.p[0][2] && z < obj.p[1][2]) {

				if (obj.type == WALL && x > obj.p[0][0] + thickness
						&& x < obj.p[1][0] - thickness
						&& y > obj.p[0][1] + thickness
						&& y < obj.p[1][1] - thickness
						&& z > obj.p[0][2] + thickness
						&& z < obj.p[1][2] - thickness) {
					// Do nothing. Because It's too deep
					inside_obj = NULL;
					break;
				} else {
					found = true;
				}
			}
		} else if (obj.shape == SPHERE) {
			Vec3f p((float)x,(float)y,(float)z);
			Vec3f c(obj.c[0], obj.c[1], obj.c[2]);
			float len = length(p, c);
			if (len < obj.r) {
				if (obj.type == WALL) {
					found = true;
					if (len < obj.r - thickness) {
						// Do nothing. Because It's too deep
						inside_obj = NULL;
						break;
					}
				} else if (obj.type == FLUID) {
					found = true;
				}
			}
		}

		if (found) {
			if (mSimulationObjects[n].type == type) {
				inside_obj = &mSimulationObjects[n]; // Found
				break;
			}
		}
	}

	if (inside_obj) {
		particle *p = new particle;
		p->p[0] = x
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->p[1] = y
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->p[2] = z
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->u[0] = 0.0;
		p->u[1] = 0.0;
		p->u[2] = 0.0;
		p->n[0] = 0.0;
		p->n[1] = 0.0;
		p->n[2] = 0.0;
		p->thinparticle = 0;
		p->dens = 10.0;
		p->type = inside_obj->type;
		p->visible = inside_obj->visible;
		p->m = 1.0;
		mParticles.push_back(ParticlePtr(p));
	}
}
bool FluidSimulation::init() {
	mSimulationTime=0;
	mSimulationIteration=0;
	mSimulationDuration=600/mTimeStep;

	FOR_EVERY_X_FLOW(mGridSize)
		{
			mVelocityLast[0][i][j][k] = mVelocity[0][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_Y_FLOW(mGridSize)
		{
			mVelocityLast[1][i][j][k] = mVelocity[1][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_Z_FLOW(mGridSize)
		{
			mVelocityLast[2][i][j][k] = mVelocity[2][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_CELL(mGridSize)
		{
			mLabel[i][j][k] = AIR;
			mPressure[i][j][k] = 0.0;
		}END_FOR

	mSorter = std::unique_ptr<sorter>(new sorter(mGridSize));

	placeObjects();

	// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
	float h = mDensityIsoLevel / mGridSize;
	FOR_EVERY_CELL(10)
		{
			particle *p = new particle;
			p->p[0] = (i + 0.5) * h;
			p->p[1] = (j + 0.5) * h;
			p->p[2] = (k + 0.5) * h;
			p->type = FLUID;
			p->m = 1.0;
			mParticles.push_back(std::unique_ptr<particle>(p));
		}END_FOR

	mSorter->sort(mParticles);
	mMaxDensity = 1.0;

	computeDensity();
	mMaxDensity = 0.0;
	for (int n = 0; n < mParticles.size(); n++) {
		particle *p = mParticles[n].get();
		mMaxDensity = fmax(mMaxDensity, p->dens);
		delete p;
	}
	mParticles.clear();

	// Place Fluid Particles And Walls
	double w = mDensityIsoLevel * mWallThickness;
	for (int i = 0; i < mGridSize / mDensityIsoLevel; i++) {
		for (int j = 0; j < mGridSize / mDensityIsoLevel; j++) {
			for (int k = 0; k < mGridSize / mDensityIsoLevel; k++) {
				double x = i * w + w / 2.0;
				double y = j * w + w / 2.0;
				double z = k * w + w / 2.0;

				if (x > mWallThickness && x < 1.0 - mWallThickness && y > mWallThickness
						&& y < 1.0 - mWallThickness && z > mWallThickness
						&& z < 1.0 - mWallThickness) {
					pushParticle(x, y, z, FLUID);
				}
			}
		}
	}

	// Place Wall Particles And Walls
	w = 1.0 / mGridSize;
	for (int i = 0; i < mGridSize; i++) {
		for (int j = 0; j < mGridSize; j++) {
			for (int k = 0; k < mGridSize; k++) {
				double x = i * w + w / 2.0;
				double y = j * w + w / 2.0;
				double z = k * w + w / 2.0;
				pushParticle(x, y, z, WALL);
			}
		}
	}

	// Remove Particles That Stuck On Wal Cells
	mSorter->sort(mParticles);
	mSorter->markWater(mLabel, mWallWeight, mDensityIsoLevel);

	for (std::vector<ParticlePtr>::iterator iter = mParticles.begin();
			iter != mParticles.end();) {
		particle &p = **iter;
		if (p.type == WALL) {
			iter++;
			continue;
		}
		int i = fmin(mGridSize - 1, fmax(0, p.p[0] * mGridSize));
		int j = fmin(mGridSize - 1, fmax(0, p.p[1] * mGridSize));
		int k = fmin(mGridSize - 1, fmax(0, p.p[2] * mGridSize));
		if (mLabel[i][j][k] == WALL) {
			iter = mParticles.erase(iter);
		} else {
			iter++;
		}
		// Comput Normal for Walls
		compute_wall_normal();
	}
	return true;
}
void FluidSimulation::pourWater( int limit ) {
    if( mSimulationIteration > limit ) return;

    int cnt = 0;
	double w = mDensityIsoLevel/mGridSize;
    for( float x=w+w/2.0; x < 1.0-w/2.0; x += w ) {
         for( float z=w+w/2.0; z < 1.0-w/2.0; z += w ) {
             if( hypot(x-mPourPosition[0],z-mPourPosition[1]) < mPourRadius ) {
                 particle *p = new particle;
                 p->p[0] = x;
                 p->p[1] = 1.0 - mWallThickness - 2.5*mDensityIsoLevel/mGridSize;
                 p->p[2] = z;
                 p->u[0] = 0.0;
                 p->u[1] = -0.5*mDensityIsoLevel/mGridSize/mTimeStep;
                 p->u[2] = 0.0;
                 p->n[0] = 0.0;
                 p->n[1] = 0.0;
                 p->n[2] = 0.0;
				 p->thinparticle = 0;
                 p->type = FLUID;
                 p->dens = mMaxDensity;
                 p->m = 1.0;
                 mParticles.push_back(ParticlePtr(p));
                 cnt ++;
             }
         }
    }
}
void  FluidSimulation::add_ExtForce() {
	for( int n=0; n<mParticles.size(); n++ ) {
		// Add Gravity
		mParticles[n]->u[1] += -mTimeStep*GRAVITY;
	}
}
void FluidSimulation::advect_particle() {
	// Advect Particle Through Grid
	OPENMP_FOR for( int n=0; n<mParticles.size(); n++ ) {
		if( mParticles[n]->type == FLUID ) {
			Vec3f vel;
			fetchVelocity( mParticles[n]->p, vel,mVelocity, mGridSize );
			for( int k=0; k<3; k++ ) {
				mParticles[n]->p[k] += mTimeStep*vel[k];
			}
		}
	}

	// Sort
	mSorter->sort(mParticles);

	// Constraint Outer Wall
	for( int n=0; n<mParticles.size(); n++ ) {
		float r = mWallThickness;
		for( int k=0; k<3; k++ ) {
			if( mParticles[n]->type == FLUID ) {
				mParticles[n]->p[k] = fmax(r,fmin(1.0-r,mParticles[n]->p[k]));
			}
		}
		particle *p = mParticles[n].get();
		if( p->type == FLUID ) {
			int i = fmin(mGridSize-1,fmax(0,p->p[0]*mGridSize));
			int j = fmin(mGridSize-1,fmax(0,p->p[1]*mGridSize));
			int k = fmin(mGridSize-1,fmax(0,p->p[2]*mGridSize));
			vector<particle*> neighbors = mSorter->getNeigboringParticles_cell(i,j,k,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *np = neighbors[n];
				double re = 1.5*mDensityIsoLevel/mGridSize;
				if( np->type == WALL ) {
					float dist = length(p->p,np->p);
					if( dist < re ) {
						float normal[3] = { np->n[0], np->n[1], np->n[2] };
						if( normal[0] == 0.0 && normal[1] == 0.0 && normal[2] == 0.0 && dist ) {
							for( int c=0; c<3; c++ ) normal[c] = (p->p[c]-np->p[c])/dist;
						}

						p->p[0] += (re-dist)*normal[0];
						p->p[1] += (re-dist)*normal[1];
						p->p[2] += (re-dist)*normal[2];
						float dot = p->u[0] * normal[0] + p->u[1] * normal[1] + p->u[2] * normal[2];
						p->u[0] -= dot*normal[0];
						p->u[1] -= dot*normal[1];
						p->u[2] -= dot*normal[2];
					}
				}
			}
		}
	}
    // Remove Particles That Stuck On The Up-Down Wall Cells...
    for( int n=0; n<mParticles.size(); n++ ) {
        particle &p = *mParticles[n];
		p.remove = 0;

		// Focus on Only Fluid Particle
		if( p.type != FLUID ) {
			continue;
		}

		// If Stuck On Wall Cells Just Repositoin
		if( mLabel[(int)fmin(mGridSize-1,fmax(0,p.p[0]*mGridSize))][(int)fmin(mGridSize-1,fmax(0,p.p[1]*mGridSize))][(int)fmin(mGridSize-1,fmax(0,p.p[2]*mGridSize))] == WALL ) {
			p.remove = 1;
		}

        int i = fmin(mGridSize-3,fmax(2,p.p[0]*mGridSize));
        int j = fmin(mGridSize-3,fmax(2,p.p[1]*mGridSize));
        int k = fmin(mGridSize-3,fmax(2,p.p[2]*mGridSize));
        if( p.dens < 0.04 && (mLabel[i][max(0,j-1)][k] == WALL || mLabel[i][min(mGridSize-1,j+1)][k] == WALL) ) {
			// Put Into Reposition List
			p.remove = 1;
        }
    }

	// Reposition If Neccessary
	vector<int> reposition_indices;
	for( int n=0; n<mParticles.size(); n++ ) {
		if( mParticles[n]->remove ) {
			mParticles[n]->remove = 0;
			reposition_indices.push_back(n);
		}
	}
	// Store Stuck Particle Number
	mStuckParticleCount = reposition_indices.size();
	reposition(reposition_indices, mParticles );
}
void FluidSimulation::cleanup(){
	mParticles.clear();
}
bool FluidSimulation::step() {

	dump( "-------------- Step %d --------------\n", mSimulationIteration++);

    // Pour Water
    pourWater(pourTime);

	// Display Env
#if _OPENMP
	static int procs = omp_get_num_procs();
	dump( "Number of threads: %d\n", procs );
#else
	dump( "OpenMP Disabled.\n" );
#endif

    // Write Image
    dump( "Writing Preview Image..." );
    dumptime();
    dump( "%.2f sec\n", dumptime() );

	// Compute Density
	dump( "Computing Density..." );
	dumptime();
	mSorter->sort(mParticles);
	computeDensity();
	dump( "%.2f sec\n", dumptime() );

    // Solve Fluid
	dump( "Solving Liquid Flow...");
    if( mSimulationIteration == 1 && mGridSize > 64 ) {
        dump( "\n>>> NOTICE:\nBe advised that the first step of pressure solver really takes time.\nJust be patient :-)\n<<<\n");
    }
	add_ExtForce();
    solve_picflip();
    dump( "Took %.2f sec\n", dumptime() );

	// Advect Particle
	dump( "Advecting Particles...");
	advect_particle();
	dump( "%.2f sec\n", dumptime() );

	// Correct Position
#if ! DISABLE_CORRECTION
	dump( "Correcting Particle Position...");
	correct(mSorter.get(),mParticles,mTimeStep,mDensityIsoLevel/mGridSize);
	dump( "%.2f sec\n", dumptime() );
#endif

	// Save Current State
#if WRITE_SAVE
    saveState();
#endif

	// Write Mesh
#if WRITE_FILE
	write_mesh();
#endif
	mSimulationTime=mSimulationIteration*mTimeStep;
    // If Exceeds Max Step Exit
	if( mSimulationTime > mSimulationDuration) {
		return false;
	} else return true;
}
void FluidSimulation::save_grid() {
	FOR_EVERY_X_FLOW(mGridSize) {
		mVelocityLast[0][i][j][k] = mVelocity[0][i][j][k];
	} END_FOR

	FOR_EVERY_Y_FLOW(mGridSize) {
		mVelocityLast[1][i][j][k] = mVelocity[1][i][j][k];
	} END_FOR

	FOR_EVERY_Z_FLOW(mGridSize) {
		mVelocityLast[2][i][j][k] = mVelocity[2][i][j][k];
	} END_FOR
}

void FluidSimulation::subtract_grid() {
	FOR_EVERY_X_FLOW(mGridSize) {
		mVelocityLast[0][i][j][k] = mVelocity[0][i][j][k] - mVelocityLast[0][i][j][k];
	} END_FOR

	FOR_EVERY_Y_FLOW(mGridSize) {
		mVelocityLast[1][i][j][k] = mVelocity[1][i][j][k] - mVelocityLast[1][i][j][k];
	} END_FOR

	FOR_EVERY_Z_FLOW(mGridSize) {
		mVelocityLast[2][i][j][k] = mVelocity[2][i][j][k] - mVelocityLast[2][i][j][k];
	} END_FOR
}


void FluidSimulation::enforce_boundary() {
	// Set Boundary Velocity Zero
	FOR_EVERY_X_FLOW(mGridSize) {
		if( i==0 || i==mGridSize ) mVelocity[0][i][j][k] = 0.0;
		if( i<mGridSize && i>0 && isWallIndicator(mLabel[i][j][k])*isWallIndicator(mLabel[i-1][j][k]) < 0 ) {
			mVelocity[0][i][j][k] = 0.0;
		}
	} END_FOR

	FOR_EVERY_Y_FLOW(mGridSize) {
		if( j==0 || j==mGridSize ) mVelocity[1][i][j][k] = 0.0;
		if( j<mGridSize && j>0 && isWallIndicator(mLabel[i][j][k])*isWallIndicator(mLabel[i][j-1][k]) < 0 ) {
			mVelocity[1][i][j][k] = 0.0;
		}
	} END_FOR

	FOR_EVERY_Z_FLOW(mGridSize) {
		if( k==0 || k==mGridSize ) mVelocity[2][i][j][k] = 0.0;
		if( k<mGridSize && k>0 && isWallIndicator(mLabel[i][j][k])*isWallIndicator(mLabel[i][j][k-1]) < 0 ) {
			mVelocity[2][i][j][k] = 0.0;
		}
	} END_FOR
}
void FluidSimulation::project() {
	// Cell Width
	float h = 1.0/mGridSize;
	// Compute Divergence
	FOR_EVERY_CELL(mGridSize) {
		if( mLabel[i][j][k] == FLUID ) {
			mDivergence[i][j][k] = (mVelocity[0][i+1][j][k]-mVelocity[0][i][j][k]+
							mVelocity[1][i][j+1][k]-mVelocity[1][i][j][k]+
							mVelocity[2][i][j][k+1]-mVelocity[2][i][j][k]) / h;
		}
	} END_FOR;

	// Compute LevelSet
	FOR_EVERY_CELL(mGridSize) {
		mLaplacian[i][j][k] = mSorter->levelset(i,j,k,mWallWeight,mDensityIsoLevel);
	} END_FOR;

	laplace_solve(mLabel, mLaplacian, mPressure, mDivergence, mGridSize );

	// Subtract Pressure Gradient
	FOR_EVERY_X_FLOW(mGridSize) {
		if( i>0 && i<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i-1][j][k];
			if(mLaplacian[i][j][k] * mLaplacian[i-1][j][k] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/fmin(1.0e-3,mLaplacian[i-1][j][k])*mPressure[i-1][j][k];
				pb = mLaplacian[i-1][j][k] < 0.0 ? mPressure[i-1][j][k] : mLaplacian[i-1][j][k]/fmin(1.0e-6,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[0][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;

	FOR_EVERY_Y_FLOW(mGridSize) {
		if( j>0 && j<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i][j-1][k];
			if(mLaplacian[i][j][k] * mLaplacian[i][j-1][k] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/fmin(1.0e-3,mLaplacian[i][j-1][k])*mPressure[i][j-1][k];
				pb = mLaplacian[i][j-1][k] < 0.0 ? mPressure[i][j-1][k] : mLaplacian[i][j-1][k]/fmin(1.0e-6,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[1][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;

	FOR_EVERY_Z_FLOW(mGridSize) {
		if( k>0 && k<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i][j][k-1];
			if(mLaplacian[i][j][k] * mLaplacian[i][j][k-1] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/fmin(1.0e-3,mLaplacian[i][j][k-1])*mPressure[i][j][k-1];
				pb = mLaplacian[i][j][k-1] < 0.0 ? mPressure[i][j][k-1] : mLaplacian[i][j][k-1]/fmin(1.0e-6,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[2][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;
}
void FluidSimulation::extrapolate_velocity() {
	// Mark Fluid Cell Face
	StaggeredGrid<char> mark(Coord(mGridSize),Coord(0),0);
	StaggeredGrid<char> wall_mark(Coord(mGridSize),Coord(0),0);
	OPENMP_FOR FOR_EVERY_X_FLOW(mGridSize) {
		mark[0][i][j][k] = (i>0 && mLabel[i-1][j][k]==FLUID) || (i<mGridSize && mLabel[i][j][k]==FLUID);
		wall_mark[0][i][j][k] = (i<=0 || mLabel[i-1][j][k]==WALL) && (i>=mGridSize || mLabel[i][j][k]==WALL);
	} END_FOR;

	OPENMP_FOR FOR_EVERY_Y_FLOW(mGridSize) {
		mark[1][i][j][k] = (j>0 && mLabel[i][j-1][k]==FLUID) || (j<mGridSize && mLabel[i][j][k]==FLUID);
		wall_mark[1][i][j][k] = (j<=0 || mLabel[i][j-1][k]==WALL) && (j>=mGridSize || mLabel[i][j][k]==WALL);
	} END_FOR;

	OPENMP_FOR FOR_EVERY_Z_FLOW(mGridSize) {
		mark[2][i][j][k] = (k>0 && mLabel[i][j][k-1]==FLUID) || (k<mGridSize && mLabel[i][j][k]==FLUID);
		wall_mark[2][i][j][k] = (k<=0 || mLabel[i][j][k-1]==WALL) && (k>=mGridSize || mLabel[i][j][k]==WALL);
	} END_FOR;

	// Now Extrapolate
	OPENMP_FOR FOR_EVERY_CELL(mGridSize+1) {
		for( int n=0; n<3; n++ ) {
			if( n!=0 && i>mGridSize-1 ) continue;
			if( n!=1 && j>mGridSize-1 ) continue;
			if( n!=2 && k>mGridSize-1 ) continue;

			if( ! mark[n][i][j][k] && wall_mark[n][i][j][k] ) {
				int wsum = 0;
				float sum = 0.0;
				int q[][3] = { {i-1,j,k}, {i+1,j,k}, {i,j-1,k}, {i,j+1,k}, {i,j,k-1}, {i,j,k+1} };
				for( int qk=0; qk<6; qk++ ) {
					if( q[qk][0]>=0 && q[qk][0]<mGridSize+(n==0) && q[qk][1]>=0 && q[qk][1]<mGridSize+(n==1) && q[qk][2]>=0 && q[qk][2]<mGridSize+(n==2) ) {
						if( mark[n][q[qk][0]][q[qk][1]][q[qk][2]] ) {
							wsum ++;
							sum += mVelocity[n][q[qk][0]][q[qk][1]][q[qk][2]];
						}
					}
				}
				if( wsum ) mVelocity[n][i][j][k] = sum/wsum;
			}
		}
	} END_FOR;
}
void FluidSimulation::solve_picflip() {
    // Map Particles Onto Grid
	mSorter->sort(mParticles);
	mapP2G(mSorter.get(),mParticles,mVelocity,mGridSize);
	mSorter->markWater(mLabel,mWallWeight,mDensityIsoLevel);

	// Solve Fluid Velocity On Grid
	save_grid();
	enforce_boundary();
	project();
	enforce_boundary();
	extrapolate_velocity();
	subtract_grid();

	// Copy Current Velocity
	OPENMP_FOR for( int n=0; n<mParticles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			mParticles[n]->tmp[0][k] = mParticles[n]->u[k];
		}
	}

	// Map Changes Back To Particles
	mapG2P(mParticles,mVelocityLast,mGridSize);

	// Set Tmp As FLIP Velocity
	OPENMP_FOR for( int n=0; n<mParticles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			mParticles[n]->tmp[0][k] = mParticles[n]->u[k] + mParticles[n]->tmp[0][k];
		}
	}

	// Set u[] As PIC Velocity
	mapG2P(mParticles,mVelocity,mGridSize);

	// Interpolate
	OPENMP_FOR for( int n=0; n<mParticles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			mParticles[n]->u[k] = (1.0-mPicFlipBlendWeight)*mParticles[n]->u[k] + mPicFlipBlendWeight*mParticles[n]->tmp[0][k];
		}
	}

}
void FluidSimulation::createLevelSet() {
	// Create Density Field
	OPENMP_FOR FOR_EVERY_CELL(mGridSize) {
		double h = 1.0/(double)(mGridSize-1);
		double x = i*h;
		double y = j*h;
		double z = k*h;
		Vec3f p( x, y, z);
        double value = implicit_func( mSorter.get(), p, mDensityIsoLevel);
        if( i==0 || i==mGridSize-1 || j==0 || j==mGridSize-1 || k==0 || k==mGridSize-1 ) {
            value = fmax(value,0.01);
        }
        mLevelSet[i][k][k] = -value;
	} END_FOR
}
void FluidSimulation::compute_wall_normal() {
	// Sort Particles
	mSorter->sort(mParticles);
	// Compute Wall Normal
	for (int n = 0; n < mParticles.size(); n++) {
		particle *p = mParticles[n].get();
		int i = fmin(mGridSize - 1, fmax(0, p->p[0] * mGridSize));
		int j = fmin(mGridSize - 1, fmax(0, p->p[1] * mGridSize));
		int k = fmin(mGridSize - 1, fmax(0, p->p[2] * mGridSize));
		mWallNormal[i][j][k] = Vec3f(0.0f);
		p->n[0] = p->n[1] = p->n[2] = 0.0;
		if (p->type == WALL) {
			if (p->p[0] <= 1.1 * mWallThickness) {
				p->n[0] = 1.0;
			}
			if (p->p[0] >= 1.0 - 1.1 * mWallThickness) {
				p->n[0] = -1.0;
			}
			if (p->p[1] <= 1.1 * mWallThickness) {
				p->n[1] = 1.0;
			}
			if (p->p[1] >= 1.0 - 1.1 * mWallThickness) {
				p->n[1] = -1.0;
			}
			if (p->p[2] <= 1.1 * mWallThickness) {
				p->n[2] = 1.0;
			}
			if (p->p[2] >= 1.0 - 1.1 * mWallThickness) {
				p->n[2] = -1.0;
			}

			if (p->n[0] == 0.0 && p->n[1] == 0.0 && p->n[2] == 0.0) {
				vector<particle*> neighbors =
						mSorter->getNeigboringParticles_cell(i, j, k, 3, 3, 3);
				for (int n = 0; n < neighbors.size(); n++) {
					particle *np = neighbors[n];
					if (p != np && np->type == WALL) {
						float d = length(p->p, np->p);
						float w = 1.0 / d;
						p->n += w * (p->p - np->p) / d;
					}
				}
			}
		}
		p->n.normalize();
		mWallNormal[i][j][k] = p->n;
	}

	mSorter->sort(mParticles);
	mSorter->markWater(mLabel, mWallWeight, mDensityIsoLevel);

	// Compute Perimeter Normal
	FOR_EVERY_CELL(mGridSize)
		{
			mWallWeight[i][j][k] = 0.0;
			if (mLabel[i][j][k] != WALL) {
				// For Every Nearby Cells
				int sum = 0;
				Vec3f norm(0.0f);
				int neighbors[][3] = { { i - 1, j, k }, { i + 1, j, k }, { i, j
						- 1, k }, { i, j + 1, k }, { i, j, k - 1 }, { i, j, k
						+ 1 } };
				for (int m = 0; m < 6; m++) {
					int si = neighbors[m][0];
					int sj = neighbors[m][1];
					int sk = neighbors[m][2];
					if (si < 0 || si > mGridSize - 1 || sj < 0
							|| sj > mGridSize - 1 || sk < 0
							|| sk > mGridSize - 1)
						continue;
					if (mLabel[si][sj][sk] == WALL) {
						sum++;
						norm += mWallNormal[si][sj][sk];
					}
				}
				if (sum > 0) {
					norm.normalize();
					mWallNormal[i][j][k] = norm;
					mWallWeight[i][j][k] = 1.0;
				}
			}
		}END_FOR;
}
FluidSimulation::~FluidSimulation() {
	// TODO Auto-generated destructor stub
}
}
} /* namespace imagesci */
