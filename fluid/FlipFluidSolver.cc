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
using namespace std;
namespace imagesci {
namespace fluid {
const float FlipFluidSolver::ALPHA = 0.95f;
const float FlipFluidSolver::DT = 0.6e-2f;
const float FlipFluidSolver::DENSITY = 0.5f;
const float FlipFluidSolver::GRAVITY = 9.8f;
FlipFluidSolver::FlipFluidSolver(int gridSize) :
		gNumStuck(0),
		step(0), pourTime(-1),pourPos(0.0),pourRad(0.12),MAX_STEP(600),
		mGridSize(gridSize), mA(Coord(gridSize), Coord(0), 0.0f), mL(
				Coord(gridSize), Coord(0), 0.0f), mPress(Coord(gridSize),
				Coord(0), 0.0f), mVol(Coord(gridSize), Coord(0), 0.0f), mVolSave(
				Coord(gridSize), Coord(0), 0.0f), mHalfWall(Coord(gridSize),
				Coord(0), 0.0f), mWallNormal(Coord(gridSize), Coord(0),
				openvdb::Vec3f(0.0f)), max_dens(1.0) {
	WALL_THICK = 1.0f / gridSize;
}
void FlipFluidSolver::computeDensity() {
	OPENMP_FOR for (int n = 0; n < particles.size(); n++) {

		// Find Neighbors
		int gn = sort->getCellSize();
		if (particles[n]->type == WALL) {
			particles[n]->dens = 1.0;
			continue;
		}

		Vec3f& p = particles[n]->p;
		std::vector<particlePtr> neighbors = sort->getNeigboringParticles_cell(
				fmax(0, fmin(gn - 1, gn * p[0])),
				fmax(0, fmin(gn - 1, gn * p[1])),
				fmax(0, fmin(gn - 1, gn * p[2])), 1, 1, 1);
		FLOAT wsum = 0.0;
		for (int m = 0; m < neighbors.size(); m++) {
			particle np = *neighbors[m];
			if (np.type == WALL)
				continue;
			FLOAT d2 = length2(np.p, p);
			FLOAT w = np.m * smooth_kernel(d2, 4.0f * DENSITY / mGridSize);
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
	obj.p[0][0] = 0.0;
	obj.p[1][0] = WALL_THICK;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Right Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 1.0 - WALL_THICK;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Floor Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GRAY;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = WALL_THICK;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	objects.push_back(obj);

	// Ceiling Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 1.0 - WALL_THICK;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 0.0;
	obj.p[1][2] = 1.0;
	objects.push_back(obj);

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
	obj.p[1][2] = WALL_THICK;
	objects.push_back(obj);

	// Back Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.visible = 0;
	obj.p[0][0] = 0.0;
	obj.p[1][0] = 1.0;
	obj.p[0][1] = 0.0;
	obj.p[1][1] = 1.0;
	obj.p[0][2] = 1.0 - WALL_THICK;
	obj.p[1][2] = 1.0;
	objects.push_back(obj);
}
void FlipFluidSolver::damBreakTest() {
	Object obj;

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = 0.2;
	obj.p[1][0] = 0.4;
	obj.p[0][1] = WALL_THICK;
	obj.p[1][1] = 0.4;
	obj.p[0][2] = 0.2;
	obj.p[1][2] = 0.8;

	objects.push_back(obj);

	obj.type = FLUID;
	obj.shape = BOX;
	obj.visible = true;
	obj.p[0][0] = WALL_THICK;
	obj.p[1][0] = 1.0 - WALL_THICK;
	obj.p[0][1] = WALL_THICK;
	obj.p[1][1] = 0.06;
	obj.p[0][2] = WALL_THICK;
	obj.p[1][2] = 1.0 - WALL_THICK;

	objects.push_back(obj);
}
void FlipFluidSolver::placeObjects() {
	// Place Object Wall
	placeWalls();

	// profileTest();
	// waterDropTest();
	// cliffPourTest();
	damBreakTest();
	// spherePourTest();
}
void FlipFluidSolver::reposition(vector<int>& indices, vector<particlePtr> particles ) {
	if( indices.empty() ) return;
	int gn = sort->getCellSize();

	// First Search for Deep Water
	vector<ipos> waters;
	while( waters.size() < indices.size() ) {
		FOR_EVERY_CELL(gn) {
			if( i > 0 && mA[i-1][j][k] != FLUID ) continue;
			if( i < mGridSize-1 && mA[i+1][j][k] != FLUID ) continue;
			if( j > 0 && mA[i][j-1][k] != FLUID ) continue;
			if( j < mGridSize-1 && mA[i][j+1][k] != FLUID ) continue;
			if( k > 0 && mA[i][j][k-1] != FLUID ) continue;
			if( k < mGridSize-1 && mA[i][j][k+1] != FLUID ) continue;
			if( mA[i][j][k] != FLUID ) continue;

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

	FLOAT h = 1.0/gn;
	for( int n=0; n<indices.size(); n++ ) {
		particle &p = *particles[indices[n]];
		p.p[0] = h*(waters[n].i+0.25+0.5*(rand()%101)/100);
		p.p[1] = h*(waters[n].j+0.25+0.5*(rand()%101)/100);
		p.p[2] = h*(waters[n].k+0.25+0.5*(rand()%101)/100);
	}

	sort->sort(particles);

	for( int n=0; n<indices.size(); n++ ) {
		particle &p = *particles[indices[n]];
		Vec3f u(0.0);
		resample( sort.get(), p.p,u, h );
		p.u = u;
	}
}
void FlipFluidSolver::pushParticle(double x, double y, double z, char type) {
	Object *inside_obj = NULL;
	for (int n = 0; n < objects.size(); n++) {
		Object &obj = objects[n];

		bool found = false;
		FLOAT thickness = 3.0 / mGridSize;
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
			FLOAT p[3] = { x, y, z };
			FLOAT c[3] = { obj.c[0], obj.c[1], obj.c[2] };
			FLOAT len = length(p, c);
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
			if (objects[n].type == type) {
				inside_obj = &objects[n]; // Found
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
		particles.push_back(particlePtr(p));
	}
}
void FlipFluidSolver::init() {
	FOR_EVERY_X_FLOW(mGridSize)
		{
			mVolSave[0][i][j][k] = mVol[0][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_Y_FLOW(mGridSize)
		{
			mVolSave[1][i][j][k] = mVol[1][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_Z_FLOW(mGridSize)
		{
			mVolSave[2][i][j][k] = mVol[2][i][j][k] = 0.0;
		}END_FOR

	FOR_EVERY_CELL(mGridSize)
		{
			mA[i][j][k] = AIR;
			mPress[i][j][k] = 0.0;
		}END_FOR

	sort = std::unique_ptr<sorter>(new sorter(mGridSize));

	placeObjects();

	// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
	FLOAT h = DENSITY / mGridSize;
	FOR_EVERY_CELL(10)
		{
			particle *p = new particle;
			p->p[0] = (i + 0.5) * h;
			p->p[1] = (j + 0.5) * h;
			p->p[2] = (k + 0.5) * h;
			p->type = FLUID;
			p->m = 1.0;
			particles.push_back(std::unique_ptr<particle>(p));
		}END_FOR

	sort->sort(particles);
	max_dens = 1.0;

	computeDensity();
	max_dens = 0.0;
	for (int n = 0; n < particles.size(); n++) {
		particle *p = particles[n].get();
		max_dens = fmax(max_dens, p->dens);
		delete p;
	}
	particles.clear();

	// Place Fluid Particles And Walls
	double w = DENSITY * WALL_THICK;
	for (int i = 0; i < mGridSize / DENSITY; i++) {
		for (int j = 0; j < mGridSize / DENSITY; j++) {
			for (int k = 0; k < mGridSize / DENSITY; k++) {
				double x = i * w + w / 2.0;
				double y = j * w + w / 2.0;
				double z = k * w + w / 2.0;

				if (x > WALL_THICK && x < 1.0 - WALL_THICK && y > WALL_THICK
						&& y < 1.0 - WALL_THICK && z > WALL_THICK
						&& z < 1.0 - WALL_THICK) {
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
	sort->sort(particles);
	sort->markWater(mA, mHalfWall, DENSITY);

	for (std::vector<particlePtr>::iterator iter = particles.begin();
			iter != particles.end();) {
		particle &p = **iter;
		if (p.type == WALL) {
			iter++;
			continue;
		}
		int i = fmin(mGridSize - 1, fmax(0, p.p[0] * mGridSize));
		int j = fmin(mGridSize - 1, fmax(0, p.p[1] * mGridSize));
		int k = fmin(mGridSize - 1, fmax(0, p.p[2] * mGridSize));
		if (mA[i][j][k] == WALL) {
			iter = particles.erase(iter);
		} else {
			iter++;
		}
		// Comput Normal for Walls
		compute_wall_normal();
	}
}
void FlipFluidSolver::pourWater( int limit ) {
    if( step > limit ) return;

    int cnt = 0;
	double w = DENSITY/mGridSize;
    for( FLOAT x=w+w/2.0; x < 1.0-w/2.0; x += w ) {
         for( FLOAT z=w+w/2.0; z < 1.0-w/2.0; z += w ) {
             if( hypot(x-pourPos[0],z-pourPos[1]) < pourRad ) {
                 particle *p = new particle;
                 p->p[0] = x;
                 p->p[1] = 1.0 - WALL_THICK - 2.5*DENSITY/mGridSize;
                 p->p[2] = z;
                 p->u[0] = 0.0;
                 p->u[1] = -0.5*DENSITY/mGridSize/DT;
                 p->u[2] = 0.0;
                 p->n[0] = 0.0;
                 p->n[1] = 0.0;
                 p->n[2] = 0.0;
				 p->thinparticle = 0;
                 p->type = FLUID;
                 p->dens = max_dens;
                 p->m = 1.0;
                 particles.push_back(particlePtr(p));
                 cnt ++;
             }
         }
    }
}
void  FlipFluidSolver::add_ExtForce() {
	for( int n=0; n<particles.size(); n++ ) {
		// Add Gravity
		particles[n]->u[1] += -DT*GRAVITY;
	}
}
void FlipFluidSolver::advect_particle() {
	// Advect Particle Through Grid
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->type == FLUID ) {
			Vec3f vel;
			fetchVelocity( particles[n]->p, vel,mVol, mGridSize );
			for( int k=0; k<3; k++ ) {
				particles[n]->p[k] += DT*vel[k];
			}
		}
	}

	// Sort
	sort->sort(particles);

	// Constraint Outer Wall
	for( int n=0; n<particles.size(); n++ ) {
		FLOAT r = WALL_THICK;
		for( int k=0; k<3; k++ ) {
			if( particles[n]->type == FLUID ) {
				particles[n]->p[k] = fmax(r,fmin(1.0-r,particles[n]->p[k]));
			}
		}
		particle *p = particles[n].get();
		if( p->type == FLUID ) {
			int i = fmin(mGridSize-1,fmax(0,p->p[0]*mGridSize));
			int j = fmin(mGridSize-1,fmax(0,p->p[1]*mGridSize));
			int k = fmin(mGridSize-1,fmax(0,p->p[2]*mGridSize));
			vector<particlePtr> neighbors = sort->getNeigboringParticles_cell(i,j,k,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				particle *np = neighbors[n].get();
				double re = 1.5*DENSITY/mGridSize;
				if( np->type == WALL ) {
					FLOAT dist = length(p->p,np->p);
					if( dist < re ) {
						FLOAT normal[3] = { np->n[0], np->n[1], np->n[2] };
						if( normal[0] == 0.0 && normal[1] == 0.0 && normal[2] == 0.0 && dist ) {
							for( int c=0; c<3; c++ ) normal[c] = (p->p[c]-np->p[c])/dist;
						}

						p->p[0] += (re-dist)*normal[0];
						p->p[1] += (re-dist)*normal[1];
						p->p[2] += (re-dist)*normal[2];
						FLOAT dot = p->u[0] * normal[0] + p->u[1] * normal[1] + p->u[2] * normal[2];
						p->u[0] -= dot*normal[0];
						p->u[1] -= dot*normal[1];
						p->u[2] -= dot*normal[2];
					}
				}
			}
		}
	}
    // Remove Particles That Stuck On The Up-Down Wall Cells...
    for( int n=0; n<particles.size(); n++ ) {
        particle &p = *particles[n];
		p.remove = 0;

		// Focus on Only Fluid Particle
		if( p.type != FLUID ) {
			continue;
		}

		// If Stuck On Wall Cells Just Repositoin
		if( mA[(int)fmin(mGridSize-1,fmax(0,p.p[0]*mGridSize))][(int)fmin(mGridSize-1,fmax(0,p.p[1]*mGridSize))][(int)fmin(mGridSize-1,fmax(0,p.p[2]*mGridSize))] == WALL ) {
			p.remove = 1;
		}

        int i = fmin(mGridSize-3,fmax(2,p.p[0]*mGridSize));
        int j = fmin(mGridSize-3,fmax(2,p.p[1]*mGridSize));
        int k = fmin(mGridSize-3,fmax(2,p.p[2]*mGridSize));
        if( p.dens < 0.04 && (mA[i][max(0,j-1)][k] == WALL || mA[i][min(mGridSize-1,j+1)][k] == WALL) ) {
			// Put Into Reposition List
			p.remove = 1;
        }
    }

	// Reposition If Neccessary
	vector<int> reposition_indices;
	for( int n=0; n<particles.size(); n++ ) {
		if( particles[n]->remove ) {
			particles[n]->remove = 0;
			reposition_indices.push_back(n);
		}
	}
	// Store Stuck Particle Number
	gNumStuck = reposition_indices.size();
	reposition(reposition_indices, particles );
}
void FlipFluidSolver::simulateStep() {

	dump( "-------------- Step %d --------------\n", step++);

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
	sort->sort(particles);
	computeDensity();
	dump( "%.2f sec\n", dumptime() );

    // Solve Fluid
	dump( "Solving Liquid Flow...");
    if( step == 1 && mGridSize > 64 ) {
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
	correct(sort.get(),particles,DT,DENSITY/mGridSize);
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

    // If Exceeds Max Step Exit
	if( step > MAX_STEP ) {
        dump( "Maximum Timestep Reached. Exiting...\n");
		exit(0);
	}
}
void FlipFluidSolver::save_grid() {
	FOR_EVERY_X_FLOW(mGridSize) {
		mVolSave[0][i][j][k] = mVol[0][i][j][k];
	} END_FOR

	FOR_EVERY_Y_FLOW(mGridSize) {
		mVolSave[1][i][j][k] = mVol[1][i][j][k];
	} END_FOR

	FOR_EVERY_Z_FLOW(mGridSize) {
		mVolSave[2][i][j][k] = mVol[2][i][j][k];
	} END_FOR
}

void FlipFluidSolver::subtract_grid() {
	FOR_EVERY_X_FLOW(mGridSize) {
		mVolSave[0][i][j][k] = mVol[0][i][j][k] - mVolSave[0][i][j][k];
	} END_FOR

	FOR_EVERY_Y_FLOW(mGridSize) {
		mVolSave[1][i][j][k] = mVol[1][i][j][k] - mVolSave[1][i][j][k];
	} END_FOR

	FOR_EVERY_Z_FLOW(mGridSize) {
		mVolSave[2][i][j][k] = mVol[2][i][j][k] - mVolSave[2][i][j][k];
	} END_FOR
}
void FlipFluidSolver::solve_picflip() {
    // Map Particles Onto Grid
	sort->sort(particles);
	mapP2G(sort.get(),particles,mVol,mGridSize);
	sort->markWater(mA,mHalfWall,DENSITY);

	// Solve Fluid Velocity On Grid
	save_grid();
	enforce_boundary();
	project();
	enforce_boundary();
	extrapolate_velocity();
	subtract_grid();

	// Copy Current Velocity
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			particles[n]->tmp[0][k] = particles[n]->u[k];
		}
	}

	// Map Changes Back To Particles
	mapG2P(particles,mVolSave,mGridSize);

	// Set Tmp As FLIP Velocity
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			particles[n]->tmp[0][k] = particles[n]->u[k] + particles[n]->tmp[0][k];
		}
	}

	// Set u[] As PIC Velocity
	mapG2P(particles,mVol,mGridSize);

	// Interpolate
	OPENMP_FOR for( int n=0; n<particles.size(); n++ ) {
		for( int k=0; k<3; k++ ) {
			particles[n]->u[k] = (1.0-ALPHA)*particles[n]->u[k] + ALPHA*particles[n]->tmp[0][k];
		}
	}

}
void FlipFluidSolver::compute_wall_normal() {
	// Sort Particles
	sort->sort(particles);
	// Compute Wall Normal
	for (int n = 0; n < particles.size(); n++) {
		particle *p = particles[n].get();
		int i = fmin(mGridSize - 1, fmax(0, p->p[0] * mGridSize));
		int j = fmin(mGridSize - 1, fmax(0, p->p[1] * mGridSize));
		int k = fmin(mGridSize - 1, fmax(0, p->p[2] * mGridSize));
		mWallNormal[i][j][k] = Vec3f(0.0f);
		p->n[0] = p->n[1] = p->n[2] = 0.0;
		if (p->type == WALL) {
			if (p->p[0] <= 1.1 * WALL_THICK) {
				p->n[0] = 1.0;
			}
			if (p->p[0] >= 1.0 - 1.1 * WALL_THICK) {
				p->n[0] = -1.0;
			}
			if (p->p[1] <= 1.1 * WALL_THICK) {
				p->n[1] = 1.0;
			}
			if (p->p[1] >= 1.0 - 1.1 * WALL_THICK) {
				p->n[1] = -1.0;
			}
			if (p->p[2] <= 1.1 * WALL_THICK) {
				p->n[2] = 1.0;
			}
			if (p->p[2] >= 1.0 - 1.1 * WALL_THICK) {
				p->n[2] = -1.0;
			}

			if (p->n[0] == 0.0 && p->n[1] == 0.0 && p->n[2] == 0.0) {
				vector<particlePtr> neighbors =
						sort->getNeigboringParticles_cell(i, j, k, 3, 3, 3);
				for (int n = 0; n < neighbors.size(); n++) {
					particle *np = neighbors[n].get();
					if (p != np && np->type == WALL) {
						FLOAT d = length(p->p, np->p);
						FLOAT w = 1.0 / d;
						p->n += w * (p->p - np->p) / d;
					}
				}
			}
		}
		p->n.normalize();
		mWallNormal[i][j][k] = p->n;
	}

	sort->sort(particles);
	sort->markWater(mA, mHalfWall, DENSITY);

	// Compute Perimeter Normal
	FOR_EVERY_CELL(mGridSize)
		{
			mHalfWall[i][j][k] = 0.0;
			if (mA[i][j][k] != WALL) {
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
					if (mA[si][sj][sk] == WALL) {
						sum++;
						norm += mWallNormal[si][sj][sk];
					}
				}
				if (sum > 0) {
					norm.normalize();
					mWallNormal[i][j][k] = norm;
					mHalfWall[i][j][k] = 1.0;
				}
			}
		}END_FOR;
}
FlipFluidSolver::~FlipFluidSolver() {
	// TODO Auto-generated destructor stub
}
}
} /* namespace imagesci */
