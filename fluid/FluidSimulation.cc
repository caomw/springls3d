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
#include "../ImageSciUtil.h"
#include <openvdb/openvdb.h>
using namespace openvdb;
using namespace openvdb::tools;
using namespace std;
namespace imagesci {
namespace fluid {
const float FluidSimulation::GRAVITY = 9.8067f;
FluidSimulation::FluidSimulation(int gridSize,MotionScheme scheme) :
		Simulation("Fluid Simulation",scheme),
		mMaxDensity(0.0),
		mStuckParticleCount(0),
		mPicFlipBlendWeight(0.95f),
		mFluidParticleDensity(0.5f),
		mGridSize(gridSize),
		mLevelSet(Coord(gridSize),Coord(0),0),
		mLabel(Coord(gridSize), Coord(0), 0.0f),mDivergence(Coord(gridSize), Coord(0), 0.0f), mLaplacian(
				Coord(gridSize), Coord(0), 0.0f), mPressure(Coord(gridSize),
				Coord(0), 0.0f), mVelocity(Coord(gridSize), Coord(0), 0.0f), mVelocityLast(
				Coord(gridSize), Coord(0), 0.0f), mWallWeight(Coord(gridSize),
				Coord(0), 0.0f), mWallNormal(Coord(gridSize), Coord(0),
				openvdb::Vec3f(0.0f)) {
	mWallThickness = 1.0f / gridSize;
	mTimeStep=0.6e-2f;
	mSimulationDuration=600.0f*mTimeStep;
}
void FluidSimulation::computeParticleDensity(float maxDensity) {
	OPENMP_FOR for(ParticlePtr& p:mParticles) {
		// Find Neighbors
		int gn = mParticleLocator->getCellSize();
		if (p->mObjectType == WALL) {
			p->mDensity = 1.0;
			continue;
		}
		Vec3f& pt = p->mLocation;
		std::vector<FluidParticle*> neighbors = mParticleLocator->getNeigboringCellParticles(
				clamp(gn*pt[0],0.0f,gn-1.0f),
				clamp(gn*pt[1],0.0f,gn-1.0f),
				clamp(gn*pt[2],0.0f,gn-1.0f), 1, 1, 1);
		float wsum = 0.0;
		for (int m = 0; m < neighbors.size(); m++) {
			FluidParticle& np = *neighbors[m];
			if (np.mObjectType == WALL)
				continue;
			float d2 = length2(np.mLocation, pt);
			float w = np.mMass * smooth_kernel(d2, 4.0f * mFluidParticleDensity / mGridSize);
			wsum += w;
		}
		p->mDensity = wsum / maxDensity;
	} OPENMP_END
}
void FluidSimulation::placeWalls() {
	CollisionObject obj;

	// Left Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(0.0,0.0,0.0);
	obj.mBounds[1]=Vec3f(mWallThickness,1.0,1.0);
	mCollisionObjects.push_back(obj);

	// Right Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(1.0-mWallThickness,0.0,0.0);
	obj.mBounds[1]=Vec3f(1.0,1.0,1.0);
	mCollisionObjects.push_back(obj);

	// Floor Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GRAY;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(0.0,0.0,0.0);
	obj.mBounds[1]=Vec3f(1.0,mWallThickness,1.0);
	mCollisionObjects.push_back(obj);

	// Ceiling Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(0.0,1.0-mWallThickness,0.0);
	obj.mBounds[1]=Vec3f(1.0,1.0,1.0);
	mCollisionObjects.push_back(obj);

	// Front Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(0.0,0.0,0.0);
	obj.mBounds[1]=Vec3f(1.0,1.0,mWallThickness);
	mCollisionObjects.push_back(obj);

	// Back Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0]=Vec3f(0.0,0.0,1.0-mWallThickness);
	obj.mBounds[1]=Vec3f(1.0,1.0,1.0);
	mCollisionObjects.push_back(obj);
}
void FluidSimulation::damBreakTest() {
	//replace with level set for falling object
	CollisionObject obj;
	obj.type = FLUID;
	obj.shape = BOX;
	obj.mVisible = true;
	obj.mBounds[0]=Vec3f(0.2,mWallThickness,0.2);
	obj.mBounds[1]=Vec3f(0.4,0.4,0.8);
	mCollisionObjects.push_back(obj);
	obj.type = FLUID;
	obj.shape = BOX;
	obj.mVisible = true;
	obj.mBounds[0]=Vec3f(mWallThickness,mWallThickness,mWallThickness);
	obj.mBounds[1]=Vec3f(1.0-mWallThickness,0.06,1.0-mWallThickness);
	mCollisionObjects.push_back(obj);
}
void FluidSimulation::placeObjects() {
	// Place Object Wall
	placeWalls();
	damBreakTest();
}
void FluidSimulation::repositionParticles(vector<int>& indices, vector<ParticlePtr> particles ) {
	if( indices.empty() ) return;
	int gn = mParticleLocator->getCellSize();

	// First Search for Deep Water
	vector<Coord> waters;
	while( waters.size() < indices.size() ) {
		FOR_EVERY_CELL(gn) {
			if( i > 0 && mLabel[i-1][j][k] != FLUID ) continue;
			if( i < mGridSize-1 && mLabel[i+1][j][k] != FLUID ) continue;
			if( j > 0 && mLabel[i][j-1][k] != FLUID ) continue;
			if( j < mGridSize-1 && mLabel[i][j+1][k] != FLUID ) continue;
			if( k > 0 && mLabel[i][j][k-1] != FLUID ) continue;
			if( k < mGridSize-1 && mLabel[i][j][k+1] != FLUID ) continue;
			if( mLabel[i][j][k] != FLUID ) continue;

			Coord aPos(i,j,k);
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
		FluidParticle &p = *particles[indices[n]];
		p.mLocation[0] = h*(waters[n][0]+0.25+0.5*(rand()%101)/100);
		p.mLocation[1] = h*(waters[n][1]+0.25+0.5*(rand()%101)/100);
		p.mLocation[2] = h*(waters[n][2]+0.25+0.5*(rand()%101)/100);
	}

	mParticleLocator->update(particles);

	for( int n=0; n<indices.size(); n++ ) {
		FluidParticle &p = *particles[indices[n]];
		Vec3f u(0.0);
		resampleParticles( mParticleLocator.get(), p.mLocation,u, h );
		p.mVelocity = u;
	}
}
void FluidSimulation::addParticle(double x, double y, double z, char type) {
	//Add case for converting MESH object into collection of particles via inside/outside level set test.

	CollisionObject *inside_obj = NULL;
	for (CollisionObject &obj: mCollisionObjects) {
		bool found = false;
		float thickness = 3.0 / mGridSize;
		if (obj.shape == BOX) {
			if (x > obj.mBounds[0][0] && x < obj.mBounds[1][0] && y > obj.mBounds[0][1]
					&& y < obj.mBounds[1][1] && z > obj.mBounds[0][2] && z < obj.mBounds[1][2]) {

				if (obj.type == WALL && x > obj.mBounds[0][0] + thickness
						&& x < obj.mBounds[1][0] - thickness
						&& y > obj.mBounds[0][1] + thickness
						&& y < obj.mBounds[1][1] - thickness
						&& z > obj.mBounds[0][2] + thickness
						&& z < obj.mBounds[1][2] - thickness) {
					// Do nothing. Because It's too deep
					inside_obj = NULL;
					break;
				} else {
					found = true;
				}
			}
		} else if (obj.shape == SPHERE) {
			Vec3f p((float)x,(float)y,(float)z);
			Vec3f c(obj.mColor[0], obj.mColor[1], obj.mColor[2]);
			float len = length(p, c);
			if (len < obj.mRadius) {
				if (obj.type == WALL) {
					found = true;
					if (len < obj.mRadius - thickness) {
						// Do nothing. Because It's too deep
						// that's what she said.
						inside_obj = NULL;
						break;
					}
				} else if (obj.type == FLUID) {
					found = true;
				}
			}
		}

		if (found) {
			if (obj.type == type) {
				inside_obj = &obj; // Found
				break;
			}
		}
	}

	if (inside_obj) {
		FluidParticle *p = new FluidParticle;
		p->mLocation[0] = x
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->mLocation[1] = y
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->mLocation[2] = z
				+ 0.01 * (inside_obj->type == FLUID) * 0.2
						* ((rand() % 101) / 50.0 - 1.0) / mGridSize;
		p->mVelocity=Vec3f(0.0);
		p->mNormal=Vec3f(0.0);
		p->mThinParticle = 0;
		p->mDensity = 10.0;
		p->mObjectType = inside_obj->type;
		p->mVisible = inside_obj->mVisible;
		p->mMass = 1.0;
		mParticles.push_back(ParticlePtr(p));
	}
}
bool FluidSimulation::init() {
	mSimulationTime=0;
	mSimulationIteration=0;
	mParticleLocator = std::unique_ptr<ParticleLocator>(new ParticleLocator(mGridSize));
	placeObjects();
	// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
	float h = mFluidParticleDensity / mGridSize;
	FOR_EVERY_CELL(10)
		{
			FluidParticle *p = new FluidParticle;
			p->mLocation=Vec3f((i + 0.5) * h,(j + 0.5) * h,(k + 0.5) * h);
			p->mObjectType = FLUID;
			p->mMass = 1.0;
			mParticles.push_back(std::unique_ptr<FluidParticle>(p));
		}END_FOR
	mParticleLocator->update(mParticles);
	computeParticleDensity(1.0f);
	mMaxDensity = 0.0;
	for (ParticlePtr& p:mParticles) {
		mMaxDensity = max(mMaxDensity, p->mDensity);
	}
	mParticles.clear();
	// Place Fluid Particles And Walls
	double w = mFluidParticleDensity * mWallThickness;
	for (int i = 0; i < mGridSize / mFluidParticleDensity; i++) {
		for (int j = 0; j < mGridSize / mFluidParticleDensity; j++) {
			for (int k = 0; k < mGridSize / mFluidParticleDensity; k++) {
				double x = i * w + w / 2.0;
				double y = j * w + w / 2.0;
				double z = k * w + w / 2.0;
				if (
						x > mWallThickness &&
						x < 1.0 - mWallThickness &&
						y > mWallThickness &&
						y < 1.0 - mWallThickness &&
						z > mWallThickness &&
						z < 1.0 - mWallThickness) {
					addParticle(x, y, z, FLUID);
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
				addParticle(x, y, z, WALL);
			}
		}
	}
	mParticleLocator->update(mParticles);
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDensity);

	// Remove Particles That Stuck On Wal Cells
	for (std::vector<ParticlePtr>::iterator iter = mParticles.begin();iter != mParticles.end();) {
		ParticlePtr& p = *iter;
		if (p->mObjectType == WALL) {
			iter++;
			continue;
		}
		int i = clamp(mGridSize*p->mLocation[0],0.0f,mGridSize-1.0f);
		int j = clamp(mGridSize*p->mLocation[1],0.0f,mGridSize-1.0f);
		int k = clamp(mGridSize*p->mLocation[2],0.0f,mGridSize-1.0f);
		if (mLabel[i][j][k] == WALL) {
			iter = mParticles.erase(iter);
		} else {
			iter++;
		}
	}
	// Comput Normal for Walls
	computeWallNormals();
	createLevelSet();
	return true;
}
void FluidSimulation::pourWater( int limit ,float maxDensity) {
    if( mSimulationIteration > limit ) return;
    Vec2f mPourPosition(0.0,0.0);
    float mPourRadius(0.12);
    int cnt = 0;
	double w = mFluidParticleDensity/mGridSize;
    for( float x=w+w/2.0; x < 1.0-w/2.0; x += w ) {
         for( float z=w+w/2.0; z < 1.0-w/2.0; z += w ) {
             if( hypot(x-mPourPosition[0],z-mPourPosition[1]) < mPourRadius ) {
                 FluidParticle *p = new FluidParticle;
                 p->mLocation=Vec3f(x, 1.0 - mWallThickness - 2.5*mFluidParticleDensity/mGridSize,z);
                 p->mVelocity=Vec3f(0.0,-0.5*mFluidParticleDensity/mGridSize/mTimeStep,0.0);
                 p->mNormal=Vec3f(0.0);
				 p->mThinParticle = 0;
                 p->mObjectType = FLUID;
                 p->mDensity = maxDensity;
                 p->mMass = 1.0;
                 mParticles.push_back(ParticlePtr(p));
                 cnt ++;
             }
         }
    }
}
void  FluidSimulation::addExternalForce() {
	for(ParticlePtr& p:mParticles) {
		// Add Gravity
		p->mVelocity[1] += -mTimeStep*GRAVITY;
	}
}
void FluidSimulation::advectParticles() {
	// Advect Particle Through Grid
	OPENMP_FOR for(ParticlePtr& p:mParticles) {
		if(p->mObjectType == FLUID ) {
			Vec3f vel;
			fetchVelocity(p->mLocation, vel,mVelocity, mGridSize );
			p->mLocation += mTimeStep*vel;
		}
	}
	// Sort
	mParticleLocator->update(mParticles);
	// Constraint Outer Wall
	for(ParticlePtr& p:mParticles) {
		float r = mWallThickness;
		for( int k=0; k<3; k++ ) {
			if( p->mObjectType == FLUID ) {
				p->mLocation[k] = clamp(p->mLocation[k],r,1.0f-r);
			}
		}
		if( p->mObjectType == FLUID ) {
			int i = clamp(p->mLocation[0]*mGridSize,0.0f,mGridSize-1.0f);
			int j = clamp(p->mLocation[1]*mGridSize,0.0f,mGridSize-1.0f);
			int k = clamp(p->mLocation[2]*mGridSize,0.0f,mGridSize-1.0f);
			vector<FluidParticle*> neighbors = mParticleLocator->getNeigboringCellParticles(i,j,k,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *np = neighbors[n];
				double re = 1.5*mFluidParticleDensity/mGridSize;
				if( np->mObjectType == WALL ) {
					float dist = length(p->mLocation,np->mLocation);
					if( dist < re ) {
						float normal[3] = { np->mNormal[0], np->mNormal[1], np->mNormal[2] };
						if( normal[0] == 0.0 && normal[1] == 0.0 && normal[2] == 0.0 && dist ) {
							for( int c=0; c<3; c++ ) normal[c] = (p->mLocation[c]-np->mLocation[c])/dist;
						}

						p->mLocation[0] += (re-dist)*normal[0];
						p->mLocation[1] += (re-dist)*normal[1];
						p->mLocation[2] += (re-dist)*normal[2];
						float dot = p->mVelocity[0] * normal[0] + p->mVelocity[1] * normal[1] + p->mVelocity[2] * normal[2];
						p->mVelocity[0] -= dot*normal[0];
						p->mVelocity[1] -= dot*normal[1];
						p->mVelocity[2] -= dot*normal[2];
					}
				}
			}
		}
	}
    // Remove Particles That Stuck On The Up-Down Wall Cells...
	for(ParticlePtr& p:mParticles) {
		p->mRemoveIndicator = 0;
		// Focus on Only Fluid Particle
		if( p->mObjectType != FLUID ) {
			continue;
		}

		// If Stuck On Wall Cells Just Repositoin
		if( mLabel
				[(int)clamp(p->mLocation[0]*mGridSize,0.0f,mGridSize-1.0f)]
				[(int)clamp(p->mLocation[1]*mGridSize,0.0f,mGridSize-1.0f)]
				[(int)clamp(p->mLocation[2]*mGridSize,0.0f,mGridSize-1.0f)] == WALL ) {
			p->mRemoveIndicator = 1;
		}

        int i =clamp(p->mLocation[0]*mGridSize,2.0f,mGridSize-3.0f);
        int j =clamp(p->mLocation[1]*mGridSize,2.0f,mGridSize-3.0f);
        int k =clamp(p->mLocation[2]*mGridSize,2.0f,mGridSize-3.0f);
        if( p->mDensity < 0.04 && (mLabel[i][max(0,j-1)][k] == WALL || mLabel[i][min(mGridSize-1,j+1)][k] == WALL) ) {
			// Put Into Reposition List
			p->mRemoveIndicator = 1;
        }
    }

	// Reposition If Neccessary
	vector<int> reposition_indices;
	size_t n=0;
	for(ParticlePtr& p:mParticles) {
		if( p->mRemoveIndicator ) {
			p->mRemoveIndicator = 0;
			reposition_indices.push_back(n);
		}
		n++;
	}
	// Store Stuck Particle Number
	mStuckParticleCount = reposition_indices.size();
	repositionParticles(reposition_indices, mParticles );
}
void FluidSimulation::cleanup(){
	mParticles.clear();
}
bool FluidSimulation::step() {
    //pourWater(pourTime);
	std::cout<<"Update particles ..."<<std::endl;
	mParticleLocator->update(mParticles);
	std::cout<<"Compute particle density ... "<<mMaxDensity<<std::endl;
	computeParticleDensity(mMaxDensity);
	std::cout<<"Add external force ... "<<std::endl;
	addExternalForce();
	std::cout<<"Solve pic/flip ... "<<std::endl;
    solvePicFlip();
	std::cout<<"Advect particles ... "<<std::endl;
	advectParticles();
	std::cout<<"Correct particles ... "<<std::endl;
	correctParticles(mParticleLocator.get(),mParticles,mTimeStep,mFluidParticleDensity/mGridSize);
    // If Exceeds Max Step Exit
	mSimulationIteration++;
	mSimulationTime=mSimulationIteration*mTimeStep;
	std::cout<<"Create level set ... t="<<mSimulationTime<<" "<<mTimeStep<<std::endl;
	createLevelSet();

	if(mSimulationTime<=mSimulationDuration&&mRunning){
		return true;
	} else {
		return false;
	}
}
void FluidSimulation::copyGridToBuffer() {
	mVelocity[0].copyTo(mVelocityLast[0]);
	mVelocity[1].copyTo(mVelocityLast[1]);
	mVelocity[2].copyTo(mVelocityLast[2]);
}

void FluidSimulation::subtractGrid() {
	mVelocityLast[0].subtractFrom(mVelocity[0]);
	mVelocityLast[1].subtractFrom(mVelocity[1]);
	mVelocityLast[2].subtractFrom(mVelocity[2]);
}

void FluidSimulation::enforceBoundaryCondition() {
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
			mDivergence[i][j][k] =
						   (mVelocity[0][i+1][j][k]-mVelocity[0][i][j][k]+
							mVelocity[1][i][j+1][k]-mVelocity[1][i][j][k]+
							mVelocity[2][i][j][k+1]-mVelocity[2][i][j][k]) / h;
		}
	} END_FOR;

	// Compute LevelSet
	FOR_EVERY_CELL(mGridSize) {
		mLaplacian[i][j][k] = mParticleLocator->getLevelSetValue(i,j,k,mWallWeight,mFluidParticleDensity);
	} END_FOR;
	std::cout<<"Laplace solve ... "<<std::endl;
	laplace_solve(mLabel, mLaplacian, mPressure, mDivergence, mGridSize );

	std::cout<<"Subtract pressure gradient ... "<<std::endl;
	// Subtract Pressure Gradient
	FOR_EVERY_X_FLOW(mGridSize) {
		if( i>0 && i<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i-1][j][k];
			if(mLaplacian[i][j][k] * mLaplacian[i-1][j][k] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/min(1.0e-3f,mLaplacian[i-1][j][k])*mPressure[i-1][j][k];
				pb = mLaplacian[i-1][j][k] < 0.0 ? mPressure[i-1][j][k] : mLaplacian[i-1][j][k]/min(1.0e-6f,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[0][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;

	FOR_EVERY_Y_FLOW(mGridSize) {
		if( j>0 && j<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i][j-1][k];
			if(mLaplacian[i][j][k] * mLaplacian[i][j-1][k] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/min(1.0e-3f,mLaplacian[i][j-1][k])*mPressure[i][j-1][k];
				pb = mLaplacian[i][j-1][k] < 0.0 ? mPressure[i][j-1][k] : mLaplacian[i][j-1][k]/min(1.0e-6f,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[1][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;

	FOR_EVERY_Z_FLOW(mGridSize) {
		if( k>0 && k<mGridSize ) {
			float pf = mPressure[i][j][k];
			float pb = mPressure[i][j][k-1];
			if(mLaplacian[i][j][k] * mLaplacian[i][j][k-1] < 0.0 ) {
				pf = mLaplacian[i][j][k] < 0.0 ? mPressure[i][j][k] : mLaplacian[i][j][k]/min(1.0e-3f,mLaplacian[i][j][k-1])*mPressure[i][j][k-1];
				pb = mLaplacian[i][j][k-1] < 0.0 ? mPressure[i][j][k-1] : mLaplacian[i][j][k-1]/min(1.0e-6f,mLaplacian[i][j][k])*mPressure[i][j][k];
			}
			mVelocity[2][i][j][k] -= (pf-pb)/h;
		}
	} END_FOR;
}
void FluidSimulation::extrapolateVelocity() {
	// Mark Fluid Cell Face
	MACGrid<char> mark(Coord(mGridSize),Coord(0),0);
	MACGrid<char> wall_mark(Coord(mGridSize),Coord(0),0);
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
void FluidSimulation::solvePicFlip() {
    // Map Particles Onto Grid
	std::cout<<"Update particles  ..."<<std::endl;
	mParticleLocator->update(mParticles);

	std::cout<<"Map particles to grid ..."<<std::endl;
	mapParticlesToGrid(mParticleLocator.get(),mParticles,mVelocity,mGridSize);
	std::cout<<"Mark water ..."<<std::endl;
	mParticleLocator->markAsWater(mLabel,mWallWeight,mFluidParticleDensity);
	// Solve Fluid Velocity On Grid
	copyGridToBuffer();
	std::cout<<"Enforce boundary condition ..."<<std::endl;
	enforceBoundaryCondition();
	std::cout<<"Project ..."<<std::endl;
	project();
	std::cout<<"Enforce boundary condition ..."<<std::endl;
	enforceBoundaryCondition();
	std::cout<<"Extrapolate velocity ..."<<std::endl;
	extrapolateVelocity();
	std::cout<<"Subtract grid ..."<<std::endl;
	subtractGrid();

	std::cout<<"Copy velocity ..."<<std::endl;

	// Copy Current Velocity
	OPENMP_FOR for(ParticlePtr& p:mParticles) {
		p->mTmp[0] = p->mVelocity;
	}
	// Map Changes Back To Particles
	mapGridToParticles(mParticles,mVelocityLast,mGridSize);
	// Set Tmp As FLIP Velocity
	OPENMP_FOR for(ParticlePtr& p:mParticles) {
		p->mTmp[0] = p->mVelocity + p->mTmp[0];
	}
	// Set u[] As PIC Velocity
	mapGridToParticles(mParticles,mVelocity,mGridSize);
	// Interpolate

	std::cout<<"Blend velocity ..."<<std::endl;
	OPENMP_FOR for(ParticlePtr& p:mParticles) {
		p->mVelocity = (1.0-mPicFlipBlendWeight)*p->mVelocity + mPicFlipBlendWeight*p->mTmp[0];
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
        double value = implicit_func( mParticleLocator.get(), p, mFluidParticleDensity);
        if( i==0 || i==mGridSize-1 || j==0 || j==mGridSize-1 || k==0 || k==mGridSize-1 ) {
            value = max(value,0.01);
        }
        mLevelSet[i][j][k] = value*mGridSize;
	} END_FOR
}
void FluidSimulation::computeWallNormals() {
	// Sort Particles
	mParticleLocator->update(mParticles);
	// Compute Wall Normal
	for (ParticlePtr& p:mParticles) {
		int i = clamp(p->mLocation[0] * mGridSize,0.0f,mGridSize-1.0f);
		int j = clamp(p->mLocation[1] * mGridSize,0.0f,mGridSize-1.0f);
		int k = clamp(p->mLocation[2] * mGridSize,0.0f,mGridSize-1.0f);
		mWallNormal[i][j][k] = Vec3f(0.0f);
		p->mNormal= Vec3f(0.0);
		if (p->mObjectType == WALL) {
			if (p->mLocation[0] <= 1.1 * mWallThickness) {
				p->mNormal[0] = 1.0;
			}
			if (p->mLocation[0] >= 1.0 - 1.1 * mWallThickness) {
				p->mNormal[0] = -1.0;
			}
			if (p->mLocation[1] <= 1.1 * mWallThickness) {
				p->mNormal[1] = 1.0;
			}
			if (p->mLocation[1] >= 1.0 - 1.1 * mWallThickness) {
				p->mNormal[1] = -1.0;
			}
			if (p->mLocation[2] <= 1.1 * mWallThickness) {
				p->mNormal[2] = 1.0;
			}
			if (p->mLocation[2] >= 1.0 - 1.1 * mWallThickness) {
				p->mNormal[2] = -1.0;
			}

			if (p->mNormal[0] == 0.0 && p->mNormal[1] == 0.0 && p->mNormal[2] == 0.0) {
				vector<FluidParticle*> neighbors =
						mParticleLocator->getNeigboringCellParticles(i, j, k, 3, 3, 3);
				for (int n = 0; n < neighbors.size(); n++) {
					FluidParticle *np = neighbors[n];
					if (p.get() != np && np->mObjectType == WALL) {
						float d = length(p->mLocation, np->mLocation);
						float w = 1.0 / d;
						p->mNormal += w * (p->mLocation - np->mLocation) / d;
					}
				}
			}
		}
		p->mNormal.normalize();
		mWallNormal[i][j][k] = p->mNormal;
	}

	mParticleLocator->update(mParticles);
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDensity);

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
