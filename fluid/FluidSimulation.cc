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
#include "FluidSimulation.h"
#include "fluid_utility.h"
#include "laplace_solver.h"
#include "../ImageSciUtil.h"

#include <sstream>
#include <openvdb/openvdb.h>
#include <openvdb/math/Math.h>
#include <openvdb/tools/GridOperators.h>
using namespace openvdb;
using namespace openvdb::tools;
using namespace openvdb::math;
using namespace std;
namespace imagesci {
namespace fluid {
const float FluidSimulation::GRAVITY = 9.8067f;
const float RELAXATION_KERNEL_WIDTH=1.4;
const float SPRING_STIFFNESS=50.0;

FluidSimulation::FluidSimulation(const openvdb::Coord& dims, float voxelSize,
		MotionScheme scheme) :
		Simulation("Fluid Simulation", scheme), mMaxDensity(0.0), mStuckParticleCount(
				0), mPicFlipBlendWeight(0.95f), mFluidParticleDiameter(0.5f), mVoxelSize(
				voxelSize), mGridSize(dims), mWallNormal(dims, voxelSize,openvdb::Vec3s(0.0)),
				mLevelSet(Coord(dims[0] * 2, dims[1] * 2, dims[2] * 2), 0.5f * voxelSize,0.0f),
				mDenseMap(Coord(dims[0] * 2, dims[1] * 2, dims[2] * 2), 0.5f * voxelSize,openvdb::Vec3s(0.0f)),
				mLabel(dims, voxelSize), mLaplacian(dims, voxelSize), mDivergence(dims,
				voxelSize), mPressure(dims, voxelSize), mVelocity(dims,
				voxelSize), mVelocityLast(dims, voxelSize), mWallWeight(dims,
				voxelSize),
				mDistanceField(dims[0] * 2, dims[1] * 2, dims[2] * 2){
	mWallThickness = voxelSize;
	srand(52372143L);
	mTimeStep = 0.5 * mVoxelSize;
	mSimulationDuration = 4.0f; //5 seconds max?
}
void FluidSimulation::computeParticleDensity(float maxDensity) {
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		if (p->mObjectType == WALL) {
			p->mDensity = 1.0;
			continue;
		}
		Vec3f& pt = p->mLocation;
		int i = clamp((int) (mGridSize[0] * p->mLocation[0]), 0,
				mGridSize[0] - 1);
		int j = clamp((int) (mGridSize[1] * p->mLocation[1]), 0,
				mGridSize[1] - 1);
		int k = clamp((int) (mGridSize[2] * p->mLocation[2]), 0,
				mGridSize[2] - 1);
		//Search in small region
		std::vector<FluidParticle*> neighbors =
				mParticleLocator->getNeigboringCellParticles(i, j, k, 1, 1, 1);
		float wsum = 0.0;
		//Density a function of how close particles are to their neighbors.
		for (FluidParticle* np : neighbors) {
			if (np->mObjectType == WALL)
				continue;
			float d2 = distanceSquared(np->mLocation, pt);
			float w = np->mMass
					* smoothKernel(d2,
							4.0f * mFluidParticleDiameter * mVoxelSize);
			wsum += w;
		}
		//Estimate density in region using current particle configuration.
		p->mDensity = wsum / maxDensity;
	}
}
void FluidSimulation::placeWalls() {
	CollisionObject obj;
	// Left Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(0.0, 0.0, 0.0);
	obj.mBounds[1] = Vec3f(mWallThickness, 1.0, 1.0);
	mCollisionObjects.push_back(obj);

	// Right Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(1.0 - mWallThickness, 0.0, 0.0);
	obj.mBounds[1] = Vec3f(1.0, 1.0, 1.0);
	mCollisionObjects.push_back(obj);

	// Floor Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GRAY;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(0.0, 0.0, 0.0);
	obj.mBounds[1] = Vec3f(1.0, mWallThickness, 1.0);
	mCollisionObjects.push_back(obj);

	// Ceiling Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(0.0, 1.0 - mWallThickness, 0.0);
	obj.mBounds[1] = Vec3f(1.0, 1.0, 1.0);
	mCollisionObjects.push_back(obj);

	// Front Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(0.0, 0.0, 0.0);
	obj.mBounds[1] = Vec3f(1.0, 1.0, mWallThickness);
	mCollisionObjects.push_back(obj);

	// Back Wall
	obj.type = WALL;
	obj.shape = BOX;
	obj.material = GLASS;
	obj.mVisible = 0;
	obj.mBounds[0] = Vec3f(0.0, 0.0, 1.0 - mWallThickness);
	obj.mBounds[1] = Vec3f(1.0, 1.0, 1.0);
	mCollisionObjects.push_back(obj);
}
void FluidSimulation::damBreakTest() {
	//replace with level set for falling object
	CollisionObject obj;
	obj.type = FLUID;
	obj.shape = BOX;
	obj.mVisible = true;
	obj.mBounds[0] = Vec3f(0.2, mWallThickness, 0.2);
	obj.mBounds[1] = Vec3f(0.4, 0.4, 0.8);
	mCollisionObjects.push_back(obj);
	obj.type = FLUID;
	obj.shape = BOX;
	obj.mVisible = true;
	obj.mBounds[0] = Vec3f(mWallThickness, mWallThickness, mWallThickness);
	obj.mBounds[1] = Vec3f(1.0 - mWallThickness, 0.06, 1.0 - mWallThickness);
	mCollisionObjects.push_back(obj);

}
void FluidSimulation::placeObjects() {
	placeWalls();
	damBreakTest();
}
void FluidSimulation::repositionParticles(vector<int>& indices) {
	if (indices.empty())
		return;
	// First Search for Deep Water
	vector<Coord> waters;
	while (waters.size() < indices.size()) {
		FOR_EVERY_GRID_CELL(mLabel)
			{
				if (i > 0 && mLabel(i - 1, j, k) != FLUID)
					continue;
				if (i < mGridSize[0] - 1 && mLabel(i + 1, j, k) != FLUID)
					continue;
				if (j > 0 && mLabel(i, j - 1, k) != FLUID)
					continue;
				if (j < mGridSize[1] - 1 && mLabel(i, j + 1, k) != FLUID)
					continue;
				if (k > 0 && mLabel(i, j, k - 1) != FLUID)
					continue;
				if (k < mGridSize[2] - 1 && mLabel(i, j, k + 1) != FLUID)
					continue;
				if (mLabel(i, j, k) != FLUID)
					continue;

				Coord aPos(i, j, k);
				waters.push_back(aPos);
				if (waters.size() >= indices.size()) {
					i = mGridSize[0];
					j = mGridSize[1];
					k = mGridSize[2];
				}
			}END_FOR;
		if (waters.empty())
			return;
	}
	// Shuffle
	shuffleCoordinates(waters);
	for (int n = 0; n < indices.size(); n++) {
		ParticlePtr& p = mParticles[indices[n]];
		p->mLocation[0] = mVoxelSize
				* (waters[n][0] + 0.25 + 0.5 * (rand() % 101) / 100);
		p->mLocation[1] = mVoxelSize
				* (waters[n][1] + 0.25 + 0.5 * (rand() % 101) / 100);
		p->mLocation[2] = mVoxelSize
				* (waters[n][2] + 0.25 + 0.5 * (rand() % 101) / 100);
	}
	mParticleLocator->update(mParticles);
	for (int n = 0; n < indices.size(); n++) {
		ParticlePtr &p = mParticles[indices[n]];
		Vec3f u(0.0);
		resampleParticles(mParticleLocator.get(), p->mLocation, u, mVoxelSize);
		p->mVelocity = u;
	}
}
void FluidSimulation::addParticle(openvdb::Vec3s pt, openvdb::Vec3s center,
		char type) {
	CollisionObject *inside_obj = NULL;
	const int MAX_INT = std::numeric_limits<int>::max();
	Vec3s axis(((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f,
			((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f,
			((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f);
	axis.normalize(1E-6f);
	const float MAX_ANGLE = 30.0f * M_PI / 180.0f;
	Mat3s R = rotation<Mat3s>(axis,
			MAX_ANGLE * (rand() % MAX_INT) / (MAX_INT - 1.0));
	float x = pt[0];
	float y = pt[1];
	float z = pt[2];
	for (CollisionObject &obj : mCollisionObjects) {
		bool found = false;
		float thickness = 3.0 * mVoxelSize;
		if (obj.shape == BOX) {
			if (x > obj.mBounds[0][0] && x < obj.mBounds[1][0]
					&& y > obj.mBounds[0][1] && y < obj.mBounds[1][1]
					&& z > obj.mBounds[0][2] && z < obj.mBounds[1][2]) {
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
			float len = distance(pt, obj.mCenter);
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

		if (inside_obj->type == FLUID) {
			p->mLocation = center + R * (pt - center);
		} else {
			p->mLocation = pt;
		}
		p->mVelocity = Vec3f(0.0);
		p->mNormal = Vec3f(0.0);
		//p->mThinParticle = 0;
		p->mDensity = 10.0;
		p->mObjectType = inside_obj->type;
		//p->mVisible = inside_obj->mVisible;
		p->mMass = 1.0;
		mParticles.push_back(ParticlePtr(p));
	}
}
bool FluidSimulation::init() {
	mSimulationTime = 0;
	mSimulationIteration = 0;
#ifdef MP
	static int procs = omp_get_num_procs();
	std::cout << "Number of OpenMP threads: " << procs << std::endl;
#endif

	mParticleLocator = std::unique_ptr<ParticleLocator>(
			new ParticleLocator(mGridSize, mVoxelSize));
	placeObjects();

	// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
	float h = mFluidParticleDiameter * mVoxelSize;
	FOR_EVERY_CELL(10,10,10)
		{
			FluidParticle *p = new FluidParticle;
			p->mLocation = Vec3f((i + 0.5) * h, (j + 0.5) * h, (k + 0.5) * h);
			p->mObjectType = FLUID;
			p->mMass = 1.0;
			mParticles.push_back(std::unique_ptr<FluidParticle>(p));
		}END_FOR
	mParticleLocator->update(mParticles);
	computeParticleDensity(1.0f);
	mMaxDensity = 0.0;
	for (ParticlePtr& p : mParticles) {
		mMaxDensity = max(mMaxDensity, p->mDensity);
	}
	mParticles.clear();
	// Place Fluid Particles And Walls
	double w = mFluidParticleDiameter * mVoxelSize;
	Vec3s center;
	Vec3s pt;
	FOR_EVERY_GRID_CELL(mLabel)
		{
			for (int ii = 0; ii < 2; ii++) {
				for (int jj = 0; jj < 2; jj++) {
					for (int kk = 0; kk < 2; kk++) {
						double x = w * (2 * i + ii + 0.5);
						double y = w * (2 * j + jj + 0.5);
						double z = w * (2 * k + kk + 0.5);
						if (x > mWallThickness && x < 1.0 - mWallThickness
								&& y > mWallThickness
								&& y < 1.0 - mWallThickness
								&& z > mWallThickness
								&& z < 1.0 - mWallThickness) {

							center = Vec3s(w * (2 * i + 1), w * (2 * j + 1),
									w * (2 * k + 1));
							addParticle(Vec3s(x, y, z), center, FLUID);
						}
					}
				}
			}
		}END_FOR;
// Place Wall Particles And Walls
	w = 2 * mFluidParticleDiameter * mVoxelSize;
	FOR_EVERY_GRID_CELL(mLabel)
		{
			double x = i * w + w * 0.5;
			double y = j * w + w * 0.5;
			double z = k * w + w * 0.5;
			addParticle(Vec3s(x, y, z), Vec3s(x, y, z), WALL);
		}END_FOR;
	mParticleLocator->update(mParticles);
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDiameter);

// Remove Particles That Stuck On Wal Cells
	for (std::vector<ParticlePtr>::iterator iter = mParticles.begin();
			iter != mParticles.end();) {
		ParticlePtr& p = *iter;
		if (p->mObjectType == WALL) {
			iter++;
			continue;
		}
		int i = clamp((int) (mGridSize[0] * p->mLocation[0]), 0,
				mGridSize[0] - 1);
		int j = clamp((int) (mGridSize[1] * p->mLocation[1]), 0,
				mGridSize[1] - 1);
		int k = clamp((int) (mGridSize[2] * p->mLocation[2]), 0,
				mGridSize[2] - 1);
		if (mLabel(i, j, k) == WALL) {
			iter = mParticles.erase(iter);
		} else {
			iter++;
		}
	}
// Comput Normal for Walls
	computeWallNormals();
	createLevelSet();
	updateParticleVolume();
	mSource.create(mLevelSet);
	if(mMotionScheme==IMPLICIT){
		mSource.mConstellation.reset();
	} else {
		mAdvect=std::unique_ptr<SpringLevelSetParticleDeformation<openvdb::util::NullInterrupter> >(new SpringLevelSetParticleDeformation<openvdb::util::NullInterrupter>(mSource,mMotionScheme));
		mAdvect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK1);
		if(mMotionScheme!=IMPLICIT){
			std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
	#pragma omp for
			for(int n=0;n<velocities.size();n++){
				velocities[n]=Vec3s(0.0);
			}
		}
	}
	return true;
}
void FluidSimulation::pourWater(int limit, float maxDensity) {
	if (mSimulationIteration > limit)
		return;
	Vec2f mPourPosition(0.0, 0.0);
	float mPourRadius(0.12);
	int cnt = 0;
	double w = mFluidParticleDiameter * mVoxelSize;
	for (float x = w + w / 2.0; x < 1.0 - w / 2.0; x += w) {
		for (float z = w + w / 2.0; z < 1.0 - w / 2.0; z += w) {
			if (hypot(x - mPourPosition[0], z - mPourPosition[1])
					< mPourRadius) {
				FluidParticle *p = new FluidParticle;
				p->mLocation = Vec3f(x,
						1.0 - mWallThickness
								- 2.5 * mFluidParticleDiameter * mVoxelSize, z);
				p->mVelocity = Vec3f(0.0,
						-0.5 * mVoxelSize * mFluidParticleDiameter / mTimeStep,
						0.0);
				p->mNormal = Vec3f(0.0);
				p->mObjectType = FLUID;
				p->mDensity = maxDensity;
				p->mMass = 1.0;
				mParticles.push_back(ParticlePtr(p));
				cnt++;
			}
		}
	}
}
void FluidSimulation::addExternalForce() {
	float velocity = -mTimeStep * GRAVITY;
	//Add graviy acceleration to all particles
	int count=0;
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		if (mParticles[n]->mObjectType == FLUID){
			mParticles[n]->mVelocity[1] += velocity;
			count++;
		}
	}
	std:cout<<"Velocity "<<velocity<<" Count "<<count<<" "<<mParticles.size()<<std::endl;

	if(mMotionScheme!=IMPLICIT){
		//Add velocity to surface particles
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
#pragma omp for
		for(int n=0;n<velocities.size();n++){
			velocities[n]+=Vec3s(0.0,velocity,0.0);
		}
	}
}
void FluidSimulation::advectParticles() {
// Advect Particle Through Grid
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		if (p->mObjectType == FLUID) {
			p->mLocation += mTimeStep * mVelocity.interpolate(p->mLocation);
		}
	}
	//WriteToRawFile(mVelocity,"velocity.xml");
	mParticleLocator->update(mParticles);
// Constraint Outer Wall
	for (ParticlePtr& p : mParticles) {
		float r = mWallThickness;
		for (int k = 0; k < 3; k++) {
			if (p->mObjectType == FLUID) {
				p->mLocation[k] = clamp(p->mLocation[k], r, 1.0f - r);
			}
		}
		if (p->mObjectType == FLUID) {
			int i = clamp((int) (p->mLocation[0] * mGridSize[0]), 0,
					mGridSize[0] - 1);
			int j = clamp((int) (p->mLocation[1] * mGridSize[1]), 0,
					mGridSize[1] - 1);
			int k = clamp((int) (p->mLocation[2] * mGridSize[2]), 0,
					mGridSize[2] - 1);
			vector<FluidParticle*> neighbors =
					mParticleLocator->getNeigboringCellParticles(i, j, k, 1, 1,
							1);
			for (int n = 0; n < neighbors.size(); n++) {
				FluidParticle *np = neighbors[n];
				double re = 1.5 * mFluidParticleDiameter * mVoxelSize;
				if (np->mObjectType == WALL) {
					float dist = distance(p->mLocation, np->mLocation);
					if (dist < re) {
						float normal[3] = { np->mNormal[0], np->mNormal[1],
								np->mNormal[2] };
						if (normal[0] == 0.0 && normal[1] == 0.0
								&& normal[2] == 0.0 && dist) {
							for (int c = 0; c < 3; c++)
								normal[c] = (p->mLocation[c] - np->mLocation[c])
										/ dist;
						}

						p->mLocation[0] += (re - dist) * normal[0];
						p->mLocation[1] += (re - dist) * normal[1];
						p->mLocation[2] += (re - dist) * normal[2];
						float dot = p->mVelocity[0] * normal[0]
								+ p->mVelocity[1] * normal[1]
								+ p->mVelocity[2] * normal[2];
						p->mVelocity[0] -= dot * normal[0];
						p->mVelocity[1] -= dot * normal[1];
						p->mVelocity[2] -= dot * normal[2];
					}
				}
			}
		}
	}
	int i, j, k;
// Remove Particles That Stuck On The Up-Down Wall Cells...
	for (ParticlePtr& p : mParticles) {
		p->mRemoveIndicator = false;
		// Focus on Only Fluid Particle
		if (p->mObjectType != FLUID) {
			continue;
		}

		i = clamp((int) (p->mLocation[0] * mGridSize[0]), 0, mGridSize[0] - 1);
		j = clamp((int) (p->mLocation[1] * mGridSize[1]), 0, mGridSize[1] - 1);
		k = clamp((int) (p->mLocation[2] * mGridSize[2]), 0, mGridSize[2] - 1);
		// If Stuck On Wall Cells Just Repositoin
		if (mLabel(i, j, k) == WALL) {
			p->mRemoveIndicator = true;
		}
		i = clamp((int) (p->mLocation[0] * mGridSize[0]), 2, mGridSize[0] - 3);
		j = clamp((int) (p->mLocation[1] * mGridSize[1]), 2, mGridSize[1] - 3);
		k = clamp((int) (p->mLocation[2] * mGridSize[2]), 2, mGridSize[2] - 3);
		if (p->mDensity < 0.04
				&& (mLabel(i, max(0, j - 1), k) == WALL
						|| mLabel(i, min(mGridSize[1] - 1, j + 1), k) == WALL)) {
			// Put Into Reposition List
			p->mRemoveIndicator = true;
		}

	}
// Reposition If Neccessary
	vector<int> reposition_indices;
	size_t n = 0;
	for (ParticlePtr& p : mParticles) {
		if (p->mRemoveIndicator) {
			p->mRemoveIndicator = false;
			reposition_indices.push_back(n);
		}
		n++;
	}

// Store Stuck Particle Number
	mStuckParticleCount = reposition_indices.size();
	repositionParticles(reposition_indices);
}
void FluidSimulation::cleanup() {
	mParticles.clear();
}
bool FluidSimulation::step() {
	//Rebuild location data structure
	mParticleLocator->update(mParticles);
	//Compute density for each cell, capped by max density as pre-computed
	computeParticleDensity(mMaxDensity);
	//Add external gravity force
	addExternalForce();
	solvePicFlip();	
	if(mMotionScheme==MotionScheme::IMPLICIT){
		advectParticles();
	} else {
		std::vector<Vec3s>& positions=mSource.mConstellation.mParticles;
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
#pragma omp for
		for(int n=0;n<positions.size();n++){
			// Variables for Neighboring Particles
			float wsum = 0.0;
			Vec3s u(0.0f);
			Vec3s pt=positions[n];
			// Gather Neighboring Particles
			std::vector<FluidParticle*> neighbors = mParticleLocator->getNeigboringCellParticles((int)(0.5f*pt[0]),(int)(0.5f*pt[1]),(int)(0.5f*pt[2]),1,1,1);
			for(FluidParticle *np:neighbors) {
				if( np->mObjectType == FLUID ) {
					float dist2 = distanceSquared(0.5f*mVoxelSize*pt,np->mLocation);
					float w = np->mMass * sharpKernel(dist2,mFluidParticleDiameter * mVoxelSize);
					u += w * np->mVelocity;
					wsum += w;
				}
			}
			if(wsum>0.0f) {
				u /= wsum;
				velocities[n]=u;
			}
		}
		advectParticles();
		mAdvect->advect(mSimulationTime,mSimulationTime+mTimeStep);
		Coord dims=mLevelSet.dimensions();
		std::vector<Vec3s>& positions2=mSource.mConstellation.mParticles;
#pragma omp for
		for(int n=0;n<positions2.size();n++){
			Vec3s& pt=positions2[n];
			pt[0]=clamp(pt[0],1.0f,dims[0]-2.0f);
			pt[1]=clamp(pt[1],1.0f,dims[1]-2.0f);
			pt[2]=clamp(pt[2],1.0f,dims[2]-2.0f);
		}
		std::vector<Vec3s>& vertexes=mSource.mConstellation.mVertexes;
#pragma omp for
		for(int n=0;n<vertexes.size();n++){
			Vec3s& pt=vertexes[n];
			pt[0]=clamp(pt[0],1.0f,dims[0]-2.0f);
			pt[1]=clamp(pt[1],1.0f,dims[1]-2.0f);
			pt[2]=clamp(pt[2],1.0f,dims[2]-2.0f);
		}
	}
	correctParticles(mParticleLocator.get(), mParticles, mTimeStep,mFluidParticleDiameter * mVoxelSize);
	updateParticleVolume();
	createLevelSet();
	if(mMotionScheme==MotionScheme::IMPLICIT){
		openvdb::tools::copyFromDense(mLevelSet,*mSource.mSignedLevelSet,0.25);
		mSource.updateIsoSurface();
	}
	//RegularGrid<float> distField(mLevelSet.dimensions(),1.0f,0.0);
	//mDistanceField.solve(mLevelSet,distField,8.0f);
	//mSparseLevelSet->clear();
	//stringstream velFile,levelFile,unsignedFile,mapFile,distFile;
	//velFile<<"/home/blake/tmp/velocity" <<std::setw(8)<<std::setfill('0')<< mSimulationIteration;
	//levelFile<<"/home/blake/tmp/levelset" <<std::setw(8)<<std::setfill('0')<< mSimulationIteration;
	//distFile<<"/home/blake/tmp/distfield" <<std::setw(8)<<std::setfill('0')<< mSimulationIteration;
	//unsignedFile<<"/home/blake/tmp/unsigned" <<std::setw(8)<<std::setfill('0')<< mSimulationIteration;
	//mapFile<<"/home/blake/tmp/densemap" <<std::setw(8)<<std::setfill('0')<< mSimulationIteration;
	//WriteToRawFile(mVelocity,"/home/blake/tmp/velocity");
	//imagesci::WriteToRawFile(mLevelSet,levelFile.str());
	//imagesci::WriteToRawFile(distField,distFile.str());
	//imagesci::WriteToRawFile(mSource.mUnsignedLevelSet,unsignedFile.str());
	//imagesci::WriteToRawFile(mDenseMap,mapFile.str());
	//imagesci::fluid::WriteToRawFile(mVelocity,velFile.str());

	mSimulationIteration++;
	mSimulationTime = mSimulationIteration * mTimeStep;
	if (mSimulationTime <= mSimulationDuration && mRunning) {
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
	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[0])
		{
			if (i == 0 || i == mGridSize[0])
				mVelocity[0](i, j, k) = 0.0;
			if (i < mGridSize[0] && i > 0
					&& isWallIndicator(mLabel(i, j, k))
							* isWallIndicator(mLabel(i - 1, j, k)) < 0) {
				mVelocity[0](i, j, k) = 0.0;
			}
		}END_FOR

	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[1])
		{
			if (j == 0 || j == mGridSize[1])
				mVelocity[1](i, j, k) = 0.0;
			if (j < mGridSize[1] && j > 0
					&& isWallIndicator(mLabel(i, j, k))
							* isWallIndicator(mLabel(i, j - 1, k)) < 0) {
				mVelocity[1](i, j, k) = 0.0;
			}
		}END_FOR

	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[2])
		{
			if (k == 0 || k == mGridSize[2])
				mVelocity[2](i, j, k) = 0.0;
			if (k < mGridSize[2] && k > 0
					&& isWallIndicator(mLabel(i, j, k))
							* isWallIndicator(mLabel(i, j, k - 1)) < 0) {
				mVelocity[2](i, j, k) = 0.0;
			}
		}END_FOR
}
void FluidSimulation::project() {
// Cell Width
// Compute Divergence
	OPENMP_FOR FOR_EVERY_GRID_CELL(mLabel)
		{
			if (mLabel(i, j, k) == FLUID) {
				mDivergence(i, j, k) = (mVelocity[0](i + 1, j, k)
						- mVelocity[0](i, j, k) + mVelocity[1](i, j + 1, k)
						- mVelocity[1](i, j, k) + mVelocity[2](i, j, k + 1)
						- mVelocity[2](i, j, k)) / mVoxelSize;
			}
		}END_FOR;

// Compute LevelSet
	OPENMP_FOR FOR_EVERY_GRID_CELL(mLaplacian)
		{
			mLaplacian(i, j, k) = mParticleLocator->getLevelSetValue(i, j, k,
					mWallWeight, mFluidParticleDiameter);
		}END_FOR;
	//WriteToRawFile(mPressure,"/home/blake/tmp/pressure");
	//WriteToRawFile(mDivergence,"/home/blake/tmp/divergence");
	//WriteToRawFile(mVelocity,"/home/blake/tmp/velocity");

	laplace_solve(mLabel, mLaplacian, mPressure, mDivergence);
// Subtract Pressure Gradient
	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[0])
		{
			if (i > 0 && i < mGridSize[0]) {
				float pf = mPressure(i, j, k);
				float pb = mPressure(i - 1, j, k);
				if (mLaplacian(i, j, k) * mLaplacian(i - 1, j, k) < 0.0) {
					pf = mLaplacian(i, j, k) < 0.0 ?
							mPressure(i, j, k) :
							mLaplacian(i, j, k)
									/ min(1.0e-3f, mLaplacian(i - 1, j, k))
									* mPressure(i - 1, j, k);
					pb = mLaplacian(i - 1, j, k) < 0.0 ?
							mPressure(i - 1, j, k) :
							mLaplacian(i - 1, j, k)
									/ min(1.0e-6f, mLaplacian(i, j, k))
									* mPressure(i, j, k);
				}
				mVelocity[0](i, j, k) -= (pf - pb) / mVoxelSize;
			}
		}END_FOR;

	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[1])
		{
			if (j > 0 && j < mGridSize[1]) {
				float pf = mPressure(i, j, k);
				float pb = mPressure(i, j - 1, k);
				if (mLaplacian(i, j, k) * mLaplacian(i, j - 1, k) < 0.0) {
					pf = mLaplacian(i, j, k) < 0.0 ?
							mPressure(i, j, k) :
							mLaplacian(i, j, k)
									/ min(1.0e-3f, mLaplacian(i, j - 1, k))
									* mPressure(i, j - 1, k);
					pb = mLaplacian(i, j - 1, k) < 0.0 ?
							mPressure(i, j - 1, k) :
							mLaplacian(i, j - 1, k)
									/ min(1.0e-6f, mLaplacian(i, j, k))
									* mPressure(i, j, k);
				}
				mVelocity[1](i, j, k) -= (pf - pb) / mVoxelSize;
			}
		}END_FOR;
	OPENMP_FOR FOR_EVERY_GRID_CELL(mVelocity[2])
		{
			if (k > 0 && k < mGridSize[2]) {
				float pf = mPressure(i, j, k);
				float pb = mPressure(i, j, k - 1);
				if (mLaplacian(i, j, k) * mLaplacian(i, j, k - 1) < 0.0) {
					pf = mLaplacian(i, j, k) < 0.0 ?
							mPressure(i, j, k) :
							mLaplacian(i, j, k)
									/ min(1.0e-3f, mLaplacian(i, j, k - 1))
									* mPressure(i, j, k - 1);
					pb = mLaplacian(i, j, k - 1) < 0.0 ?
							mPressure(i, j, k - 1) :
							mLaplacian(i, j, k - 1)
									/ min(1.0e-6f, mLaplacian(i, j, k))
									* mPressure(i, j, k);
				}
				mVelocity[2](i, j, k) -= (pf - pb) / mVoxelSize;
			}
		}END_FOR;
}
void FluidSimulation::extrapolateVelocity() {
// Mark Fluid Cell Face
	MACGrid<char> mark(mGridSize, mVoxelSize);
	MACGrid<char> wall_mark(mGridSize, mVoxelSize);
	OPENMP_FOR FOR_EVERY_GRID_CELL(mark[0])
		{
			mark[0](i, j, k) = (i > 0 && mLabel(i - 1, j, k) == FLUID)|| (i < mGridSize[0] && mLabel(i, j, k) == FLUID);
			wall_mark[0](i, j, k) = (i <= 0 || mLabel(i - 1, j, k) == WALL)&& (i >= mGridSize[0] || mLabel(i, j, k) == WALL);
		}END_FOR;

	OPENMP_FOR FOR_EVERY_GRID_CELL(mark[1])
		{
			mark[1](i, j, k) = (j > 0 && mLabel(i, j - 1, k) == FLUID)|| (j < mGridSize[1] && mLabel(i, j, k) == FLUID);
			wall_mark[1](i, j, k) = (j <= 0 || mLabel(i, j - 1, k) == WALL)&& (j >= mGridSize[1] || mLabel(i, j, k) == WALL);
		}END_FOR;

	OPENMP_FOR FOR_EVERY_GRID_CELL(mark[2])
		{
			mark[2](i, j, k) = (k > 0 && mLabel(i, j, k - 1) == FLUID)|| (k < mGridSize[2] && mLabel(i, j, k) == FLUID);
			wall_mark[2](i, j, k) = (k <= 0 || mLabel(i, j, k - 1) == WALL)&& (k >= mGridSize[2] || mLabel(i, j, k) == WALL);
		}END_FOR;

// Now Extrapolate
	OPENMP_FOR FOR_EVERY_CELL(mGridSize[0]+1,mGridSize[1]+1,mGridSize[2]+1)
		{
			for (int n = 0; n < 3; n++) {
				if (n != 0 && i > mGridSize[0] - 1)
					continue;
				if (n != 1 && j > mGridSize[1] - 1)
					continue;
				if (n != 2 && k > mGridSize[2] - 1)
					continue;
				if (!mark[n](i, j, k) && wall_mark[n](i, j, k)) {
					int wsum = 0;
					float sum = 0.0;
					int q[][3] = { { i - 1, j, k }, { i + 1, j, k }, { i, j - 1,
							k }, { i, j + 1, k }, { i, j, k - 1 },
							{ i, j, k + 1 } };
					for (int qk = 0; qk < 6; qk++) {
						if (q[qk][0] >= 0
								&& q[qk][0] < mGridSize[0] + ((n == 0) ? 1 : 0)
								&& q[qk][1] >= 0
								&& q[qk][1] < mGridSize[1] + ((n == 1) ? 1 : 0)
								&& q[qk][2] >= 0
								&& q[qk][2]
										< mGridSize[2] + ((n == 2) ? 1 : 0)) {
							if (mark[n](q[qk][0], q[qk][1], q[qk][2])) {
								wsum++;
								sum += mVelocity[n](q[qk][0], q[qk][1],
										q[qk][2]);
							}
						}
					}
					if (wsum)
						mVelocity[n](i, j, k) = sum / wsum;
				}
			}
		}END_FOR;
}
void FluidSimulation::solvePicFlip() {
	mParticleLocator->update(mParticles);
	mapParticlesToGrid();
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDiameter);
	copyGridToBuffer();
	enforceBoundaryCondition();
	project();
	enforceBoundaryCondition();
	extrapolateVelocity();

	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		openvdb::Vec3s currentVelocity=mVelocity.interpolate(p->mLocation);
		openvdb::Vec3s velocity=p->mVelocity+currentVelocity-mVelocityLast.interpolate(p->mLocation);
		p->mVelocity = (1.0 - mPicFlipBlendWeight) *currentVelocity  + mPicFlipBlendWeight * velocity;
	}
	if(mMotionScheme!=IMPLICIT){
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
		std::vector<Vec3s>& positions=mSource.mConstellation.mParticles;
#pragma omp for
		for(int n=0;n<velocities.size();n++){
			Vec3s pt=0.5f*mVoxelSize*positions[n];
			openvdb::Vec3s currentVelocity=mVelocity.interpolate(pt);
			openvdb::Vec3s velocity=velocities[n]+currentVelocity-mVelocityLast.interpolate(pt);
			velocities[n] = (1.0 - mPicFlipBlendWeight) *currentVelocity  + mPicFlipBlendWeight * velocity;
		}
	}
}
void FluidSimulation::updateParticleVolume(){
	//WriteToRawFile(mLevelSet,"/home/blake/tmp/levelset");
	float voxelSize = mLevelSet.voxelSize();
		mSource.mParticleVolume.mParticles.clear();
		float scale = mVoxelSize / voxelSize;
		FOR_EVERY_PARTICLE(mParticles)
		{
			FluidParticle* p = mParticles[n].get();
			if (p->mObjectType == ObjectType::FLUID) {
				Vec3s l = p->mLocation / voxelSize;
				Vec4s v(l[0], l[1], l[2], scale * 0.5f * mFluidParticleDiameter);
				mSource.mParticleVolume.mParticles.push_back(v);
			}
		}
		Coord dims(mLevelSet.rows(), mLevelSet.cols(), mLevelSet.slices());
		mSource.mParticleVolume.setBoundingBox(
				BBoxd(Vec3d(0, 0, 0), Vec3d(dims[0], dims[1], dims[2])));
}
void FluidSimulation::createLevelSet() {
// Create Density Field
	Coord dims(mLevelSet.rows(), mLevelSet.cols(), mLevelSet.slices());
	float voxelSize = mLevelSet.voxelSize();
	OPENMP_FOR FOR_EVERY_GRID_CELL(mLevelSet)
		{
			double x = i * voxelSize;
			double y = j * voxelSize;
			double z = k * voxelSize;
			Vec3f p(x, y, z);
			double value = implicit_func(mParticleLocator.get(), p,
					mFluidParticleDiameter);
			//std::cout<<i<<" "<<j<<" "<<k<<" P "<<p<<" "<<value<<std::endl;
			if (i == 0 || i == dims[0] - 1 || j == 0 || j == dims[1] - 1
					|| k == 0 || k == dims[2] - 1) {
				//value = max(value,0.01);
				mLevelSet(i, j, k) = 0.001;
			} else {
				mLevelSet(i, j, k) = clamp(value / mVoxelSize,
						-openvdb::LEVEL_SET_HALF_WIDTH,
						openvdb::LEVEL_SET_HALF_WIDTH);
			}
		}END_FOR;


}
float FluidSimulation::lengthSquared( float a, float b, float c ) {
    return a*a + b*b + c*c;
}

float FluidSimulation::distanceSquared(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).lengthSqr();
}

float FluidSimulation::distance(const openvdb::Vec3f& p0,const openvdb::Vec3f& p1) {
	return (p0-p1).length();
}

void FluidSimulation::shuffleCoordinates( std::vector<openvdb::Coord> &waters ) {
	random_shuffle( waters.begin(), waters.end() );
}


float FluidSimulation::smoothKernel( float r2, float h ) {
    return max( 1.0-r2/(h*h), 0.0 );
}
float FluidSimulation::sharpKernel( float r2, float h ) {
    return max( h*h/fmax(r2,1.0e-5) - 1.0, 0.0 );
}




void FluidSimulation::mapParticlesToGrid() {

	// Compute Mapping

	openvdb::Coord dims(mVelocity.rows(),mVelocity.cols(),mVelocity.slices());
//OPENMP_FOR
	FOR_EVERY_CELL(dims[0]+1,dims[1]+1,dims[2]+1) {
		// Variales for Particle Sorter
		vector<FluidParticle*> neighbors;

		// Map X Grids
		if( j <dims[1] && k < dims[2]) {
			openvdb::Vec3f px(i, j+0.5, k+0.5);
			float sumw = 0.0;
			float sumx = 0.0;
			neighbors = mParticleLocator->getNeigboringWallParticles(i,j,k,1,2,2);

			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					openvdb::Vec3f pos(x, y, z);
					float w = p->mMass * sharpKernel(distanceSquared(pos,px),RELAXATION_KERNEL_WIDTH);
					sumx += w*p->mVelocity[0];
					sumw += w;
				}
			}
			mVelocity[0](i,j,k) = sumw ? sumx/sumw : 0.0;
		}

		// Map Y Grids
		if( i < dims[0] && k < dims[2] ) {
			openvdb::Vec3f py( i+0.5, j, k+0.5);
			float sumw = 0.0;
			float sumy = 0.0;
			neighbors =mParticleLocator->getNeigboringWallParticles(i,j,k,2,1,2);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					openvdb::Vec3s pos( x, y, z );
					float w = p->mMass * sharpKernel(distanceSquared(pos,py),RELAXATION_KERNEL_WIDTH);
					sumy += w*p->mVelocity[1];
					sumw += w;
				}
			}
			mVelocity[1](i,j,k) = sumw ? sumy/sumw : 0.0;
		}
		// Map Z Grids
		if( i < dims[0] && j < dims[1] ) {
			openvdb::Vec3f pz(i+0.5, j+0.5, k);
			float sumw = 0.0;
			float sumz = 0.0;
			neighbors =mParticleLocator->getNeigboringWallParticles(i,j,k,2,2,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *p = neighbors[n];
				if( p->mObjectType == FLUID ) {
					float x = clamp(dims[0]*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(dims[1]*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(dims[2]*p->mLocation[2],0.0f,(float)dims[2]);
					openvdb::Vec3s pos( x, y, z );
					float w = p->mMass * sharpKernel(distanceSquared(pos,pz),RELAXATION_KERNEL_WIDTH);
					sumz += w*p->mVelocity[2];
					sumw += w;
				}
			}
			mVelocity[2](i,j,k) = sumw ? sumz/sumw : 0.0;
		}
	} END_FOR
}

float FluidSimulation::linear ( RegularGrid<float>& q, float x, float y, float z) {
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

void FluidSimulation::resampleParticles( ParticleLocator *sort, openvdb::Vec3f& p,openvdb::Vec3f& u, float re ) {
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
			float dist2 = distanceSquared(p,np->mLocation);
			float w = np->mMass * sharpKernel(dist2,re);
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

void FluidSimulation::correctParticles( ParticleLocator *sort, std::vector<ParticlePtr>& particles, float dt, float re ) {
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
					float dist = distance(p->mLocation,np->mLocation);
					float w = SPRING_STIFFNESS * np->mMass * smoothKernel(dist*dist,re);
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
void FluidSimulation::mapGridToParticles() {
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles){
		ParticlePtr& p=mParticles[n];
		p->mVelocity=mVelocity.interpolate(p->mLocation);
	}
}
double FluidSimulation::implicit_func( vector<FluidParticle*> &neighbors,openvdb::Vec3f& p, float density,float voxelSize) {
	double phi = 8.0*density*voxelSize;
	for( int m=0; m<neighbors.size(); m++ ) {
		FluidParticle &np = *neighbors[m];
		if( np.mObjectType == WALL ) {
			if( distance(np.mLocation,p) < density*voxelSize) return 4.5*density*voxelSize;
			continue;
		}
		double d = distance(np.mLocation,p);
		if( d < phi ) {
			phi = d;
		}
	}
	return phi - density*voxelSize;
}
double FluidSimulation::implicit_func( ParticleLocator *sort, openvdb::Vec3f& p, float density ) {
	openvdb::Coord cell_size = sort->getCellSize();
	vector<FluidParticle *> neighbors = sort->getNeigboringCellParticles(
			clamp((int)(p[0]*cell_size[0]),0,cell_size[0]-1),
			clamp((int)(p[1]*cell_size[1]),0,cell_size[1]-1),
			clamp((int)(p[2]*cell_size[2]),0,cell_size[2]-1),2,2,2
			);
	return implicit_func( neighbors, p, density,sort->getVoxelSize());
}
void FluidSimulation::computeWallNormals() {
// Sort Particles
	mParticleLocator->update(mParticles);
// Compute Wall Normal
	for (ParticlePtr& p : mParticles) {
		int i = clamp((int) (p->mLocation[0] * mGridSize[0]), 0,
				mGridSize[0] - 1);
		int j = clamp((int) (p->mLocation[1] * mGridSize[1]), 0,
				mGridSize[1] - 1);
		int k = clamp((int) (p->mLocation[2] * mGridSize[2]), 0,
				mGridSize[2] - 1);
		mWallNormal(i, j, k) = Vec3f(0.0f);
		p->mNormal = Vec3f(0.0);

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

			if (p->mNormal[0] == 0.0 && p->mNormal[1] == 0.0
					&& p->mNormal[2] == 0.0) {
				vector<FluidParticle*> neighbors =
						mParticleLocator->getNeigboringCellParticles(i, j, k, 3,
								3, 3);
				for (int n = 0; n < neighbors.size(); n++) {
					FluidParticle *np = neighbors[n];
					if (p.get() != np && np->mObjectType == WALL) {
						float d = distance(p->mLocation, np->mLocation);
						float w = 1.0 / d;
						p->mNormal += w * (p->mLocation - np->mLocation) / d;
					}
				}
			}
		}
		p->mNormal.normalize();
		mWallNormal(i, j, k) = p->mNormal;
	}

	mParticleLocator->update(mParticles);
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDiameter);

// Compute Perimeter Normal
	FOR_EVERY_GRID_CELL(mWallNormal)
		{
			mWallWeight(i, j, k) = 0.0;
			if (mLabel(i, j, k) != WALL) {
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
					if (si < 0 || si > mGridSize[0] - 1 || sj < 0
							|| sj > mGridSize[1] - 1 || sk < 0
							|| sk > mGridSize[2] - 1)
						continue;
					if (mLabel(si, sj, sk) == WALL) {
						sum++;
						norm += mWallNormal(si, sj, sk);
					}
				}
				if (sum > 0) {
					norm.normalize();
					mWallNormal(i, j, k) = norm;
					mWallWeight(i, j, k) = 1.0;
				}
			}
		}END_FOR;
}
FluidSimulation::~FluidSimulation() {
// TODO Auto-generated destructor stub
}
}
} /* namespace imagesci */
