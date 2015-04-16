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
#include <openvdb/tools/Composite.h>
#include <openvdb/tools/GridOperators.h>
#include <openvdb/tools/MeshToVolume.h>
using namespace openvdb;
using namespace openvdb::tools;
using namespace openvdb::math;
using namespace std;
namespace imagesci {
namespace fluid {
const float FluidSimulation::GRAVITY = 9.8067f;
const float RELAXATION_KERNEL_WIDTH=1.4;
const float SPRING_STIFFNESS=50.0;
static int frameCounter=0;
FluidSimulation::FluidSimulation(const openvdb::Coord& dims, float voxelSize,
		MotionScheme scheme) :
		Simulation("Fluid_Simulation", scheme), mMaxDensity(0.0), mStuckParticleCount(
				0), mPicFlipBlendWeight(0.95f), mFluidParticleDiameter(0.5f), mVoxelSize(
				voxelSize), mGridSize(dims), mWallNormal(dims, voxelSize,openvdb::Vec3s(0.0)),
				mParticleLevelSet(Coord(dims[0] * 2, dims[1] * 2, dims[2] * 2), 0.5f * voxelSize,0.0f),
				mDenseMap(Coord(dims[0] * 2, dims[1] * 2, dims[2] * 2), 0.5f * voxelSize,openvdb::Vec3s(0.0f)),
				mLabel(dims, voxelSize), mLaplacian(dims, voxelSize), mDivergence(dims,
				voxelSize), mPressure(dims, voxelSize), mVelocity(dims,
				voxelSize), mVelocityLast(dims, voxelSize), mWallWeight(dims,
				voxelSize),mSpringlTracking(scheme!=IMPLICIT),
				mSignedLevelSet(Coord(dims[0] * 2, dims[1] * 2, dims[2] * 2), 0.5f * voxelSize,0.0f),
				mDistanceField(dims[0] * 2, dims[1] * 2, dims[2] * 2){
	mWallThickness = mVoxelSize;
	srand(52372143L);
	mTimeStep = 0.5 * mVoxelSize;
	mDomainSize=Vec3s(dims[0]*mVoxelSize,dims[1]*mVoxelSize,dims[2]*mVoxelSize);
	mSimulationDuration = 4.0f;
}
void FluidSimulation::computeParticleDensity(float maxDensity) {
	float scale=1.0f/mVoxelSize;
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		if (p->mObjectType == ObjectType::WALL) {
			p->mDensity = 1.0;
			continue;
		}
		Vec3f& pt = p->mLocation;
		int i = clamp((int) (scale * p->mLocation[0]), 0,
				mGridSize[0] - 1);
		int j = clamp((int) (scale * p->mLocation[1]), 0,
				mGridSize[1] - 1);
		int k = clamp((int) (scale* p->mLocation[2]), 0,
				mGridSize[2] - 1);
		//Search in small region
		std::vector<FluidParticle*> neighbors =
				mParticleLocator->getNeigboringCellParticles(i, j, k, 1, 1, 1);
		float wsum = 0.0;
		//Density a function of how close particles are to their neighbors.
		for (FluidParticle* np : neighbors) {
			if (np->mObjectType == ObjectType::WALL)
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
	BoxObject* obj;
	Coord dims=mLabel.dimensions();
	float mx=mVoxelSize*dims[0];
	float my=mVoxelSize*dims[1];
	float mz=mVoxelSize*dims[2];
	obj=new BoxObject;
	obj->mThickness=3*mVoxelSize;
	// Left Wall
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GLASS;
	obj->mVisible = false;
	obj->mMin = Vec3f(0.0, 0.0, 0.0);
	obj->mMax = Vec3f(mWallThickness, my,mz);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));

	obj=new BoxObject;
	// Right Wall
	obj->mThickness=3*mVoxelSize;
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GLASS;
	obj->mVisible = false;
	obj->mMin = Vec3f(mx - mWallThickness, 0.0, 0.0);
	obj->mMax = Vec3f(mx,my,mz);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));

	obj=new BoxObject;
	// Floor Wall
	obj->mThickness=3*mVoxelSize;
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GRAY;
	obj->mVisible = false;
	obj->mMin = Vec3f(0.0, 0.0, 0.0);
	obj->mMax = Vec3f(mx, mWallThickness, mz);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));

	obj=new BoxObject;
	// Ceiling Wall
	obj->mThickness=3*mVoxelSize;
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GLASS;
	obj->mVisible = false;
	obj->mMin = Vec3f(0.0, my - mWallThickness, 0.0);
	obj->mMax = Vec3f(mx,my,mz);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));

	obj=new BoxObject;
	// Front Wall
	obj->mThickness=3*mVoxelSize;
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GLASS;
	obj->mVisible = false;
	obj->mMin = Vec3f(0.0, 0.0, 0.0);
	obj->mMax = Vec3f(mx,my, mWallThickness);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));

	obj=new BoxObject;
	// Back Wall
	obj->mThickness=3*mVoxelSize;
	obj->mType = ObjectType::WALL;
	obj->mMaterial = ObjectMaterial::GLASS;
	obj->mVisible = false;
	obj->mMin = Vec3f(0.0, 0.0, mz - mWallThickness);
	obj->mMax = Vec3f(mx,my,mz);
	obj->mVoxelSize=mVoxelSize;
	addSimulationObject(static_cast<SimulationObject*>(obj));
}
void FluidSimulation::placeObjects() {
	placeWalls();
	addFluid();
}
void FluidSimulation::addSimulationObject(SimulationObject* obj){
		switch(obj->mType){
			case ObjectType::AIR:
				mAirObjects.push_back(std::shared_ptr<SimulationObject>(obj));break;
			case ObjectType::FLUID:
				mFluidObjects.push_back(std::shared_ptr<SimulationObject>(obj));break;
			case ObjectType::WALL:
				mWallObjects.push_back(std::shared_ptr<SimulationObject>(obj));break;

		}
}
void FluidSimulation::operator()(Springl& springl,double time,double dt){
	Transform::Ptr trans=mSource.transformPtr();
	Vec3d v = Vec3d(springl.particle());
	Vec3d pt = trans->indexToWorld(v);
	Vec3s vel=dt*springl.velocity();
	Vec3d npt=pt+vel;
	double re = 1.5 * mFluidParticleDiameter * mVoxelSize;
	double r = mWallThickness;
	float scale=1.0f/mVoxelSize;
	npt = clamp(npt, r, 1.0 - r);
	int i = clamp((int) (npt[0] * scale), 0,mGridSize[0] - 1);
	int j = clamp((int) (npt[1] * scale), 0,mGridSize[1] - 1);
	int k = clamp((int) (npt[2] * scale), 0,mGridSize[2] - 1);

	vector<FluidParticle*> neighbors =mParticleLocator->getNeigboringCellParticles(i, j, k, 1, 1,1);
	for (int n = 0; n < neighbors.size(); n++) {
		FluidParticle *np = neighbors[n];
		if (np->mObjectType == ObjectType::WALL) {
			float dist = distance(npt, np->mLocation);
			if (dist < re) {
				Vec3f normal =  np->mNormal;
				if (normal[0] == 0.0 && normal[1] == 0.0&& normal[2] == 0.0 && dist>1E-6f) {
					normal = (npt- np->mLocation)/ dist;
				}
				npt += (re - dist) * normal;
				float dot = springl.velocity().dot(normal);
				springl.velocity()-= dot * normal;
			}
		}
	}
	springl.particle() = trans->worldToIndex(npt);	//Apply integration scheme here, need buffer for previous time points?
	int K=springl.size();
	npt=npt-pt;
	for (int k = 0; k < K; k++) {
		pt = trans->indexToWorld(springl[k]);
		springl[k] = trans->worldToIndex(pt+npt);
	}
}
void FluidSimulation::repositionParticles(vector<int>& indices) {
	if (indices.empty())
		return;
	// First Search for Deep Water
	vector<Coord> waters;
	while (waters.size() < indices.size()) {
		FOR_EVERY_GRID_CELL(mLabel)
			{
				if (i > 0 && mLabel(i - 1, j, k) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (i < mGridSize[0] - 1 && mLabel(i + 1, j, k) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (j > 0 && mLabel(i, j - 1, k) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (j < mGridSize[1] - 1 && mLabel(i, j + 1, k) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (k > 0 && mLabel(i, j, k - 1) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (k < mGridSize[2] - 1 && mLabel(i, j, k + 1) != static_cast<char>(ObjectType::FLUID))
					continue;
				if (mLabel(i, j, k) != static_cast<char>(ObjectType::FLUID))
					continue;

				Coord aPos(i, j, k);
				waters.push_back(aPos);
				if (waters.size() >= indices.size()) {
					//Water is larger than particles! Set to max to break out of triple loop.
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
		p->mLocation[0] = mVoxelSize* (waters[n][0] + 0.25 + 0.5 * (rand() % 101) / 100);
		p->mLocation[1] = mVoxelSize* (waters[n][1] + 0.25 + 0.5 * (rand() % 101) / 100);
		p->mLocation[2] = mVoxelSize* (waters[n][2] + 0.25 + 0.5 * (rand() % 101) / 100);
	}
	mParticleLocator->update(mParticles);
	for (int n = 0; n < indices.size(); n++) {
		ParticlePtr &p = mParticles[indices[n]];
		Vec3f u(0.0);
		resampleParticles(p->mLocation, u, mVoxelSize);
		p->mVelocity = u;
	}
}
void FluidSimulation::addParticle(openvdb::Vec3s pt, openvdb::Vec3s center,ObjectType type) {
	SimulationObject *inside_obj = nullptr;
	const int MAX_INT = std::numeric_limits<int>::max();
	const float MAX_ANGLE = 30.0f * M_PI / 180.0f;
	bool found = false;
	if(type==ObjectType::FLUID){
		for (std::shared_ptr<SimulationObject>& obj : mFluidObjects) {
			found=obj->inside(pt);
			if(found){
				inside_obj = obj.get(); // Found
				break;
			}
		}
	} else if(type==ObjectType::WALL){
		for (std::shared_ptr<SimulationObject>& obj : mWallObjects) {
			found=obj->insideShell(pt);
			if(found){
				inside_obj = obj.get(); // Found
				break;
			}
		}
	}
	if (inside_obj) {
		FluidParticle *p = new FluidParticle;

		Vec3s axis(((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f,
				((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f,
				((rand() % MAX_INT) / (MAX_INT - 1.0)) * 2.0f - 1.0f);
		axis.normalize(1E-6f);
		Mat3s R = rotation<Mat3s>(axis,MAX_ANGLE * (rand() % MAX_INT) / (MAX_INT - 1.0));
		if (inside_obj->mType == ObjectType::FLUID) {
			p->mLocation = center + R * (pt - center);
		} else {
			p->mLocation = pt;
		}
		p->mVelocity = Vec3f(0.0);
		p->mNormal = Vec3f(0.0);
		p->mDensity = 10.0;
		p->mObjectType = inside_obj->mType;
		p->mMass = 1.0;
		mParticles.push_back(ParticlePtr(p));
	}
}
bool FluidSimulation::init() {
	mSimulationTime = 0;
	mSimulationIteration = 0;
	mParticleLocator = std::unique_ptr<ParticleLocator>(
			new ParticleLocator(mGridSize, mVoxelSize));
	placeObjects();
	// This Is A Test Part. We Generate Pseudo Particles To Measure Maximum Particle Density
	float h = mFluidParticleDiameter * mVoxelSize;
	FOR_EVERY_CELL(10,10,10)
		{
			FluidParticle *p = new FluidParticle;
			p->mLocation = Vec3f((i + 0.5) * h, (j + 0.5) * h, (k + 0.5) * h);
			p->mObjectType = ObjectType::FLUID;
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
	Vec3s center;
	Vec3s pt;
	// Place Fluid Particles And Walls
	Coord dims=mLabel.dimensions();
	float mx=mVoxelSize*dims[0];
	float my=mVoxelSize*dims[1];
	float mz=mVoxelSize*dims[2];

	double wx = mFluidParticleDiameter * mVoxelSize;
	double wy = mFluidParticleDiameter * mVoxelSize;
	double wz = mFluidParticleDiameter * mVoxelSize;

	FOR_EVERY_GRID_CELL(mLabel)
		{
			for (int ii = 0; ii < 2; ii++) {
				for (int jj = 0; jj < 2; jj++) {
					for (int kk = 0; kk < 2; kk++) {
						double x = wx * (2 * i + ii + 0.5);
						double y = wy * (2 * j + jj + 0.5);
						double z = wz * (2 * k + kk + 0.5);
						if (	   x > mWallThickness
								&& x < mx - mWallThickness
								&& y > mWallThickness
								&& y < my - mWallThickness
								&& z > mWallThickness
								&& z < mz - mWallThickness) {
							center = Vec3s(
									wx * (2 * i + 1),
									wy * (2 * j + 1),
									wz * (2 * k + 1));
							addParticle(Vec3s(x, y, z), center, ObjectType::FLUID);
						}
					}
				}
			}
		}END_FOR;
	FOR_EVERY_GRID_CELL(mLabel)
		{
			double x = wx * (2 * i + 1);
			double y = wy * (2 * j + 1);
			double z = wz * (2 * k + 1);
			addParticle(Vec3s(x, y, z), Vec3s(x, y, z), ObjectType::WALL);
		}END_FOR;
	mParticleLocator->update(mParticles);
	mParticleLocator->markAsWater(mLabel, mWallWeight, mFluidParticleDiameter);
// Remove Particles That Stuck On Wal Cells
	float scale=1.0f/mVoxelSize;
	for (std::vector<ParticlePtr>::iterator iter = mParticles.begin();
			iter != mParticles.end();) {
		ParticlePtr& p = *iter;
		if (p->mObjectType == ObjectType::WALL) {
			iter++;
			continue;
		}
		int i = clamp((int) (scale * p->mLocation[0]), 0,mGridSize[0] - 1);
		int j = clamp((int) (scale * p->mLocation[1]), 0,mGridSize[1] - 1);
		int k = clamp((int) (scale * p->mLocation[2]), 0,mGridSize[2] - 1);
		if (mLabel(i, j, k) == static_cast<char>(ObjectType::WALL)) {
			iter = mParticles.erase(iter);
		} else {
			iter++;
		}
	}
// Comput Normal for Walls
	computeWallNormals();
	updateParticleVolume();
	initLevelSet();
	//WriteToRawFile(mParticleLevelSet,MakeString()<<"/home/blake/tmp/init"<<mSimulationIteration<<".xml");
	mSource.create(mParticleLevelSet);
	//mSource.mIsoSurface.save(MakeString()<<"/home/blake/tmp/init"<<mSimulationIteration<<".ply");
	if(!mSpringlTracking){
		mSource.mConstellation.reset();
	} else {
		mAdvect=std::unique_ptr<SpringLevelSetParticleDeformation<FluidSimulation,openvdb::util::NullInterrupter> >(new SpringLevelSetParticleDeformation<FluidSimulation,openvdb::util::NullInterrupter>(mSource,*this,mMotionScheme));
		mAdvect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK1);
		mAdvect->setResampleEnabled(true);
		mAdvect->setTrackingIterations(16);
		mAdvect->setConvergenceThreshold(1E-6f);
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
	#pragma omp for
		for(int n=0;n<velocities.size();n++){
			velocities[n]=Vec3s(0.0);
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
				p->mObjectType = ObjectType::FLUID;
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
	//Add gravity acceleration to all particles
	int count=0;
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		if (mParticles[n]->mObjectType == ObjectType::FLUID){
			mParticles[n]->mVelocity[1] += velocity;
			count++;
		}
	}
	if(mSpringlTracking){
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
		if (p->mObjectType == ObjectType::FLUID) {
			p->mLocation += mTimeStep * mVelocity.interpolate(p->mLocation);
		}
	}
	//Update localization
	mParticleLocator->update(mParticles);
	double re = 1.5 * mFluidParticleDiameter * mVoxelSize;
	float r = mWallThickness;

	float scale=1.0f/mVoxelSize;
	float mx=mVoxelSize*mGridSize[0];
	float my=mVoxelSize*mGridSize[1];
	float mz=mVoxelSize*mGridSize[2];
	//Correct particle locations
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		if (p->mObjectType == ObjectType::FLUID) {
			p->mLocation[0] = clamp(p->mLocation[0], r, mx - r);
			p->mLocation[1] = clamp(p->mLocation[1], r, my - r);
			p->mLocation[2] = clamp(p->mLocation[2], r, mz - r);
			int i = clamp((int) (p->mLocation[0] * scale), 0,mGridSize[0] - 1);
			int j = clamp((int) (p->mLocation[1] * scale), 0,mGridSize[1] - 1);
			int k = clamp((int) (p->mLocation[2] * scale), 0,mGridSize[2] - 1);
			vector<FluidParticle*> neighbors =mParticleLocator->getNeigboringCellParticles(i, j, k, 1, 1,1);
			for (int n = 0; n < neighbors.size(); n++) {
				FluidParticle *np = neighbors[n];
				if (np->mObjectType == ObjectType::WALL) {
					float dist = distance(p->mLocation, np->mLocation);
					if (dist < re) {
						Vec3f normal =  np->mNormal;
						if (normal[0] == 0.0 && normal[1] == 0.0&& normal[2] == 0.0 && dist) {
							normal = (p->mLocation - np->mLocation)/ dist;
						}
						p->mLocation += (re - dist) * normal;
						float dot = p->mVelocity.dot(normal);
						p->mVelocity -= dot * normal;
					}
				}
			}
		}
	}
	/*
	if(mSpringlTracking){
		mAdvect->advect(mSimulationTime,mSimulationTime+mTimeStep);
	}
	*/
	if(mSpringlTracking){
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
		std::vector<Vec3s>& positions=mSource.mConstellation.mParticles;
		std::vector<Vec3s>& springls=mSource.mConstellation.mVertexes;
		float invScale=mTimeStep/(0.5f*mVoxelSize);
#pragma omp for
		for(int n=0;n<velocities.size();n++){
			Vec3f v=invScale*velocities[n];
			positions[n]+=v;
			size_t offset=n*4;
			for(int i=0;i<4;i++){
				springls[offset++]+=v;
			}
		}
	}

// Remove Particles That Stuck On The Up-Down Wall Cells...
	OPENMP_FOR FOR_EVERY_PARTICLE(mParticles)
	{
		ParticlePtr& p = mParticles[n];
		p->mRemoveIndicator = false;
		// Focus on Only Fluid Particle
		if (p->mObjectType == ObjectType::FLUID) {
			int i = clamp((int) (p->mLocation[0] * scale), 0, mGridSize[0] - 1);
			int j = clamp((int) (p->mLocation[1] * scale), 0, mGridSize[1] - 1);
			int k = clamp((int) (p->mLocation[2] * scale), 0, mGridSize[2] - 1);
			// If Stuck On Wall Cells Just Reposition
			if (mLabel(i, j, k) == static_cast<char>(ObjectType::WALL)) {
				p->mRemoveIndicator = true;
			}
			i = clamp((int) (p->mLocation[0] * scale), 2, mGridSize[0] - 3);
			j = clamp((int) (p->mLocation[1] * scale), 2, mGridSize[1] - 3);
			k = clamp((int) (p->mLocation[2] * scale), 2, mGridSize[2] - 3);
			if (p->mDensity < 0.04
					&& (mLabel(i, max(0, j - 1), k) == static_cast<char>(ObjectType::WALL)
							|| mLabel(i, min(mGridSize[1] - 1, j + 1), k) == static_cast<char>(ObjectType::WALL))) {
				// Put Into Reposition List
				p->mRemoveIndicator = true;
			}
		}

	}
// Reposition If Necessary
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
	if(!mSpringlTracking){
		advectParticles();
		correctParticles( mParticles, mTimeStep,mFluidParticleDiameter * mVoxelSize);
		createLevelSet();
		mSource.updateIsoSurface();
	} else {
		/*
		createLevelSet();
		mSource.updateUnSignedLevelSet(2.5f*LEVEL_SET_HALF_WIDTH);
		mSource.updateGradient();
		mAdvect->evolve();
		mSource.updateIsoSurface();
		*/
		mSource.clean();
		mSource.updateUnSignedLevelSet();
		int count=mSource.fill();
		mSource.fillWithVelocityField(mVelocity,0.5f*mVoxelSize);
		mSource.updateUnSignedLevelSet(2.5f*LEVEL_SET_HALF_WIDTH);
		advectParticles();
		correctParticles( mParticles, mTimeStep,mFluidParticleDiameter * mVoxelSize);
		createLevelSet();
		mSource.updateUnSignedLevelSet(2.5f*LEVEL_SET_HALF_WIDTH);
		mSource.updateGradient();
		mAdvect->evolve();
		mSource.updateIsoSurface();

		//mSource.mIsoSurface.save(MakeString()<<"/home/blake/tmp/iso_after"<<(frameCounter-1)<<".ply");
		//WriteToRawFile(mSource.mSignedLevelSet,MakeString()<<"/home/blake/tmp/signed_sparse"<<mSimulationIteration<<".xml");
		//mSource.mIsoSurface.save(MakeString()<<"/home/blake/tmp/iso"<<mSimulationIteration<<".ply");
		//std::exit(0);
	}
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
			if (mLabel(i, j, k) == static_cast<char>(ObjectType::FLUID)) {
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
			mark[0](i, j, k) = (i > 0 && mLabel(i - 1, j, k) == static_cast<char>(ObjectType::FLUID))|| (i < mGridSize[0] && mLabel(i, j, k) == static_cast<char>(ObjectType::FLUID));
			wall_mark[0](i, j, k) = (i <= 0 || mLabel(i - 1, j, k) == static_cast<char>(ObjectType::WALL))&& (i >= mGridSize[0] || mLabel(i, j, k) == static_cast<char>(ObjectType::WALL));
		}END_FOR;

	OPENMP_FOR FOR_EVERY_GRID_CELL(mark[1])
		{
			mark[1](i, j, k) = (j > 0 && mLabel(i, j - 1, k) == static_cast<char>(ObjectType::FLUID))|| (j < mGridSize[1] && mLabel(i, j, k) == static_cast<char>(ObjectType::FLUID));
			wall_mark[1](i, j, k) = (j <= 0 || mLabel(i, j - 1, k) == static_cast<char>(ObjectType::WALL))&& (j >= mGridSize[1] || mLabel(i, j, k) == static_cast<char>(ObjectType::WALL));
		}END_FOR;

	OPENMP_FOR FOR_EVERY_GRID_CELL(mark[2])
		{
			mark[2](i, j, k) = (k > 0 && mLabel(i, j, k - 1) == static_cast<char>(ObjectType::FLUID))|| (k < mGridSize[2] && mLabel(i, j, k) == static_cast<char>(ObjectType::FLUID));
			wall_mark[2](i, j, k) = (k <= 0 || mLabel(i, j, k - 1) == static_cast<char>(ObjectType::WALL))&& (k >= mGridSize[2] || mLabel(i, j, k) == static_cast<char>(ObjectType::WALL));
		}END_FOR;
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
	if(mSpringlTracking){
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
		std::vector<Vec3s>& positions=mSource.mConstellation.mParticles;
#pragma omp for
		for(int n=0;n<velocities.size();n++){
			Vec3s pt=0.5f*mVoxelSize*positions[n];
			openvdb::Vec3s currentVelocity=mVelocity.maxInterpolate(pt,0.5f*mVoxelSize);
			openvdb::Vec3s velocity=velocities[n]+currentVelocity-mVelocityLast.maxInterpolate(pt,0.5f*mVoxelSize);
			velocities[n] = (1.0 - mPicFlipBlendWeight) *currentVelocity  + mPicFlipBlendWeight * velocity;
		}
	}
}

void FluidSimulation::updateParticleVolume(){
	//
	float voxelSize = mParticleLevelSet.voxelSize();
		mSource.mParticleVolume.mParticles.clear();
		mSource.mParticleVolume.mVelocities.clear();
		float scale = mVoxelSize / voxelSize;
		FOR_EVERY_PARTICLE(mParticles)
		{
			FluidParticle* p = mParticles[n].get();
			if (p->mObjectType == ObjectType::FLUID) {
				Vec3s l = p->mLocation / voxelSize;
				Vec4s v(l[0], l[1], l[2], scale * 0.5f * mFluidParticleDiameter);
				mSource.mParticleVolume.mParticles.push_back(v);
				mSource.mParticleVolume.mVelocities.push_back(p->mVelocity);
			}
		}
		Coord dims(mParticleLevelSet.rows(), mParticleLevelSet.cols(), mParticleLevelSet.slices());
		mSource.mParticleVolume.setBoundingBox(BBoxd(Vec3d(0, 0, 0), Vec3d(dims[0], dims[1], dims[2])));
}
void FluidSimulation::createLevelSet() {
// Create Density Field

	updateParticleVolume();
	Coord dims(mParticleLevelSet.rows(), mParticleLevelSet.cols(), mParticleLevelSet.slices());
	float voxelSize = mParticleLevelSet.voxelSize();
	OPENMP_FOR FOR_EVERY_GRID_CELL_Y(mParticleLevelSet)
		{
			double x = i * voxelSize;
			double y = j * voxelSize;
			double z = k * voxelSize;
			Vec3f p(x, y, z);

			if (i == 0 || i == dims[0] - 1 || j == 0 || j == dims[1] - 1|| k == 0 || k == dims[2] - 1) {
				//value = max(value,0.01);
				mParticleLevelSet(i, j, k) = openvdb::LEVEL_SET_HALF_WIDTH;
			} else {
				double value = implicit_func(p,mFluidParticleDiameter);
				mParticleLevelSet(i, j, k) = clamp(2.0f*value,-openvdb::LEVEL_SET_HALF_WIDTH,openvdb::LEVEL_SET_HALF_WIDTH);
			}
		}END_FOR;


	const float R=openvdb::LEVEL_SET_HALF_WIDTH;
	mDistanceField.solve(mParticleLevelSet,mSignedLevelSet,openvdb::LEVEL_SET_HALF_WIDTH);

	mSource.mSignedLevelSet=std::unique_ptr<FloatGrid>(new FloatGrid());
	mSource.mSignedLevelSet->setBackground(openvdb::LEVEL_SET_HALF_WIDTH);
	mSource.mSignedLevelSet->setTransform(Transform::createLinearTransform(1.0));
	mSource.mSignedLevelSet->setGridClass(GRID_LEVEL_SET);
	openvdb::tools::copyFromDense(mSignedLevelSet,*mSource.mSignedLevelSet,1E-3f);
	frameCounter++;
}
void FluidSimulation::initLevelSet() {
// Create Density Field
	Coord dims(mParticleLevelSet.rows(), mParticleLevelSet.cols(), mParticleLevelSet.slices());
	float voxelSize = mParticleLevelSet.voxelSize();
	OPENMP_FOR FOR_EVERY_GRID_CELL_Y(mParticleLevelSet)
		{
			double x = i * voxelSize;
			double y = j * voxelSize;
			double z = k * voxelSize;
			Vec3f p(x, y, z);
			if (i == 0 || i == dims[0] - 1 || j == 0 || j == dims[1] - 1|| k == 0 || k == dims[2] - 1) {
				mParticleLevelSet(i, j, k) = openvdb::LEVEL_SET_HALF_WIDTH;
			} else {
				float value = 2*openvdb::LEVEL_SET_HALF_WIDTH;
				for(shared_ptr<SimulationObject>& obj:mFluidObjects){
					value=std::min(obj->signedDistance(p),value);
				}
				mParticleLevelSet(i, j, k) = clamp(value / mVoxelSize,-(float)openvdb::LEVEL_SET_HALF_WIDTH,(float)openvdb::LEVEL_SET_HALF_WIDTH);
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
	float scale=1.0f/mVoxelSize;
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
				if( p->mObjectType == ObjectType::FLUID ) {
					float x = clamp(scale*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(scale*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(scale*p->mLocation[2],0.0f,(float)dims[2]);
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
				if( p->mObjectType == ObjectType::FLUID ) {
					float x = clamp(scale*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(scale*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(scale*p->mLocation[2],0.0f,(float)dims[2]);
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
				if( p->mObjectType == ObjectType::FLUID ) {
					float x = clamp(scale*p->mLocation[0],0.0f,(float)dims[0]);
					float y = clamp(scale*p->mLocation[1],0.0f,(float)dims[1]);
					float z = clamp(scale*p->mLocation[2],0.0f,(float)dims[2]);
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

void FluidSimulation::resampleParticles( openvdb::Vec3f& p,openvdb::Vec3f& u, float re ) {
	// Variables for Neighboring Particles
	std::vector<FluidParticle*> neighbors;
	openvdb::Coord cell_size = mParticleLocator->getGridSize();
	float wsum = 0.0;
	openvdb::Vec3f save(u);
	u[0] = u[1] = u[2] = 0.0;
	float scale=mParticleLocator->getVoxelSize();
	int i = clamp((int)(p[0]*scale),0,cell_size[0]-1);
	int j = clamp((int)(p[1]*scale),0,cell_size[1]-1);
	int k = clamp((int)(p[2]*scale),0,cell_size[2]-1);
	// Gather Neighboring Particles
	neighbors = mParticleLocator->getNeigboringCellParticles(i,j,k,1,1,1);
	for(FluidParticle *np:neighbors) {
		if( np->mObjectType == ObjectType::FLUID ) {
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

void FluidSimulation::correctParticles(std::vector<ParticlePtr>& particles, float dt, float re ) {
	// Variables for Neighboring Particles
	openvdb::Coord cell_size = mParticleLocator->getGridSize();
	mParticleLocator->update(particles);
	float scale=1.0f/mParticleLocator->getVoxelSize();
	// Compute Pseudo Moved Point
	OPENMP_FOR FOR_EVERY_PARTICLE(particles) {
		if( particles[n]->mObjectType == ObjectType::FLUID ) {
			FluidParticle *p = particles[n].get();
			openvdb::Vec3f spring(0.0);
			int i = clamp((int)(p->mLocation[0]*scale),0,cell_size[0]-1);
			int j = clamp((int)(p->mLocation[1]*scale),0,cell_size[1]-1);
			int k = clamp((int)(p->mLocation[2]*scale),0,cell_size[2]-1);
			std::vector<FluidParticle*> neighbors = mParticleLocator->getNeigboringCellParticles(i,j,k,1,1,1);
			for( int n=0; n<neighbors.size(); n++ ) {
				FluidParticle *np = neighbors[n];
				if( p != np ) {
					float dist = distance(p->mLocation,np->mLocation);
					float w = SPRING_STIFFNESS * np->mMass * smoothKernel(dist*dist,re);
					if( dist > 0.1*re ) {
						spring += w * (p->mLocation-np->mLocation) / dist * re;
					} else {
						if( np->mObjectType == ObjectType::FLUID ) {
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
		if( particles[n]->mObjectType == ObjectType::FLUID ) {
			FluidParticle *p = particles[n].get();
			p->mTmp[1] = p->mVelocity;
			resampleParticles( p->mTmp[0], p->mTmp[1], re );
		}
	}

	if(mSpringlTracking){
		std::vector<Vec3s>& positions=mSource.mConstellation.mParticles;
		std::vector<Vec3s>& velocities=mSource.mConstellation.mParticleVelocity;
		#pragma omp for
		for(int n=0;n<positions.size();n++){
			//velocities[n]=mVelocity.interpolate(0.5f*mVoxelSize*positions[n]);
			// Variables for Neighboring Particles
			float wsum = 0.0;
			Vec3s u(0.0f);
			Vec3s pt=positions[n];
			std::vector<FluidParticle*> neighbors = mParticleLocator->getNeigboringCellParticles((int)(0.5f*pt[0]),(int)(0.5f*pt[1]),(int)(0.5f*pt[2]),1,1,1);
			pt*=0.5f*mVoxelSize;
			for(FluidParticle *np:neighbors) {
				if( np->mObjectType == ObjectType::FLUID ) {
					float w = np->mMass * sharpKernel(distanceSquared(pt,np->mLocation),mFluidParticleDiameter * mVoxelSize);
					u += w * np->mVelocity;
					wsum += w;
				}
			}
			if(wsum>0.0f) {
				u /= wsum;
				velocities[n]=u;
			}
		}
	}

	// Update
	OPENMP_FOR FOR_EVERY_PARTICLE(particles) {
		if( particles[n]->mObjectType == ObjectType::FLUID ) {
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
double FluidSimulation::implicit_func( vector<FluidParticle*> &neighbors,openvdb::Vec3f& p, float radius,float voxelSize) {
	double phi = 8.0f*radius;
	float scale=1.0f/mParticleLocator->getVoxelSize();
	for( int m=0; m<neighbors.size(); m++ ) {
		FluidParticle &np = *neighbors[m];
		double d= distance(np.mLocation,p)*scale;
		if( np.mObjectType == ObjectType::WALL ) {
			if(d < radius) return 4.5*radius;
			continue;
		}
		if( d < phi ) {
			phi = d;
		}
	}
	//std::cout<<"IMPLICIT "<<phi<<" "<<density<<" "<<voxelSize<<std::endl;
	return phi - radius;
}
double FluidSimulation::implicit_func( openvdb::Vec3f& p, float radius ) {
	openvdb::Coord cell_size = mParticleLocator->getGridSize();
	float scale=1.0f/mParticleLocator->getVoxelSize();
	vector<FluidParticle *> neighbors = mParticleLocator->getNeigboringCellParticles(
			clamp((int)(p[0]*scale),0,cell_size[0]-1),
			clamp((int)(p[1]*scale),0,cell_size[1]-1),
			clamp((int)(p[2]*scale),0,cell_size[2]-1),2,2,2
			);
	return implicit_func( neighbors, p, radius,mParticleLocator->getVoxelSize());
}
void FluidSimulation::computeWallNormals() {
// mParticleLocator Particles
	mParticleLocator->update(mParticles);
// Compute Wall Normal
	float scale=1.0f/mVoxelSize;

	float mx=mVoxelSize*mGridSize[0];
	float my=mVoxelSize*mGridSize[1];
	float mz=mVoxelSize*mGridSize[2];

	for (ParticlePtr& p : mParticles) {
		int i = clamp((int) (p->mLocation[0] * scale), 0,mGridSize[0] - 1);
		int j = clamp((int) (p->mLocation[1] * scale), 0,mGridSize[1] - 1);
		int k = clamp((int) (p->mLocation[2] * scale), 0,mGridSize[2] - 1);
		mWallNormal(i, j, k) = Vec3f(0.0f);
		p->mNormal = Vec3f(0.0);

		if (p->mObjectType == ObjectType::WALL) {
			if (p->mLocation[0] <= (mx+0.1) * mWallThickness) {
				p->mNormal[0] = 1.0;
			}
			if (p->mLocation[0] >= mx - (mx-0.1) * mWallThickness) {
				p->mNormal[0] = -1.0;
			}
			if (p->mLocation[1] <= (my+0.1) * mWallThickness) {
				p->mNormal[1] = 1.0;
			}
			if (p->mLocation[1] >= my - (my-0.1) * mWallThickness) {
				p->mNormal[1] = -1.0;
			}
			if (p->mLocation[2] <= (mz+0.1) * mWallThickness) {
				p->mNormal[2] = 1.0;
			}
			if (p->mLocation[2] >= mz - (mz-0.1) * mWallThickness) {
				p->mNormal[2] = -1.0;
			}

			if (p->mNormal[0] == 0.0 && p->mNormal[1] == 0.0
					&& p->mNormal[2] == 0.0) {
				vector<FluidParticle*> neighbors =
						mParticleLocator->getNeigboringCellParticles(i, j, k, 3,
								3, 3);
				for (int n = 0; n < neighbors.size(); n++) {
					FluidParticle *np = neighbors[n];
					if (p.get() != np && np->mObjectType == ObjectType::WALL) {
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
			if (mLabel(i, j, k) != static_cast<char>(ObjectType::WALL)) {
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
					if (si < 0 || si > mGridSize[0] - 1 || sj < 0|| sj > mGridSize[1] - 1 || sk < 0|| sk > mGridSize[2] - 1)
						continue;
					if (mLabel(si, sj, sk) == static_cast<char>(ObjectType::WALL)) {
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
