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

#include "SplashSimulation.h"
#include <openvdb/tools/DenseSparseTools.h>
#include <openvdb/tools/MeshToVolume.h>
#include <memory>
namespace imagesci {
using namespace openvdb::tools;
using namespace imagesci::fluid;
SplashSimulation::SplashSimulation(const std::string& fileName,int gridSize,MotionScheme scheme):FluidSimulation(Coord(
		gridSize,2*gridSize,gridSize),1.0f/gridSize,scheme),mSourceFileName(fileName),mGridSize(gridSize) {
}

bool SplashSimulation::init(){
	bool ret=FluidSimulation::init();
	mIsMeshDirty=true;
	mSimulationDuration=2.0;
	return ret;
}
void SplashSimulation::addFluid(){
	//replace with level set for falling object

	Coord dims=mLabel.dimensions();
	if(mSourceFileName.size()==0){
		SphereObject* obj=new SphereObject;
		obj->mType = ObjectType::FLUID;
		obj->mVisible = true;
		obj->mRadius=0.075f;
		obj->mThickness=3.0*mVoxelSize;
		obj->mCenter=Vec3f(mVoxelSize*dims[0]*0.5f,mVoxelSize*dims[1]-0.2-obj->mRadius,mVoxelSize*dims[2]*0.5f);
		mSimulationObjects.push_back(std::shared_ptr<SimulationObject>(static_cast<SimulationObject*>(obj)));
	} else {
		Mesh mesh;
		if(mesh.openMesh(mSourceFileName)){
			MeshObject* obj=new MeshObject;
			obj->mType = ObjectType::FLUID;
			mesh.updateBoundingBox();
			mesh.mapIntoBoundingBox(2.0f*mesh.estimateVoxelSize());
			mesh.updateBoundingBox();
			openvdb::math::Transform::Ptr trans =openvdb::math::Transform::createLinearTransform(1.0);
			openvdb::tools::MeshToVolume<openvdb::FloatGrid> mtol(trans);
			mtol.convertToLevelSet(mesh.mVertexes, mesh.mFaces);
			SLevelSetPtr levelSet= mtol.distGridPtr();
			openvdb::CoordBBox bbox = levelSet->evalActiveVoxelBoundingBox();
			mSourceLevelSet=std::unique_ptr<RegularGrid<float> >(new RegularGrid<float>(bbox));
			copyToDense(*levelSet, *mSourceLevelSet);
			obj->mSignedLevelSet=mSourceLevelSet.get();
			obj->mVisible = true;
			obj->mThickness=3.0*mVoxelSize;
			obj->mVoxelSize=0.5*mVoxelSize;
			std::cout<<"Mesh "<<mVoxelSize<<" "<<mSourceLevelSet->voxelSize()<<std::endl;
			mSimulationObjects.push_back(std::shared_ptr<SimulationObject>(static_cast<SimulationObject*>(obj)));
		}
	}
	BoxObject* box=new BoxObject;
	box->mType = ObjectType::FLUID;
	box->mVisible = true;
	box->mThickness=3.0*mVoxelSize;
	box->mMin = Vec3f(mWallThickness, mWallThickness, mWallThickness);
	box->mMax = Vec3f(mVoxelSize*dims[0] - mWallThickness,0.15,mVoxelSize*dims[2] - mWallThickness);
	mSimulationObjects.push_back(std::shared_ptr<SimulationObject>(static_cast<SimulationObject*>(box)));

}

void SplashSimulation::cleanup(){
	mAdvect.reset();
	FluidSimulation::cleanup();
}
bool SplashSimulation::step(){
	Clock::time_point t0 = Clock::now();
	bool ret=FluidSimulation::step();
	Clock::time_point t1 = Clock::now();
	mComputeTimeSeconds= 1E-6*std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
	mIsMeshDirty=true;
	return ret;
}
} /* namespace imagesci */
