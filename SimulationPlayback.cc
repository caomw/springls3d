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

#include "SimulationPlayback.h"
#include "ImageSciUtil.h"
namespace imagesci {

SimulationPlayback::SimulationPlayback(const std::string& directory):Simulation("Recording",MotionScheme::UNDEFINED),mDirectory(directory) {
	// TODO Auto-generated constructor stub

}
bool SimulationPlayback::init(){
	if(mIsInitialized)return true;
	mIsoSurfaceFiles.clear();
	mConstellationFiles.clear();
	mSignedDistanceFiles.clear();
	mParticleVolumeFiles.clear();
	mTimeSteps.clear();
	std::vector<std::string> jsonFiles;
	int n=GetDirectoryListing(mDirectory,jsonFiles,"",".sim");

	Json::Reader reader;
	SpringLevelSetDescription springlDesc;
	SimulationTimeStepDescription simDesc;
	std::ifstream ifs;
	if(n==0){
		std::cout<<"Could not find any *.sim files"<<std::endl;
		return false;
	}
	for(std::string& file:jsonFiles){
		Json::Value deserializeRoot;
		ifs.open(file, std::ifstream::in);
		if (!ifs.is_open()) {
			std::cout<<"Could not open "<<file<<std::endl;
			return false;
		}
		std::string input((std::istreambuf_iterator<char>(ifs)),
		std::istreambuf_iterator<char>());
		ifs.close();
		if ( !reader.parse(input, deserializeRoot) ){
			std::cout<<"Could not parse "<<file<<std::endl;
			continue;
		}
		springlDesc.deserialize(deserializeRoot["Simulation Record"]);
		simDesc.deserialize(deserializeRoot["Simulation Record"]);
		mIsoSurfaceFiles.push_back(springlDesc.mIsoSurfaceFile);
		mConstellationFiles.push_back(springlDesc.mConstellationFile);
		mSignedDistanceFiles.push_back(springlDesc.mSignedLevelSetFile);
		mParticleVolumeFiles.push_back(springlDesc.mParticleVolumeFile);
		mSimulationDuration=simDesc.mSimulationDuration;
		mName=simDesc.mSimulationName;
		mTimeStep=simDesc.mTimeStep;
		mMotionScheme=simDesc.mMotionScheme;
		mTimeSteps.push_back(simDesc);
	}

	mSimulationIteration=0;
	if(mSimulationIteration<mConstellationFiles.size()&&mConstellationFiles[mSimulationIteration].length()>0&&mTemporaryMesh.openMesh(mDirectory+GetFileName(mConstellationFiles[mSimulationIteration]))){
		mSource.mConstellation.create(&mTemporaryMesh);
		mSource.mConstellation.updateVertexNormals();
		mSource.mConstellation.updateBoundingBox();
	}
	if(mSimulationIteration<mIsoSurfaceFiles.size()&&mIsoSurfaceFiles[mSimulationIteration].length()>0&&mSource.mIsoSurface.openMesh(mDirectory+GetFileName(mIsoSurfaceFiles[mSimulationIteration]))){
		mSource.mIsoSurface.updateVertexNormals(4);
	}

	if(mSimulationIteration<mParticleVolumeFiles.size()&&mParticleVolumeFiles[mSimulationIteration].length()>0&&mSource.mParticleVolume.open(mDirectory+GetFileName(mParticleVolumeFiles[mSimulationIteration]))){
	}

	if(mSimulationIteration<mSignedDistanceFiles.size()&&mSignedDistanceFiles[mSimulationIteration].length()>0){
		openvdb::io::File file(mDirectory+GetFileName(mSignedDistanceFiles[mSimulationIteration]));
		file.open();
		if(file.isOpen()){
			openvdb::GridPtrVecPtr grids =file.getGrids();
			openvdb::GridPtrVec allGrids;
			allGrids.insert(allGrids.end(), grids->begin(), grids->end());
			GridBase::Ptr ptr = allGrids[0];
			FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
			mSource.transform()=mSignedLevelSet->transform();
			mSignedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
			mSource.mSignedLevelSet=mSignedLevelSet;
		}
	}

	simDesc=mTimeSteps[mSimulationIteration];
	mSimulationTime=simDesc.mSimulationTime;
	mTimeStep=simDesc.mTimeStep;
	mSimulationDuration=simDesc.mSimulationDuration;
	mName=simDesc.mSimulationName;
	mComputeTimeSeconds=simDesc.mComputeTimeSeconds;
	mIsMeshDirty=true;
	return true;
}
bool SimulationPlayback::step(){
	if(mSimulationIteration<mConstellationFiles.size()&&mConstellationFiles[mSimulationIteration].length()>0&&mTemporaryMesh.openMesh(mDirectory+GetFileName(mConstellationFiles[mSimulationIteration]))){
		mSource.mConstellation.create(&mTemporaryMesh);
		mSource.mConstellation.updateVertexNormals();
		mSource.mConstellation.updateBoundingBox();
	}
	if(mSimulationIteration<mIsoSurfaceFiles.size()&&mIsoSurfaceFiles[mSimulationIteration].length()>0&&mSource.mIsoSurface.openMesh(mDirectory+GetFileName(mIsoSurfaceFiles[mSimulationIteration]))){
		mSource.mIsoSurface.updateVertexNormals(4);
	}
	if(mSimulationIteration<mParticleVolumeFiles.size()&&mParticleVolumeFiles[mSimulationIteration].length()>0&&mSource.mParticleVolume.open(mDirectory+GetFileName(mParticleVolumeFiles[mSimulationIteration]))){
	}
	/*
	if(mSimulationIteration<mSignedDistanceFiles.size()&&mSignedDistanceFiles[mSimulationIteration].length()>0){
		openvdb::io::File file(mDirectory+GetFileName(mSignedDistanceFiles[mSimulationIteration]));
		file.open();
		if(file.isOpen()){
			openvdb::GridPtrVecPtr grids =file.getGrids();
			openvdb::GridPtrVec allGrids;
			allGrids.insert(allGrids.end(), grids->begin(), grids->end());
			GridBase::Ptr ptr = allGrids[0];
			FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
			mSource.transform()=mSignedLevelSet->transform();
			mSignedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
			mSource.mSignedLevelSet=mSignedLevelSet;
		}
	}
	*/
	SimulationTimeStepDescription& simDesc=mTimeSteps[mSimulationIteration];

	mSimulationTime=simDesc.mSimulationTime;
	mTimeStep=simDesc.mTimeStep;
	mSimulationDuration=simDesc.mSimulationDuration;
	mName=simDesc.mSimulationName;
	mComputeTimeSeconds=simDesc.mComputeTimeSeconds;
	mSimulationIteration++;
	mIsMeshDirty=true;

	if(mSimulationTime<=mSimulationDuration&&mSimulationIteration<mTimeSteps.size()&&mRunning){
		return true;
	} else {
		return false;
	}
}
void SimulationPlayback::cleanup(){
	mIsoSurfaceFiles.clear();
	mConstellationFiles.clear();
	mSignedDistanceFiles.clear();
	mParticleVolumeFiles.clear();
	mTimeSteps.clear();
	mSource.mIsoSurface.reset();
	mSource.mConstellation.reset();
	mSource.mParticleVolume.reset();
	mIsInitialized=false;
}
SimulationPlayback::~SimulationPlayback() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
