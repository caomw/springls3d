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
	mIsoSurfaceFiles.clear();
	mConstellationFiles.clear();
	mSignedDistanceFiles.clear();
	std::vector<std::string> jsonFiles;
	int n=GetDirectoryListing(mDirectory,jsonFiles,"",".sim");

	Json::Reader reader;
	SpringLevelSetDescription springlDesc;
	SimulationTimeStepDescription simDesc;
	std::ifstream ifs;
	if(n==0)return false;
	for(std::string& file:jsonFiles){
		Json::Value deserializeRoot;
		ifs.open(file, std::ifstream::in);
		if (!ifs.is_open()) return false;
		std::string input((std::istreambuf_iterator<char>(ifs)),
		std::istreambuf_iterator<char>());
		ifs.close();
		if ( !reader.parse(input, deserializeRoot) )return false;
		springlDesc.deserialize(deserializeRoot["Simulation Record"]);
		simDesc.deserialize(deserializeRoot["Simulation Record"]);
		mIsoSurfaceFiles.push_back(springlDesc.mIsoSurfaceFile);
		mConstellationFiles.push_back(springlDesc.mConstellationFile);
		mSignedDistanceFiles.push_back(springlDesc.mSignedLevelSetFile);
		mSimulationDuration=simDesc.mSimulationDuration;
		mTimeStep=simDesc.mTimeStep;
		mTimeSteps.push_back(simDesc);
	}

	mSimulationIteration=0;
	Mesh c;
	c.openMesh(mConstellationFiles[mSimulationIteration]);
	mSource.mConstellation.create(&c);
	mSource.mIsoSurface.openMesh(mIsoSurfaceFiles[mSimulationIteration]);
	mSource.mIsoSurface.updateVertexNormals(16);
	mSource.mConstellation.updateVertexNormals();
	return true;
}
bool SimulationPlayback::step(){
	mTemporaryMesh.openMesh(mConstellationFiles[mSimulationIteration]);
	mSource.mConstellation.create(&mTemporaryMesh);
	mSource.mConstellation.updateVertexNormals();
	mSource.mIsoSurface.openMesh(mIsoSurfaceFiles[mSimulationIteration]);
	mSource.mIsoSurface.updateVertexNormals(16);
	openvdb::io::File file(mSignedDistanceFiles[mSimulationIteration]);
	file.open();
	openvdb::GridPtrVecPtr grids =file.getGrids();
	openvdb::GridPtrVec allGrids;
	allGrids.insert(allGrids.end(), grids->begin(), grids->end());
	GridBase::Ptr ptr = allGrids[0];
	FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
	mSource.transform()=mSignedLevelSet->transform();
	mSignedLevelSet->setTransform(openvdb::math::Transform::createLinearTransform(1.0));
	mSource.mSignedLevelSet=mSignedLevelSet;
	mSource.mConstellation.updateBoundingBox();
	SimulationTimeStepDescription& simDesc=mTimeSteps[mSimulationIteration];

	mSimulationTime=simDesc.mSimulationTime;
	mTimeStep=simDesc.mTimeStep;
	mSimulationDuration=simDesc.mSimulationDuration;
	mName=simDesc.mSimulationName;

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
}
SimulationPlayback::~SimulationPlayback() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
