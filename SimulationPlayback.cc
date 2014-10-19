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

SimulationPlayback::SimulationPlayback(const std::string& directory):Simulation("Recording"),mDirectory(directory) {
	// TODO Auto-generated constructor stub

}
bool SimulationPlayback::init(){
	isoSurfaceFiles.clear();
	constellationFiles.clear();
	signedDistanceFiles.clear();
	int n1=GetDirectoryListing(mDirectory,isoSurfaceFiles,"_iso",".ply");
	int n2=GetDirectoryListing(mDirectory,constellationFiles,"_sls",".ply");
	int n3=GetDirectoryListing(mDirectory,signedDistanceFiles,"",".vdb");
	mSimulationDuration=3.0f;
	mTimeStep=0.005;
	mSimulationIteration=0;
	if(!(n1==n2&&n2==n3)||n1==0)return false;
	Mesh c;
	c.openMesh(constellationFiles[mSimulationIteration]);
	mSource.mConstellation.create(&c);
	mSource.mIsoSurface.openMesh(isoSurfaceFiles[mSimulationIteration]);
	mSource.mIsoSurface.updateVertexNormals(16);
	mSource.mConstellation.updateVertexNormals();
	return true;
}
bool SimulationPlayback::step(){
	mTemporaryMesh.openMesh(constellationFiles[mSimulationIteration]);
	mSource.mConstellation.create(&mTemporaryMesh);
	mSource.mConstellation.updateVertexNormals();
	mSource.mIsoSurface.openMesh(isoSurfaceFiles[mSimulationIteration]);
	mSource.mIsoSurface.updateVertexNormals(16);
	openvdb::io::File file(signedDistanceFiles[mSimulationIteration]);
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
	mSimulationIteration++;
	mSimulationTime=mTimeStep*mSimulationIteration;

	mIsMeshDirty=true;

	if(mSimulationTime<=mSimulationDuration&&mRunning){
		return true;
	} else {
		return false;
	}
}
void SimulationPlayback::cleanup(){
	isoSurfaceFiles.clear();
	constellationFiles.clear();
	signedDistanceFiles.clear();
}
SimulationPlayback::~SimulationPlayback() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
