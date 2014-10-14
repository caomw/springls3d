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

#include "Simulation.h"
#include <boost/filesystem.hpp>
#include <openvdb/openvdb.h>
namespace imagesci {
void ExecuteSimulation(Simulation* sim){
	sim->reset();
	while(sim->step()){
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

Simulation::Simulation():mIsMeshDirty(false),mRunning(false),mTimeStep(0),mSimulationDuration(0),mSimulationTime(0),mSimulationIteration(0) {
	// TODO Auto-generated constructor stub

}
bool Simulation::updateGL(){
	if(mIsMeshDirty){
		mSource.mConstellation.updateGL();
		mSource.mIsoSurface.updateGL();
		mIsMeshDirty=false;
		return true;
	} else return false;
}
bool Simulation::setSource(const std::string& fileName){
	std::string ext = boost::filesystem::extension(
			boost::filesystem::path(fileName));
	if (ext == std::string(".ply")) {
		Mesh* mesh=new Mesh();
		if(!mesh->openMesh(fileName)){
			delete mesh;
			return false;
		}
		//Normalize mesh to lie within unit cube, centered at (0.5f,0.5f,0.5f)
		mesh->mapIntoBoundingBox(mesh->estimateVoxelSize());
		BBoxd bbox=mesh->updateBBox();
		mSource.create(mesh);
		openvdb::math::Transform::Ptr trans=mSource.transformPtr();
		Vec3d extents=bbox.extents();
		double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));
		double scale=1.0/max_extent;
		const float radius = 0.15f;
		const openvdb::Vec3f center(0.5f,0.5f,0.5f);
		Vec3s t=-0.5f*(bbox.min()+bbox.max());
		trans->postTranslate(t);
		trans->postScale(scale);
		trans->postTranslate(center);
		mIsMeshDirty=true;
		return true;
	} else if (ext == std::string(".vdb")) {
		openvdb::io::File file(fileName);
		file.open();
		openvdb::GridPtrVecPtr grids = file.getGrids();
		openvdb::GridPtrVec allGrids;
		allGrids.insert(allGrids.end(), grids->begin(), grids->end());
		GridBase::Ptr ptr = allGrids[0];
		Mesh* mesh = new Mesh();
		FloatGrid::Ptr mSignedLevelSet=boost::static_pointer_cast<FloatGrid>(ptr);
	    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
	    mSource.create(*mSignedLevelSet);
	    mSource.mIsoSurface.updateBBox();
	    mSource.transform()=mSource.mSignedLevelSet->transform();
	    mSource.mSignedLevelSet->transform()=*trans;
	    mIsMeshDirty=true;
		return true;
	}
	return false;
}
void Simulation::reset(){
	mSimulationTime=0;
	mSimulationIteration=0;
}
bool Simulation::stop(){
	if(!mRunning)return true;
	mRunning=false;
	if(mSimulationThread.joinable()){
		mSimulationThread.join();
	} else {
		return false;
	}
	return true;
}
bool Simulation::start(){
	if(!mRunning)return false;
	mRunning=true;
	mSimulationThread=std::thread(ExecuteSimulation,this);
	return true;
}
Simulation::~Simulation() {
	stop();
}

} /* namespace imagesci */
