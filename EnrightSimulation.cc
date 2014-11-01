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

#include "EnrightSimulation.h"
#include <openvdb/openvdb.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/LevelSetSphere.h>

namespace imagesci {

EnrightSimulation::EnrightSimulation(int gridSize,MotionScheme scheme):Simulation("Enright",scheme),mGridSize(gridSize) {
}
bool EnrightSimulation::init(){
	const float radius = 0.15f;
	const openvdb::Vec3f center(0.35f, 0.35f, 0.35f);
	float voxelSize = 1 / (float) (mGridSize - 1);
	FloatGrid::Ptr mSignedLevelSet =openvdb::tools::createLevelSetSphere<FloatGrid>(radius,center, voxelSize);
	mSource.create(*mSignedLevelSet);
    mSource.mIsoSurface.updateBoundingBox();
	mAdvect=std::unique_ptr<AdvectT>(new AdvectT(mSource,mField,mMotionScheme));
	mAdvect->setTemporalScheme(imagesci::TemporalIntegrationScheme::RK4b);
	mSimulationDuration=3.0f;
	mTimeStep=0.005;
	mIsMeshDirty=true;
	return true;
}
void EnrightSimulation::cleanup(){
	mAdvect.reset();
}
bool EnrightSimulation::step(){
	mAdvect->advect(mSimulationTime,mSimulationTime+mTimeStep);
	mIsMeshDirty=true;
	mSimulationIteration++;
	mSimulationTime=mTimeStep*mSimulationIteration;

	if(mSimulationTime<=mSimulationDuration&&mRunning){
		return true;
	} else {
		mSimulationIteration--;
		mSimulationTime=mSimulationDuration;
		return false;
	}
}

} /* namespace imagesci */
