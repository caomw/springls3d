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

#include "ArmadilloTwist.h"

namespace imagesci {

ArmadilloTwist::ArmadilloTwist(const std::string& fileName):Simulation(),mSourceFileName(fileName) {
	//if(!setSource(fileName))throw Exception("Could not open "+fileName);
}

ArmadilloTwist::~ArmadilloTwist() {
	// TODO Auto-generated destructor stub
}

bool ArmadilloTwist::init(){
	std::cout<<"SOURCE FILE "<<mSourceFileName<<std::endl;
	//if(mSourceFileName.length()>0)setSource(mSourceFileName);
	Mesh mesh;
	if(!mesh.openMesh(mSourceFileName))return false;

	mesh.mapIntoBoundingBox(2*mesh.estimateVoxelSize());
	mesh.updateBoundingBox();
	std::cout<<"Bounding Box "<<mesh.getBoundingBox()<<" "<<mesh.mVertexes.size()<<std::endl;
    openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
    mSource.create(&mesh);
    BBoxd bbox=mSource.mIsoSurface.updateBoundingBox();
	trans=mSource.mSignedLevelSet->transformPtr();
    Vec3d extents=bbox.extents();
	double max_extent = std::max(extents[0], std::max(extents[1], extents[2]));
	double scale=1.0/max_extent;
	const float radius = 0.15f;
    const openvdb::Vec3f center(0.35f,0.35f,0.35f);
	Vec3s t=-0.5f*(bbox.min()+bbox.max());
	trans=mSource.transformPtr();
	trans->postTranslate(t);
	trans->postScale(scale*2*radius);
	trans->postTranslate(center);
	mIsMeshDirty=true;
	return true;
}
bool ArmadilloTwist::step(){
	mSimulationIteration++;
	return false;
}
void ArmadilloTwist::cleanup(){

}
} /* namespace imagesci */
