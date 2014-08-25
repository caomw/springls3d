/*
 * TrackingGrid.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "SpringlGrid.h"
#include "ImageSciUtil.h"
#include "AdvectionForce.h"
#include <openvdb/tools/MeshToVolume.h>
#include <iterator>
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
bool SpringlGrid::create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform){
	MeshToVolume<FloatGrid> mtol(transform,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToLevelSet(mesh.vertexes,mesh.faces);
	signedLevelSet=mtol.distGridPtr();
	springlIndexGrid=mtol.indexGridPtr();
    Mesh* c=new Mesh();
	c->create(signedLevelSet);
	isoSurface=std::unique_ptr<Mesh>(c);
	return true;
}
void SpringlGrid::draw(bool colorEnabled){
	if(isoSurface.get()!=nullptr){
		glColor3f(0.8f,0.3f,0.3f);
		isoSurface->draw(colorEnabled);
	}
}
void SpringlGrid::updateNearestNeighbors(bool threaded){
	NearestNeighbors<openvdb::Int32> nn(*this);
	nn.process();
}

void SpringlGrid::updateUnsignedLevelSet(){
	openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
	MeshToVolume<FloatGrid> mtol(trans,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(isoSurface->vertexes,isoSurface->faces,float(LEVEL_SET_HALF_WIDTH)*2);
	unsignedLevelSet=mtol.distGridPtr();
	unsignedLevelSet->setBackground(float(LEVEL_SET_HALF_WIDTH)*2);
	springlIndexGrid=mtol.indexGridPtr();
}
void SpringlGrid::updateGradient(){
	//gradient = openvdb::tools::gradient(*unsignedLevelSet);
	gradient = advectionForce(*unsignedLevelSet);

}
SpringlGrid::SpringlGrid() {
}

SpringlGrid::~SpringlGrid() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
