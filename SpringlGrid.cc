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
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
bool SpringlGrid::create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform){
	MeshToVolume<FloatGrid> mtol(transform,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToLevelSet(mesh.points,mesh.faces);
	signedLevelSet=mtol.distGridPtr();
	springlPointerGrid=mtol.indexGridPtr();
    Constellation* c=new Constellation();
	c->create(signedLevelSet);
	constellation=std::unique_ptr<Constellation>(c);
	return true;
}
void SpringlGrid::draw(bool colorEnabled){
	if(constellation.get()!=nullptr){
		glColor3f(0.8f,0.3f,0.3f);
		constellation->draw(colorEnabled);
	}
}
void SpringlGrid::updateNearestNeighbors(){

}
void SpringlGrid::updateUnsignedLevelSet(){
	openvdb::math::Transform::Ptr trans=openvdb::math::Transform::createLinearTransform();
	MeshToVolume<FloatGrid> mtol(trans,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToUnsignedDistanceField(constellation->points,constellation->faces,float(LEVEL_SET_HALF_WIDTH)*2);
	unsignedLevelSet=mtol.distGridPtr();
	unsignedLevelSet->setBackground(float(LEVEL_SET_HALF_WIDTH)*2);
	springlPointerGrid=mtol.indexGridPtr();
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
