/*
 * TrackingGrid.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "SpringlGrid.h"
#include "ImageSciUtil.h"
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
	glColor3f(0.8f,0.3f,0.3f);
	if(constellation.get()!=nullptr){
		constellation->draw(colorEnabled);
	}
}
SpringlGrid::SpringlGrid() {
}

SpringlGrid::~SpringlGrid() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
