/*
 * TrackingGrid.cc
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#include "SpringlGrid.h"

#include <openvdb/tools/MeshToVolume.h>
using namespace openvdb;
using namespace openvdb::tools;
namespace imagesci {
bool SpringlGrid::create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform){
	MeshToVolume<FloatGrid> mtol(transform,GENERATE_PRIM_INDEX_GRID);
	mtol.convertToLevelSet(mesh.points,mesh.faces);
	signedLevelSet=mtol.distGridPtr();
	springlPointerGrid=mtol.indexGridPtr();
	return true;
}
SpringlGrid::SpringlGrid() {
}

SpringlGrid::~SpringlGrid() {
	// TODO Auto-generated destructor stub
}

} /* namespace imagesci */
