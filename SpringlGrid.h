/*
 * TrackingGrid.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef SPRINGLGRID_H_
#define SPRINGLGRID_H_
#include <openvdb/openvdb.h>
#include <vector>
#include <list>
#include "Mesh.h"
#include "Constellation.h"
namespace imagesci {
class SpringlGrid {
public:
	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::Int32Grid::Ptr springlPointerGrid;
	boost::shared_ptr<Constellation> constellation;

	void draw(bool colorEnabled);

	bool create(const Mesh& mesh,openvdb::math::Transform::Ptr& transform);
	SpringlGrid();
	virtual ~SpringlGrid();
};

}
#endif /* TRACKINGGRID_H_ */
