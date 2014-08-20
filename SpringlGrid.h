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
namespace imagesci {

class SpringlGrid {
public:
	openvdb::FloatGrid::Ptr signedLevelSet;
	openvdb::FloatGrid::Ptr unsignedLevelSet;
	openvdb::Int32Grid::Ptr springlPointerGrid;
	SpringlGrid();
	virtual ~SpringlGrid();
};

}
#endif /* TRACKINGGRID_H_ */
