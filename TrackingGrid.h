/*
 * TrackingGrid.h
 *
 *  Created on: Aug 17, 2014
 *      Author: blake
 */

#ifndef TRACKINGGRID_H_
#define TRACKINGGRID_H_
#include <openvdb/openvdb.h>
#include <vector>
#include <list>
namespace imagesci {

class TrackingGrid {
public:
	openvdb::FloatGrid signedLevelSet;
	openvdb::FloatGrid unsignedLevelSet;
	openvdb::Int32Grid springlPointerGrid;
	TrackingGrid();
	virtual ~TrackingGrid();
};

} /* namespace imagesci */

#endif /* TRACKINGGRID_H_ */
