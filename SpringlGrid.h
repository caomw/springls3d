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

template<typename GridType> class SpringlGrid<GridType> {
public:
	openvdb::Grid<GridType>::Ptr signedLevelSet;
	openvdb::Grid<GridType>::Ptr unsignedLevelSet;
	openvdb::Int32Grid::Ptr springlPointerGrid;
	SpringlGrid();
	virtual ~SpringlGrid();
};
typedef SpringlGrid<openvdb::FloatTree> FloatSGrid;

}
#endif /* TRACKINGGRID_H_ */
