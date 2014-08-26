/*
 * ImageSciUtil.h
 *
 *  Created on: Aug 18, 2014
 *      Author: blake
 */

#ifndef IMAGESCIUTIL_H_
#define IMAGESCIUTIL_H_
#include <openvdb/openvdb.h>
namespace imagesci {
	bool WriteToRawFile(openvdb::FloatGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::VectorGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::Int32Grid::Ptr grid,const std::string& fileName);
	//Distance between point and triangle edge
	//Implementation from geometric tools (http://www.geometrictools.com)
	float DistanceToEdge(openvdb::Vec3s pt, openvdb::Vec3s pt1, openvdb::Vec3s pt2,openvdb::Vec3s* lastClosestSegmentPoint);
	float DistanceToEdge(openvdb::Vec3s pt, openvdb::Vec3s pt1, openvdb::Vec3s pt2);
}
#endif /* IMAGESCIUTIL_H_ */
