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
	float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2,openvdb::Vec3s* lastClosestSegmentPoint);
	float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2);
}
#endif /* IMAGESCIUTIL_H_ */
