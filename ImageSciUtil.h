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
	template<typename T> T clamp(T val,T min,T max){
		return std::min(std::max(val,min),max);
	}
	bool WriteToRawFile(openvdb::FloatGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::VectorGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::Int32Grid::Ptr grid,const std::string& fileName);
	float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2,openvdb::Vec3s* lastClosestSegmentPoint);
	float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2);
	openvdb::math::Mat3<float> CreateAxisAngle(openvdb::Vec3s axis,float angle);
}
#endif /* IMAGESCIUTIL_H_ */
