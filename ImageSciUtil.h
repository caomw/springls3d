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
	int GetDirectoryListing(const std::string& dirName,
			std::vector<std::string>& files,
			const std::string& mask,
			const std::string& ext);
	bool WriteToRawFile(openvdb::FloatGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::VectorGrid::Ptr grid,const std::string& fileName);
	bool WriteToRawFile(openvdb::Int32Grid::Ptr grid,const std::string& fileName);
	float DistanceToEdgeSqr(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2,openvdb::Vec3s* lastClosestSegmentPoint);
	float DistanceToEdgeSqr(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2);
	float DistanceToTriangleSqr(
			const openvdb::Vec3s& p,
			const openvdb::Vec3s& v0,
			const openvdb::Vec3s& v1,
			const openvdb::Vec3s& v2,
			openvdb::Vec3s* closestPoint);
	float DistanceToQuadSqr(
			const openvdb::Vec3s& p,
			const openvdb::Vec3s& v0,
			const openvdb::Vec3s& v1,
			const openvdb::Vec3s& v2,
			const openvdb::Vec3s& v3,
			const openvdb::Vec3s& normal,
			openvdb::Vec3s* closestPoint);
	float Angle(openvdb::Vec3s& v0, openvdb::Vec3s& v1,
			openvdb::Vec3s& v2);
	openvdb::math::Mat3<float> CreateAxisAngle(openvdb::Vec3s axis,float angle);
}
#endif /* IMAGESCIUTIL_H_ */
