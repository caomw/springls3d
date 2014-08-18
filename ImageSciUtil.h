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
static bool WriteToRawFile(openvdb::FloatGrid::Ptr grid,const std::string fileName);
static bool WriteToRawFile(openvdb::Int32Grid::Ptr grid,const std::string fileName);
}
#endif /* IMAGESCIUTIL_H_ */
