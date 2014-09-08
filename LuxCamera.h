/*
 * LUXExporter.h
 *
 *  Created on: Sep 6, 2014
 *      Author: blake
 */

#ifndef LUXCAMERA_H_
#define LUXCAMERA_H_
#include "Camera.h"
namespace imagesci {

class LuxCamera:public Camera {
protected:
	std::string geometryFile;
	std::string materialFile;
	openvdb::Mat4s geomPose;
	bool writeMaterialFile(const std::string& file);
	bool writeGeometryFile(const std::string& file);
public:
	LuxCamera();
	void setGeometryFile(const std::string& meshFile,const openvdb::Mat4s& M);
	void setMaterialFile(const std::string& matFile);
	bool write(const std::string& file,int w,int h);
	virtual ~LuxCamera();
};

} /* namespace imagesci */

#endif /* LUXEXPORTER_H_ */
