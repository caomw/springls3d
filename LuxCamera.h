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
	std::string geometryFie;
public:
	LuxCamera();
	void setGeometryFile(const std::string& meshFile);
	bool write(std::string& file);
	virtual ~LuxCamera();
};

} /* namespace imagesci */

#endif /* LUXEXPORTER_H_ */
