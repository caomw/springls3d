/*
 * LUXExporter.cc
 *
 *  Created on: Sep 6, 2014
 *      Author: blake
 */

#include "LuxCamera.h"

namespace imagesci {

LuxCamera::LuxCamera():Camera() {
	// TODO Auto-generated constructor stub

}

LuxCamera::~LuxCamera() {
	// TODO Auto-generated destructor stub
}
void LuxCamera::setGeometryFile(const std::string& meshFile){

}
bool LuxCamera::write(std::string& file){
	return true;
}
} /* namespace imagesci */
