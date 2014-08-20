/*
 * ImageSciUtil.cc
 *
 *  Created on: Aug 18, 2014
 *      Author: blake
 */

#include "ImageSciUtil.h"
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/PointAdvect.h>
#include <openvdb/tools/PointScatter.h>
#include <openvdb/tools/ValueTransformer.h>
#include <openvdb/tools/VectorTransformer.h>
namespace imagesci {
using namespace openvdb;
using namespace openvdb::tools;
bool WriteToRawFile(openvdb::FloatGrid::Ptr mGrid,const std::string& fileName){
    std::ostringstream vstr;
    vstr << fileName<<".raw";
    FILE* f=fopen(vstr.str().c_str(),"wb");
	openvdb::CoordBBox bbox = mGrid->evalActiveVoxelBoundingBox();
	Dense<float> dense(bbox);//LayoutZYX is the default
	copyToDense(*mGrid, dense);
	std::cout<<"Grid size "<<dense.valueCount()<<std::endl;
	Coord dims=bbox.max()-bbox.min()+Coord(1,1,1);
	std::cout<<"Dimensions "<<dims<<std::endl;
	openvdb::Coord P(0,0,0);
	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {

            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		float val=dense.getValue(P);
        		fwrite(&val,sizeof(float),1,f);
    		}
     	}
    }
    fclose(f);
    std::cout<<vstr.str()<<std::endl;
    std::stringstream sstr;
	sstr<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	sstr<<"<!-- MIPAV header file -->\n";
	sstr<<"<image xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" nDimensions=\"3\">\n";
	sstr<<"	<Dataset-attributes>\n";
	sstr<<"		<Image-offset>0</Image-offset>\n";
	sstr<<"		<Data-type>Float</Data-type>\n";
	sstr<<"		<Endianess>Little</Endianess>\n";
	sstr<<"		<Extents>"<<dims[0]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[1]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[2]<<"</Extents>\n";
	sstr<<"		<Resolutions>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"		</Resolutions>\n";
	sstr<<"		<Slice-spacing>1.0</Slice-spacing>\n";
	sstr<<"		<Slice-thickness>0.0</Slice-thickness>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Compression>none</Compression>\n";
	sstr<<"		<Orientation>Unknown</Orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Modality>Unknown Modality</Modality>\n";
	sstr<<"	</Dataset-attributes>\n";
	sstr<<"</image>\n";
	std::ofstream myfile;
    std::stringstream xmlFile;
	xmlFile << fileName << ".xml";
	myfile.open(xmlFile.str().c_str(),std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	std::cout<<xmlFile.str()<<std::endl;
	return true;
}
bool WriteToRawFile(openvdb::Int32Grid::Ptr mGrid,const std::string& fileName){
    std::ostringstream vstr;
    vstr << fileName<<".raw";
    FILE* f=fopen(vstr.str().c_str(),"wb");
	openvdb::CoordBBox bbox = mGrid->evalActiveVoxelBoundingBox();
	Dense<Int32> dense(bbox);//LayoutZYX is the default
	copyToDense(*mGrid, dense);
	std::cout<<"Grid size "<<dense.valueCount()<<std::endl;
	Coord dims=bbox.max()-bbox.min()+Coord(1,1,1);
	std::cout<<"Dimensions "<<dims<<std::endl;
	openvdb::Coord P(0,0,0);
	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {

            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		float val=dense.getValue(P);
        		fwrite(&val,sizeof(float),1,f);
    		}
     	}
    }
    fclose(f);
    std::cout<<vstr.str()<<std::endl;
    std::stringstream sstr;
	sstr<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	sstr<<"<!-- MIPAV header file -->\n";
	sstr<<"<image xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" nDimensions=\"3\">\n";
	sstr<<"	<Dataset-attributes>\n";
	sstr<<"		<Image-offset>0</Image-offset>\n";
	sstr<<"		<Data-type>Unsigned Integer</Data-type>\n";
	sstr<<"		<Endianess>Little</Endianess>\n";
	sstr<<"		<Extents>"<<dims[0]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[1]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[2]<<"</Extents>\n";
	sstr<<"		<Resolutions>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"			<Resolution>1.0</Resolution>\n";
	sstr<<"		</Resolutions>\n";
	sstr<<"		<Slice-spacing>1.0</Slice-spacing>\n";
	sstr<<"		<Slice-thickness>0.0</Slice-thickness>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Units>Millimeters</Units>\n";
	sstr<<"		<Compression>none</Compression>\n";
	sstr<<"		<Orientation>Unknown</Orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Subject-axis-orientation>Unknown</Subject-axis-orientation>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Origin>0.0</Origin>\n";
	sstr<<"		<Modality>Unknown Modality</Modality>\n";
	sstr<<"	</Dataset-attributes>\n";
	sstr<<"</image>\n";
	std::ofstream myfile;
    std::stringstream xmlFile;
	xmlFile << fileName << ".xml";
	myfile.open(xmlFile.str().c_str(),std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	std::cout<<xmlFile.str()<<std::endl;
	return true;
}

} /* namespace imagesci */
