/*
 * ImageSciUtil.cc
 *
 *  Created on: Aug 18, 2014
 *      Author: blake
 */

#include "ImageSciUtil.h"
#include <openvdb/tools/Dense.h>
#include <openvdb/tools/LevelSetUtil.h>
namespace imagesci {
using namespace openvdb;
using namespace openvdb::tools;
openvdb::math::Mat3<float> CreateAxisAngle(Vec3s a1,float angle){
	 openvdb::math::Mat3<float> M;
     float mag = a1.length();
     if( mag < 1E-6f ) {
		 M[0][0] = 1.0f;
		 M[0][1] = 0.0f;
		 M[0][2] = 0.0f;

		 M[1][0] = 0.0f;
		 M[1][1] = 1.0f;
		 M[1][2] = 0.0f;

		 M[2][0] = 0.0f;
	 	 M[2][1] = 0.0f;
		 M[2][2] = 1.0f;
     } else {
	 	mag = 1.0f/mag;
         float ax = a1[0]*mag;
         float ay = a1[1]*mag;
         float az = a1[2]*mag;
         float sinTheta = (float)sin(angle);
         float cosTheta = (float)cos(angle);
         float t = 1.0f - cosTheta;

         float xz = ax * az;
         float xy = ax * ay;
         float yz = ay * az;

         M[0][0] = t * ax * ax + cosTheta;
         M[0][1] = t * xy - sinTheta * az;
         M[0][2] = t * xz + sinTheta * ay;

         M[1][0] = t * xy + sinTheta * az;
         M[1][1] = t * ay * ay + cosTheta;
         M[1][2] = t * yz - sinTheta * ax;

         M[2][0] = t * xz - sinTheta * ay;
         M[2][1] = t * yz + sinTheta * ax;
         M[2][2] = t * az * az + cosTheta;
      }
      return M;
}
bool WriteToRawFile(openvdb::FloatGrid::Ptr grid,const std::string& fileName){
    std::ostringstream vstr;
    vstr << fileName<<".raw";
    FILE* f=fopen(vstr.str().c_str(),"wb");
	openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
	Dense<float> dense(bbox);//LayoutZYX is the default
	copyToDense(*grid, dense);
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
bool WriteToRawFile(openvdb::VectorGrid::Ptr grid,const std::string& fileName){
    std::ostringstream vstr;
    vstr << fileName<<".raw";
    FILE* f=fopen(vstr.str().c_str(),"wb");
	openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
	Dense<Vec3f> dense(bbox);//LayoutZYX is the default
	copyToDense(*grid, dense);
	std::cout<<"Grid size "<<dense.valueCount()<<std::endl;
	Coord dims=bbox.max()-bbox.min()+Coord(1,1,1);
	std::cout<<"Dimensions "<<dims<<std::endl;
	openvdb::Coord P(0,0,0);

	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {
            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		Vec3f val=dense.getValue(P);
        		fwrite(&val[0],sizeof(float),1,f);
    		}
     	}
    }

	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {
            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		Vec3f val=dense.getValue(P);
        		fwrite(&val[1],sizeof(float),1,f);
    		}
     	}
    }

	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {
            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		Vec3f val=dense.getValue(P);
        		fwrite(&val[2],sizeof(float),1,f);
    		}
     	}
    }

	fclose(f);
    std::cout<<vstr.str()<<std::endl;
    std::stringstream sstr;
	sstr<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
	sstr<<"<!-- MIPAV header file -->\n";
	sstr<<"<image xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" nDimensions=\"4\">\n";
	sstr<<"	<Dataset-attributes>\n";
	sstr<<"		<Image-offset>0</Image-offset>\n";
	sstr<<"		<Data-type>Float</Data-type>\n";
	sstr<<"		<Endianess>Little</Endianess>\n";
	sstr<<"		<Extents>"<<dims[0]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[1]<<"</Extents>\n";
	sstr<<"		<Extents>"<<dims[2]<<"</Extents>\n";
	sstr<<"		<Extents>3</Extents>\n";
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
bool WriteToRawFile(openvdb::Int32Grid::Ptr grid,const std::string& fileName){
    std::ostringstream vstr;
    vstr << fileName<<".raw";
    FILE* f=fopen(vstr.str().c_str(),"wb");
	openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
	Dense<Index32> dense(bbox);//LayoutZYX is the default
	copyToDense(*grid, dense);
	std::cout<<"Grid size "<<dense.valueCount()<<std::endl;
	Coord dims=bbox.max()-bbox.min()+Coord(1,1,1);
	std::cout<<"Dimensions "<<dims<<std::endl;
	openvdb::Coord P(0,0,0);
	for (P[2] = bbox.min()[2]; P[2] <= bbox.max()[2]; ++P[2]) {
    	for (P[1] = bbox.min()[1]; P[1] <= bbox.max()[1]; ++P[1]) {

            for (P[0]=bbox.min()[0] ;P[0] <= bbox.max()[0]; ++P[0]) {
        		Index32 val=dense.getValue(P);
        		fwrite(&val,sizeof(Index32),1,f);
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
float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2,openvdb::Vec3s* lastClosestSegmentPoint) {
	using namespace openvdb;
	using namespace openvdb::math;
	Vec3s dir=pt2-pt1;
	float len=dir.length();
	dir.normalize(1E-6f);
	Vec3s diff=pt-pt1;
	float mSegmentParameter = dir.dot(diff);
	if (0 < mSegmentParameter) {
		if (mSegmentParameter < len) {
			*lastClosestSegmentPoint=dir*mSegmentParameter+pt1;
		} else {
			*lastClosestSegmentPoint=pt2;
		}
	} else {
		*lastClosestSegmentPoint=pt1;
	}
	return (pt-(*lastClosestSegmentPoint)).lengthSqr();
}
float DistanceToEdge(const openvdb::Vec3s& pt,const openvdb::Vec3s& pt1,const openvdb::Vec3s& pt2) {
	openvdb::Vec3s tmp;
	return DistanceToEdge(pt,pt1,pt2,&tmp);
}
} /* namespace imagesci */
