/*
 * LUXExporter.cc
 *
 *  Created on: Sep 6, 2014
 *      Author: blake
 */

#include "LuxCamera.h"
#include "ImageSciUtil.h"
namespace imagesci {

LuxCamera::LuxCamera():Camera() {
	// TODO Auto-generated constructor stub

}

LuxCamera::~LuxCamera() {
	// TODO Auto-generated destructor stub
}
void LuxCamera::setGeometryFile(const std::string& meshFile,const openvdb::Mat4s& Pose){
	geometryFile=meshFile;
	geomPose=Pose;
}
bool LuxCamera::writeMaterialFile(const std::string& file){
	std::stringstream sstr;
	sstr<<"MakeNamedMaterial \"MatteMaterial\""<<std::endl;
	sstr<<"	\"color Kd\" [0.63999999 0.63999999 0.63999999]"<<std::endl;
	sstr<<"	\"float sigma\" [0.000000000000000]"<<std::endl;
	sstr<<"	\"string type\" [\"matte\"]"<<std::endl;
	std::ofstream myfile;
	myfile.open(file, std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	std::cout << file << std::endl;
	return true;
}
bool LuxCamera::writeGeometryFile(const std::string& geomFile){
	//std::cout<<"Pose\n"<<geomPose<<std::endl;
	std::stringstream sstr;

	sstr<<"AttributeBegin #  \"mesh\""<<std::endl;
    sstr<<""<<std::endl;
	sstr<<"Transform [";
	for(int i=0;i<15;i++){
		float val=geomPose.asPointer()[i];
		sstr<<val<<" ";
	}
	sstr<<geomPose.asPointer()[15]<<"]"<<std::endl;
    sstr<<""<<std::endl;
	sstr<<"NamedMaterial \"MatteMaterial\""<<std::endl;
    sstr<<""<<std::endl;
	sstr<<"Shape \"plymesh\""<<std::endl;
	sstr<<"	\"string filename\" [\""<<geometryFile<<"\"]"<<std::endl;
	sstr<<"	\"bool generatetangents\" [\"false\"]"<<std::endl;
	sstr<<"	\"string name\" [\"mesh\"]"<<std::endl;
    sstr<<""<<std::endl;
	sstr<<"AttributeEnd # \"\""<<std::endl;
	std::ofstream myfile;
	myfile.open(geomFile, std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	std::cout << geomFile << std::endl;
	return true;
}
void LuxCamera::setMaterialFile(const std::string& matFile){
	materialFile=matFile;
}
bool LuxCamera::write(const std::string& file,int w,int h){
	std::string rootFile=GetFileWithoutExtension(file);
	std::string imgFile=GetFileNameWithoutExtension(file);
	std::string geomFile=rootFile+".lxo";
	std::string matFile=rootFile+".lxm";
	std::string mainFile=rootFile+".lxs";
	std::stringstream sstr;
	sstr<<"# Main Scene File"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Renderer \"sampler\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Sampler \"metropolis\""<<std::endl;
	sstr<<"	\"float largemutationprob\" [0.400000005960464]"<<std::endl;
	sstr<<"	\"bool noiseaware\" [\"false\"]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Accelerator \"qbvh\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"SurfaceIntegrator \"bidirectional\""<<std::endl;
	sstr<<"	\"integer eyedepth\" [16]"<<std::endl;
	sstr<<"	\"integer lightdepth\" [16]"<<std::endl;
	sstr<<"	\"integer lightraycount\" [1]"<<std::endl;
	sstr<<"	\"string lightpathstrategy\" [\"auto\"]"<<std::endl;
	sstr<<"	\"string lightstrategy\" [\"auto\"]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"VolumeIntegrator \"multi\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"PixelFilter \"mitchell\""<<std::endl;
	sstr<<"	\"bool supersample\" [\"true\"]"<<std::endl;
	sstr<<"	\"float B\" [0.333333343267441]"<<std::endl;
	sstr<<"	\"float C\" [0.333333343267441]"<<std::endl;
	sstr<<"	\"float xwidth\" [2.000000000000000]"<<std::endl;
	sstr<<"	\"float ywidth\" [2.000000000000000]"<<std::endl;
	sstr<<""<<std::endl;
//	sstr<<"LookAt "<<std::setprecision(6)<<mEye[0]<<" "<<mEye[1]<<" "<<mEye[2]<<" "<<mLookAt[0]<<" "<<mLookAt[1]<<" "<<mLookAt[2]<<" "<<mUp[0]<<" "<<mUp[1]<<" "<<mUp[2]<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Camera \"perspective\""<<std::endl;
	sstr<<"	\"float fov\" ["<<mFov<<"]"<<std::endl;
	sstr<<"	\"float screenwindow\" [-1.000000000000000 1.000000000000000 -1.000000000000000 1.000000000000000]"<<std::endl;
	sstr<<"	\"bool autofocus\" [\"false\"]"<<std::endl;
	sstr<<"	\"float shutteropen\" [0.000000000000000]"<<std::endl;
	sstr<<"	\"float shutterclose\" [0.041666666666667]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Film \"fleximage\""<<std::endl;
	sstr<<"	\"integer xresolution\" ["<<w<<"]"<<std::endl;
	sstr<<"	\"integer yresolution\" ["<<h<<"]"<<std::endl;
	sstr<<"	\"float gamma\" [2.200000000000000]"<<std::endl;
	sstr<<"	\"float colorspace_white\" [0.314275000000000 0.329411000000000]"<<std::endl;
	sstr<<"	\"float colorspace_red\" [0.630000000000000 0.340000000000000]"<<std::endl;
	sstr<<"	\"float colorspace_green\" [0.310000000000000 0.595000000000000]"<<std::endl;
	sstr<<"	\"float colorspace_blue\" [0.155000000000000 0.070000000000000]"<<std::endl;
	sstr<<"	\"string filename\" [\""<<imgFile<<"\"]"<<std::endl;
	sstr<<"	\"bool write_resume_flm\" [\"false\"]"<<std::endl;
	sstr<<"	\"bool restart_resume_flm\" [\"false\"]"<<std::endl;
	sstr<<"	\"bool write_flm_direct\" [\"false\"]"<<std::endl;
	sstr<<"	\"bool write_exr_halftype\" [\"false\"]"<<std::endl;
	sstr<<"	\"bool write_exr_applyimaging\" [\"true\"]"<<std::endl;
	sstr<<"	\"bool write_exr_ZBuf\" [\"false\"]"<<std::endl;
	sstr<<"	\"string write_exr_compressiontype\" [\"PIZ (lossless)\"]"<<std::endl;
	sstr<<"	\"string write_exr_zbuf_normalizationtype\" [\"None\"]"<<std::endl;
	sstr<<"	\"bool write_exr\" [\"false\"]"<<std::endl;
	sstr<<"	\"string write_exr_channels\" [\"RGB\"]"<<std::endl;
	sstr<<"	\"bool write_png\" [\"true\"]"<<std::endl;
	sstr<<"	\"string write_png_channels\" [\"RGB\"]"<<std::endl;
	sstr<<"	\"bool write_png_16bit\" [\"false\"]"<<std::endl;
	sstr<<"	\"bool write_tga\" [\"false\"]"<<std::endl;
	sstr<<"	\"string write_tga_channels\" [\"RGB\"]"<<std::endl;
	sstr<<"	\"string ldr_clamp_method\" [\"cut\"]"<<std::endl;
	sstr<<"	\"integer displayinterval\" [10]"<<std::endl;
	sstr<<"	\"integer writeinterval\" [180]"<<std::endl;
	sstr<<"	\"integer flmwriteinterval\" [900]"<<std::endl;
	sstr<<"	\"integer outlierrejection_k\" [2]"<<std::endl;
	sstr<<"	\"integer tilecount\" [0]"<<std::endl;
	sstr<<"	\"string tonemapkernel\" [\"autolinear\"]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"WorldBegin"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Include \""<<matFile<<"\""<<std::endl;
	sstr<<"Include \""<<materialFile<<"\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Include \""<<geomFile<<"\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"TransformBegin #  \"Lamp\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Transform [0.989293038845062 0.009074658155441 0.145660310983658 0.000000000000000 -0.144697949290276 0.191066205501556 0.970853328704834 0.000000000000000 -0.019020602107048 -0.981535196304321 0.190333545207977 0.000000000000000 -0.010122179985046 -2.745507240295410 1.906243205070496 1.000000000000000]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"LightGroup \"default\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"TransformBegin #  \"\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"Scale -1.000000000000000 1.000000000000000 1.000000000000000"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"LightSource \"infinite\""<<std::endl;
	sstr<<"	\"float gain\" [0.109999991953373]"<<std::endl;
	sstr<<"	\"float importance\" [1.000000000000000]"<<std::endl;
	sstr<<"	\"color L\" [1.00000000 1.00000000 1.00000000]"<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"TransformEnd # \"\""<<std::endl;
	sstr<<""<<std::endl;
	sstr<<"TransformEnd # \"\""<<std::endl;
	sstr<<"WorldEnd"<<std::endl;
	std::ofstream myfile;
	myfile.open(mainFile, std::ios_base::out);
	myfile << sstr.str();
	myfile.close();
	writeGeometryFile(geomFile);
	writeMaterialFile(matFile);
	std::cout << mainFile << std::endl;
	return true;
}
} /* namespace imagesci */
