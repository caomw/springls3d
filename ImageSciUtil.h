/*
 * Copyright(C) 2014, Blake C. Lucas, Ph.D. (img.science@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef IMAGESCIUTIL_H_
#define IMAGESCIUTIL_H_
#include <openvdb/openvdb.h>
#include <openvdb/tools/Dense.h>

#ifdef MP
#include <omp.h>
#pragma omp
#define OPENMP_FOR		_Pragma("omp parallel for" )
#define OPENMP_SECTION  _Pragma("omp section" )
#define OPENMP_BEGIN	_Pragma("omp parallel" ) {
#define OPENMP_END		}
#define OPENMP_FOR_P	_Pragma("omp for" )
#else
#define OPENMP_FOR
#define OPENMP_SECTION
#define OPENMP_BEGIN
#define OPENMP_END
#define OPENMP_FOR_P
#endif
namespace imagesci {

typedef openvdb::math::Vec4<unsigned char> RGBA;
typedef openvdb::math::Vec4<float> RGBAf;
typedef openvdb::math::Vec2<float> UV;
typedef openvdb::math::Vec3<float> float3;
typedef openvdb::math::Vec4<float> float4;
typedef openvdb::math::Mat4f Matrix4f;
template<typename T> T clamp(T val, T min, T max) {
	return std::min(std::max(val, min), max);
}
struct plyVertex {
	float x[3];             // the usual 3-space position of a vertex
	unsigned char red;
	unsigned char green;
	unsigned char blue;
	unsigned char alpha;
	plyVertex() {
		x[0] = 0;
		x[1] = 0;
		x[2] = 0;
		red = 0;
		green = 0;
		blue = 0;
		alpha = 0;
	}
};
struct plyParticle {
	float x[3];             // the usual 3-space position of a vertex
	float radius;
	plyParticle() {
		x[0] = 0;
		x[1] = 0;
		x[2] = 0;
		radius=0.0f;
	}
};


typedef struct _plyFace {
	unsigned char nverts;    // number of vertex indices in list
	int *verts;              // vertex index list
	_plyFace() {
		nverts = 0;
		verts = NULL;
	}
} plyFace;
typedef struct _plyFaceTexutre {
	unsigned char nverts;    // number of vertex indices in list
	int *verts;              // vertex index list
	unsigned char uvcount;
	float* uvs;
	_plyFaceTexutre(){
		nverts = 0;
		uvs=NULL;
		verts = NULL;
		uvcount = 6;
	}
} plyFaceTexutre;

class Exception:public std::exception{
protected:
	std::string message;
public:

	Exception(const char* str){
		message=str;
	}
	Exception(const std::string& str){
		message=str;
	}
	const char* what(){
		return message.c_str();
	}
};
std::string GetFileWithoutExtension(const std::string& file);
std::string GetFileNameWithoutExtension(const std::string& file);
std::string GetFileDirectoryPath(const std::string& file);
std::string GetFileName(const std::string& file);

int GetDirectoryListing(const std::string& dirName,
		std::vector<std::string>& files, const std::string& mask,
		const std::string& ext);
bool ReadImageFromFile(const std::string& file,
		std::vector<openvdb::math::Vec4<unsigned char>>& image, int& w, int& h);
bool WriteImageToFile(const std::string& file,
		const std::vector<RGBA>& image,
		const int w, const int h);
bool WriteImageToFile(const std::string& file,
		const std::vector<openvdb::math::Vec4s>& image,
		const int w, const int h);
bool WriteToRawFile(openvdb::FloatGrid::Ptr grid, const std::string& fileName);
bool WriteToRawFile(openvdb::tools::Dense<float,openvdb::tools::MemoryLayout::LayoutZYX>& grid, const std::string& fileName);
bool WriteToRawFile(openvdb::VectorGrid::Ptr grid, const std::string& fileName);
bool WriteToRawFile(openvdb::Int32Grid::Ptr grid, const std::string& fileName);
float DistanceToEdgeSqr(const openvdb::Vec3s& pt, const openvdb::Vec3s& pt1,
		const openvdb::Vec3s& pt2, openvdb::Vec3s* lastClosestSegmentPoint);
float DistanceToEdgeSqr(const openvdb::Vec3s& pt, const openvdb::Vec3s& pt1,
		const openvdb::Vec3s& pt2);
float DistanceToTriangleSqr(const openvdb::Vec3s& p, const openvdb::Vec3s& v0,
		const openvdb::Vec3s& v1, const openvdb::Vec3s& v2,
		openvdb::Vec3s* closestPoint);
float DistanceToQuadSqr(const openvdb::Vec3s& p, const openvdb::Vec3s& v0,
		const openvdb::Vec3s& v1, const openvdb::Vec3s& v2,
		const openvdb::Vec3s& v3, const openvdb::Vec3s& normal,
		openvdb::Vec3s* closestPoint);
const std::string ReadTextFile(const std::string& str);
float Angle(openvdb::Vec3s& v0, openvdb::Vec3s& v1, openvdb::Vec3s& v2);
openvdb::math::Mat3<float> CreateAxisAngle(openvdb::Vec3s axis, float angle);
}
#endif /* IMAGESCIUTIL_H_ */
