#pragma once
#include "JsonSerializable.h"
#include <openvdb/openvdb.h>
namespace openvdb{
	typedef openvdb::math::Mat3<float>   Mat3s;
}
class JsonSerializer
{
private:
	static std::vector<float> Vec4sToVector(openvdb::Vec4s &in);
	static openvdb::Vec4s VectorToVec4s(const std::vector<float> &vector_in);

	static std::vector<float> Vec3sToVector(openvdb::Vec3s &in);
	static openvdb::Vec3s VectorToVec3s(const std::vector<float> &vector_in);

	static std::vector<float> Vec2sToVector(openvdb::Vec2s &in);
	static openvdb::Vec2s VectorToVec2s(const std::vector<float> &vector_in);

	static std::vector<int> Vec2iToVector(openvdb::Vec2i &in);
	static openvdb::Vec2i VectorToVec2i(const std::vector<int> &vector_in);

	static std::vector<int> Vec3iToVector(openvdb::Vec3i &in);
	static openvdb::Vec3i VectorToVec3i(const std::vector<int> &vector_in);

	static std::vector<int> Vec4iToVector(openvdb::Vec4i &in);
	static openvdb::Vec4i VectorToVec4i(const std::vector<int> &vector_in);

	static std::vector<float> Mat4sToVector(openvdb::Mat4s &in);
	static std::vector<float> Mat3sToVector(openvdb::Mat3s &in);

	static openvdb::Mat4s VectorToMat4s(const std::vector<float> &vector_in);
	static openvdb::Mat3s VectorToMat3s(const std::vector<float> &vector_in);

public:
	static bool Serialize( JsonSerializable* pObj, std::string& output );
	static bool Deserialize( JsonSerializable* pObj, std::string& input );

	static bool FloatVectorToJson(const std::vector<float> &vector_in, Json::Value& node_out);
	
	static bool JsonToFloatVector(Json::Value& node_in, std::vector<float> &vector_out);
	
	static bool IntVectorToJson(const std::vector<int> &vector_in, Json::Value& node_out);
	static bool JsonToIntVector(Json::Value& node_in, std::vector<int> &vector_out);

	static bool FloatArrayToJson(const float* vector_in, Json::Value& node_out,int size);
	static bool JsonToFloatArray(Json::Value& node_in,float* vector_out,int size);
	static openvdb::Mat4s JsonToMat4s(Json::Value& in){
		std::vector<float> out;
		JsonToFloatVector(in,out);
		return VectorToMat4s(out);
	}
	static openvdb::Mat3s JsonToMat3s(Json::Value& in){
		std::vector<float> out;
		JsonToFloatVector(in,out);
		return VectorToMat3s(out);
	}
	static openvdb::Vec4s JsonToVec4s(Json::Value& in){
		std::vector<float> out;
		JsonToFloatVector(in,out);
		return VectorToVec4s(out);
	}
	static openvdb::Vec3s JsonToVec3s(Json::Value& in){
		std::vector<float> out;
		JsonToFloatVector(in,out);
		return VectorToVec3s(out);
	}
	static openvdb::Vec2s JsonToVec2s(Json::Value& in){
		std::vector<float> out;
		JsonToFloatVector(in,out);
		return VectorToVec2s(out);
	}

	static openvdb::Vec4i JsonToVec4i(Json::Value& in){
		std::vector<int> out;
		JsonToIntVector(in,out);
		return VectorToVec4i(out);
	}
	static openvdb::Vec3i JsonToVec3i(Json::Value& in){
		std::vector<int> out;
		JsonToIntVector(in,out);
		return VectorToVec3i(out);
	}
	static openvdb::Vec2i JsonToVec2i(Json::Value& in){
		std::vector<int> out;
		JsonToIntVector(in,out);
		return VectorToVec2i(out);
	}
	static bool Mat4sToJson(openvdb::Mat4s& in,Json::Value& out){
		return FloatVectorToJson(Mat4sToVector(in), out);
	}
	static bool Mat3sToJson(openvdb::Mat3s& in,Json::Value& out){
		return FloatVectorToJson(Mat3sToVector(in), out);
	}
	static bool Vec4sToJson(openvdb::Vec4s& in,Json::Value& out){
		return FloatVectorToJson(Vec4sToVector(in), out);
	}
	static bool Vec3sToJson(openvdb::Vec3s& in,Json::Value& out){
		return FloatVectorToJson(Vec3sToVector(in), out);
	}
	static bool Vec2sToJson(openvdb::Vec2s& in,Json::Value& out){
		return FloatVectorToJson(Vec2sToVector(in), out);
	}
	static bool Vec4iToJson(openvdb::Vec4i& in,Json::Value& out){
		return IntVectorToJson(Vec4iToVector(in), out);
	}
	static bool Vec3iToJson(openvdb::Vec3i& in,Json::Value& out){
		return IntVectorToJson(Vec3iToVector(in), out);
	}
	static bool Vec2iToJson(openvdb::Vec2i& in,Json::Value& out){
		return IntVectorToJson(Vec2iToVector(in), out);
	}
private:
	JsonSerializer( void ) {};
};

