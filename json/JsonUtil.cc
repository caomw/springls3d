#include "JsonUtil.h"



bool JsonUtil::Serialize( JsonSerializable* pObj, std::string& output )
{
	if (pObj == NULL)
		return false;

	Json::Value serializeRoot;
	pObj->serialize(serializeRoot);

	Json::StyledWriter writer;
	output = writer.write( serializeRoot );

	return true;
}

bool JsonUtil::Deserialize( JsonSerializable* pObj, std::string& input )
{
	if (pObj == NULL)
		return false;

	Json::Value deserializeRoot;
	Json::Reader reader;

	if ( !reader.parse(input, deserializeRoot) )
		return false;

	pObj->deserialize(deserializeRoot);

	return true;
}

bool JsonUtil::FloatArrayToJson(const float* vector_in, Json::Value& node_out,int size)
{
	node_out.resize(static_cast<Json::Value::UInt>(size));

	if(node_out.size() != size)
		return false;

	for(int i = 0; i < size; i++)
	{
		node_out[i] = vector_in[i];
	}

	return true;
}



bool JsonUtil::JsonToFloatArray(Json::Value& node_in,float* vector_out,int size)
{
	if(size != node_in.size())
		return false;

	for(unsigned int i = 0; i < node_in.size(); i++)
	{
		vector_out[i] = node_in[i].asFloat();
	}

	return true;

}
bool JsonUtil::FloatVectorToJson(const std::vector<float> &vector_in, Json::Value& node_out)
{
	node_out.resize(static_cast<Json::Value::UInt>(vector_in.size()));

	if(node_out.size() != vector_in.size())
		return false;

	for(unsigned int i = 0; i < vector_in.size(); i++)
	{
		node_out[i] = vector_in[i];
	}

	return true;
}
bool JsonUtil::IntVectorToJson(const std::vector<int> &vector_in, Json::Value& node_out)
{
	node_out.resize(static_cast<Json::Value::UInt>(vector_in.size()));
	if(node_out.size() != vector_in.size())
		return false;

	for(unsigned int i = 0; i < vector_in.size(); i++)
	{
		node_out[i] = vector_in[i];
	}

	return true;
}
bool JsonUtil::JsonToIntVector(Json::Value& node_in, std::vector<int> &vector_out)
{

	vector_out.resize(node_in.size());
	if(vector_out.size() != node_in.size())
		return false;

	for(unsigned int i = 0; i < node_in.size(); i++)
	{
		vector_out[i] = node_in[i].asInt();
	}
	return true;
}

bool JsonUtil::JsonToFloatVector(Json::Value& node_in, std::vector<float> &vector_out)
{

	vector_out.resize(node_in.size());
	if(vector_out.size() != node_in.size())
		return false;
	for(unsigned int i = 0; i < node_in.size(); i++)
	{
		vector_out[i] = node_in[i].asFloat();
	}
	return true;
}
std::vector<float> JsonUtil::Vec4sToVector(openvdb::Vec4s &in)
{
	std::vector<float> output(4);
	for(int i=0;i<4;i++){output[i]=in[i];}
	return output;
}
std::vector<float> JsonUtil::Vec3sToVector(openvdb::Vec3s &in)
{
	std::vector<float> output(3);
	for(int i=0;i<3;i++){output[i]=in[i];}
	return output;
}
std::vector<float> JsonUtil::Vec2sToVector(openvdb::Vec2s &in)
{
	std::vector<float> output(2);
	for(int i=0;i<2;i++){output[i]=in[i];}

	return output;
}

std::vector<float> JsonUtil::Mat4sToVector(openvdb::Mat4s &M)
{
	std::vector<float> output(16);
	float* ptr=M.asPointer();
	for(int i=0;i<16;i++){
		output[i]=ptr[i];
	}
	return output;
}
openvdb::Mat4s JsonUtil::VectorToMat4s(const std::vector<float>& in)
{
	openvdb::Mat4s M;
	float* ptr=M.asPointer();
	for(int i=0;i<16;i++){
		ptr[i]=in[i];
	}
	return M;
}

std::vector<float> JsonUtil::Mat3sToVector(openvdb::Mat3s &M)
{
	std::vector<float> output(9);
	float* ptr=M.asPointer();
	for(int i=0;i<9;i++){
		output[i]=ptr[i];
	}
	return output;
}
openvdb::Mat3s JsonUtil::VectorToMat3s(const std::vector<float>& in)
{
	openvdb::Mat3s M;
	float* ptr=M.asPointer();
	for(int i=0;i<9;i++){
		ptr[i]=in[i];
	}
	return M;
}
openvdb::Vec4s JsonUtil::VectorToVec4s(const std::vector<float>& in)
{
	openvdb::Vec4s value;
	for(int i=0;i<4;i++){value[i]=in[i];}
	return value;
}
openvdb::Vec3s JsonUtil::VectorToVec3s(const std::vector<float>& in)
{
	openvdb::Vec3s value;
	for(int i=0;i<3;i++){value[i]=in[i];}
	return value;
}
openvdb::Vec2s JsonUtil::VectorToVec2s(const std::vector<float>& in)
{
	openvdb::Vec2s value;
	for(int i=0;i<2;i++){value[i]=in[i];}
	return value;
}

openvdb::Vec4i JsonUtil::VectorToVec4i(const std::vector<int>& in)
{
	openvdb::Vec4i value;
	for(int i=0;i<4;i++){value[i]=in[i];}
	return value;
}

openvdb::Vec2i JsonUtil::VectorToVec2i(const std::vector<int>& in)
{
	openvdb::Vec2i value;
	for(int i=0;i<2;i++){value[i]=in[i];}
	return value;
}
openvdb::Vec3i JsonUtil::VectorToVec3i(const std::vector<int>& in)
{
	openvdb::Vec3i value;
	for(int i=0;i<3;i++){value[i]=in[i];}
	return value;
}
std::vector<int> JsonUtil::Vec4iToVector(openvdb::Vec4i &in)
{
	std::vector<int> output(4);
	for(int i=0;i<4;i++){output[i]=in[i];}
	return output;
}
std::vector<int> JsonUtil::Vec2iToVector(openvdb::Vec2i &in)
{
	std::vector<int> output(2);
	for(int i=0;i<2;i++){output[i]=in[i];}
	return output;
}
std::vector<int> JsonUtil::Vec3iToVector(openvdb::Vec3i &in)
{
	std::vector<int> output(3);
	for(int i=0;i<3;i++){output[i]=in[i];}
	return output;
}



