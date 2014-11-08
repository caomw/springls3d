///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2012-2013 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of DreamWorks Animation nor the names of
// its contributors may be used to endorse or promote products derived
// from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////
//
/// @file Camera.h
/// @brief Basic GL camera class

#ifndef OPENVDB_VIEWER_CAMERA_HAS_BEEN_INCLUDED
#define OPENVDB_VIEWER_CAMERA_HAS_BEEN_INCLUDED

#include <openvdb/Types.h>
#define GLFW_INCLUDE_GLU

#include <GL/glx.h>
#include <GL/glxext.h>
#include <GLFW/glfw3.h>
#include "GLShader.h"
#include "../json/JsonSerializable.h"
#include "../json/JsonUtil.h"
#include <fstream>
namespace imagesci{
class CameraAndSceneConfig:public JsonSerializable{
public:
	openvdb::Mat3s mWorldRotation;
	openvdb::Vec3s mWorldTranslation;

	openvdb::Mat3s mModelRotation;
	openvdb::Vec3s mModelTranslation;

	double mDistanceToObject;

	void serialize(Json::Value& root_in)
	{
		Json::Value &root = root_in["CameraAndScene"];
		JsonUtil::Mat3sToJson(mWorldRotation,root["WorldRotaiton"]);
		JsonUtil::Vec3sToJson(mWorldTranslation,root["WorldTranslation"]);

		JsonUtil::Mat3sToJson(mModelRotation,root["ModelRotaiton"]);
		JsonUtil::Vec3sToJson(mModelTranslation,root["ModelTranslation"]);

		root["DistanceToObject"]=mDistanceToObject;

	}
	void deserialize(Json::Value& root_in)
	{
		Json::Value &root = root_in["CameraAndScene"];
		mWorldRotation=JsonUtil::JsonToMat3s(root["WorldRotaiton"]);
		mWorldTranslation=JsonUtil::JsonToVec3s(root["WorldTranslation"]);

		mModelRotation=JsonUtil::JsonToMat3s(root["ModelRotaiton"]);
		mModelTranslation=JsonUtil::JsonToVec3s(root["ModelTranslation"]);

		mDistanceToObject=root.get("DistanceToObject",0.0).asDouble();
	}
	static bool load(const std::string& file,CameraAndSceneConfig* out){
		std::ifstream ifs;
		ifs.open(file, std::ifstream::in);
		if (ifs.is_open()){
			std::cout << "Reading " << file << " ... ";
			std::string str((std::istreambuf_iterator<char>(ifs)),
				std::istreambuf_iterator<char>());
			ifs.close();
			std::cout << "Done." << std::endl;
			return JsonUtil::Deserialize(out, str);
		}
		else {
			return false;
		}
	}

	bool save(const std::string& file) {
		std::ofstream ofs;
		ofs.open(file, std::ofstream::out);
		if (ofs.is_open()){
			std::cout << "Saving " << file << " ... ";
			std::string output;
			bool ret = JsonUtil::Serialize(this, output);
			if (ret)ofs << output;
			ofs.close();
			std::cout << "Done." << std::endl;
			return ret;
		}
		return false;
	}
};
class Camera:public JsonSerializable
{
protected:
    // Camera parameters
    openvdb::math::Mat4s mRw,mRm;
    openvdb::Mat4s mProjection,mView,mModel;
    openvdb::Vec3d mCameraTrans;
    double mFov, mNearPlane, mFarPlane;
    openvdb::Vec3d mLookAt, mEye;
    double mTumblingSpeed, mZoomSpeed, mStrafeSpeed;
    double mDistanceToObject;
    bool mMouseDown, mStartTumbling, mZoomMode, mChanged, mNeedsDisplay;
    double mMouseXPos, mMouseYPos;
    int mWheelPos;
public:
	void serialize(Json::Value& root_in)
	{
		Json::Value &root = root_in["Camera"];
		JsonUtil::Mat4sToJson(mModel,root["ModelMatrix"]);
		JsonUtil::Mat4sToJson(mView,root["ViewMatrix"]);
		JsonUtil::Mat4sToJson(mView,root["ProjectionMatrix"]);
	}
	void deserialize(Json::Value& root_in)
	{
		Json::Value &root = root_in["Camera"];
		mModel=JsonUtil::JsonToMat4s(root["ModelMatrix"]);
		mView=JsonUtil::JsonToMat4s(root["ViewMatrix"]);
		mProjection=JsonUtil::JsonToMat4s(root["ProjectionMatrix"]);
	}
    Camera();

    void aim(int x,int y,int width,int height,GLShader& shader);
    void setPose(const openvdb::Mat4s& m){
    	mModel=m;
    }
    openvdb::Mat4s& getPose(){
    	return mModel;
    }
    double getScale(){
    	return mModel(0,0)*mDistanceToObject;
    }

    CameraAndSceneConfig getConfig();
    void setConfig(const CameraAndSceneConfig& pose);
    bool saveConfig(const std::string& file=".pose_desc");
    bool loadConfig(const std::string& file=".pose_desc");
    void lookAt(const openvdb::Vec3d& p, double dist = 1.0);

    void setNearFarPlanes(double n, double f) { mNearPlane = n; mFarPlane = f; }
    void setFieldOfView(double degrees) { mFov = degrees; }
    void setSpeed(double zoomSpeed, double strafeSpeed, double tumblingSpeed);

    void keyCallback(GLFWwindow* win,int key, int action);
    void mouseButtonCallback(int button, int action);
    void mousePosCallback(int x, int y);
    void mouseWheelCallback(double pos);
    void setDistance(double distance){
    	mDistanceToObject=distance;
    	mChanged = true;
    	mNeedsDisplay=true;
    }
    void setLookAt(double x,double y, double z){
        	mLookAt[0]=x;
        	mLookAt[1]=y;
        	mLookAt[2]=z;
        	mChanged = true;
        	mNeedsDisplay=true;
        }
    float nearPlane(){return mNearPlane;}
    float farPlane(){return mFarPlane;}
    bool needsDisplay() const { return mNeedsDisplay; }
    openvdb::Vec3s transform(openvdb::Vec3s& pt){
    	openvdb::Vec4s ptp(pt[0],pt[1],pt[2],1.0f);
    	openvdb::Vec4s p=mProjection*mView*mModel*ptp;
    	return openvdb::Vec3s(p[0]/p[3],p[1]/p[3],p[2]/p[3]);
    }
    void resetTranslation(){
    	mCameraTrans=openvdb::Vec3d(0,0,0);
    	mLookAt=openvdb::Vec3d(0,0,0);
    }


    void setZoom(double z){
    	mDistanceToObject=z;
    }
    static const double sDeg2rad;
}; // class Camera

} // namespace openvdb_viewer

#endif // OPENVDB_VIEWER_CAMERA_HAS_BEEN_INCLUDED

// Copyright (c) 2012-2013 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
