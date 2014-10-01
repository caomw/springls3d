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

#include "Camera.h"

#include "ImageSciUtil.h"
#include <cmath>
#include <string>
#include <list>
#include <fstream>

namespace imagesci {

const double Camera::sDeg2rad = M_PI / 180.0;


Camera::Camera()
    : mFov(65.0)
	,mRw(openvdb::Mat4s::identity())
	,mRm(openvdb::Mat4s::identity())
	,mCameraTrans(0,0,0)
    , mNearPlane(0.1)
    , mFarPlane(10000.0)
    , mEye(openvdb::Vec3d(0.0, 0.0, -1.0))
    , mTumblingSpeed(0.5)
    , mZoomSpeed(0.2)
    , mStrafeSpeed(0.05)
    , mDistance(1.0)
    , mMouseDown(false)
    , mStartTumbling(false)
    , mZoomMode(false)
    , mChanged(true)
    , mNeedsDisplay(true)
    , mMouseXPos(0.0)
    , mMouseYPos(0.0)
    , mWheelPos(0)
	, P(openvdb::Mat4s::identity())
	, V(openvdb::Mat4s::identity())
	, M(openvdb::Mat4s::identity())
{
}


void
Camera::lookAt(const openvdb::Vec3d& p, double dist)
{
    mLookAt = p;
    mDistance = dist;
    mChanged=true;
    mNeedsDisplay = true;
}



void
Camera::setSpeed(double zoomSpeed, double strafeSpeed, double tumblingSpeed)
{
    mZoomSpeed = std::max(0.0001, zoomSpeed);
    mStrafeSpeed = std::max(0.0001, strafeSpeed);
    mTumblingSpeed = std::max(0.01, tumblingSpeed);
}



openvdb::Mat4s perspectiveMatrix(const float &fovy, const float &aspect, const float &zNear, const float &zFar)
{
    float f = 1.0f/tanf(M_PI*fovy / 360.0f);
    float sx = f/aspect;
    float sy = f;
    float sz = -(zFar + zNear) / (zFar - zNear);
    float pz = -(2.0f * zFar * zNear) / (zFar - zNear);
    openvdb::Mat4s Mat=openvdb::Mat4s::zero();
    float* M=Mat.asPointer();
    M[0] = sx;
    M[5] = sy;
    M[10] = sz;
    M[14] = pz;
    M[11] = -1.0f;

    return M;
}

openvdb::Mat4s lookAtMatrix(openvdb::Vec3s eyePosition3D,openvdb::Vec3s center3D, openvdb::Vec3s upVector3D ){
   openvdb::Vec3s forward, side, up;
   openvdb::Mat4s matrix2Mat;
   float* matrix2=matrix2Mat.asPointer();
   openvdb::Mat4s resultMatrix;
   //------------------
   forward=center3D-eyePosition3D;
   forward.normalize();
   //------------------
   //Side = forward x up
   side=forward.cross(upVector3D);
   side.normalize();
   //------------------
   up=side.cross(forward);
   //------------------
   matrix2[0] = side[0];
   matrix2[4] = side[1];
   matrix2[8] = side[2];
   matrix2[12] = 0.0;
   //------------------
   matrix2[1] = up[0];
   matrix2[5] = up[1];
   matrix2[9] = up[2];
   matrix2[13] = 0.0;
   //------------------
   matrix2[2] = -forward[0];
   matrix2[6] = -forward[1];
   matrix2[10] = -forward[2];
   matrix2[14] = 0.0;
   //------------------
   matrix2[3] = matrix2[7] = matrix2[11] = 0.0;
   matrix2[15] = 1.0;

   resultMatrix=matrix2;

   openvdb::Mat4s T=openvdb::Mat4s::identity();


   //------------------
   T(0,3)=-eyePosition3D[0];
   T(1,3)=-eyePosition3D[1];
   T(2,3)=-eyePosition3D[2];


   return resultMatrix.transpose()*T;
}
void Camera::aim(int x,int y,int width,int height,GLShader& shader){



    // Set up the projection matrix
   // glMatrixMode(GL_PROJECTION);
    // Window aspect (assumes square pixels)
    double aspectRatio = (double)width / (double)height;
    // Set perspective view (fov is in degrees in the y direction.)
   // gluPerspective(mFov, aspectRatio, mNearPlane, mFarPlane);

    if (mChanged) {
        mChanged = false;
        openvdb::Mat4s Tinv=openvdb::math::Mat4s::identity();
        openvdb::Mat4s Teye=openvdb::math::Mat4s::identity();
        openvdb::Mat4s S=openvdb::math::Mat4s::identity();
        Tinv(0,3)=-mLookAt[0];
        Tinv(1,3)=-mLookAt[1];
        Tinv(2,3)=-mLookAt[2];

        openvdb::Mat4s T=openvdb::math::Mat4s::identity();
        T(0,3)=mLookAt[0];
        T(1,3)=mLookAt[1];
        T(2,3)=mLookAt[2];
        Teye(0,3)=mEye[0];
        Teye(1,3)=mEye[1];
        Teye(2,3)=mEye[2];

        S(0,0)=mDistance;
        S(1,1)=mDistance;
        S(2,2)=mDistance;



        openvdb::Mat4s Tcamera=openvdb::math::Mat4s::identity();
        Tcamera(0,3)=mCameraTrans[0];
        Tcamera(1,3)=mCameraTrans[1];
        Tcamera(2,3)=mCameraTrans[2];

        P=Tcamera*perspectiveMatrix(mFov,aspectRatio,mNearPlane,mFarPlane).transpose();
        V=Teye*S*mRw*T*mRm;
    }
    /*
    std::cout<<"P=\n"<<P<<std::endl;
    std::cout<<"V=\n"<<V<<std::endl;
    std::cout<<"M=\n"<<M<<std::endl;
    */
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "P"), 1,GL_TRUE, P.asPointer());
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "V"), 1,GL_TRUE, V.asPointer());
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "M"), 1,GL_TRUE, M.asPointer());
    mNeedsDisplay = false;
}

void Camera::keyCallback(GLFWwindow* win,int key, int action)
{
	if((char)key=='A'){
		mRm=openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::Y_AXIS,2*sDeg2rad)*mRm;
		mChanged=true;
	} else if((char)key=='D'){
		mRm=openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::Y_AXIS,-2*sDeg2rad)*mRm;
		mChanged=true;
	} else if((char)key=='S'){
		mRm=openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::X_AXIS,-2*sDeg2rad)*mRm;
		mChanged=true;
	} else if((char)key=='W'){
		mRm=openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::X_AXIS,2*sDeg2rad)*mRm;
		mChanged=true;
	} else if(key==GLFW_KEY_UP){
		mCameraTrans[1]+=0.025;
		mChanged=true;
	} else if(key==GLFW_KEY_DOWN){
		mCameraTrans[1]-=0.025;
		mChanged=true;
	} else if(key==GLFW_KEY_LEFT){
		mCameraTrans[0]-=0.025;
		mChanged=true;
	} else if(key==GLFW_KEY_RIGHT){
		mCameraTrans[0]+=0.025;
		mChanged=true;
	} else if(key==GLFW_KEY_PAGE_UP){
		mDistance =(1+mZoomSpeed)*mDistance;
		mChanged=true;
	} else if(key==GLFW_KEY_PAGE_DOWN){
		mDistance =(1-mZoomSpeed)*mDistance;
		mChanged=true;
	} else{
		if (glfwGetKey(win,key) == GLFW_PRESS) {
			switch(key) {
				case GLFW_KEY_SPACE:
					mZoomMode = true;
					break;
			}
		} else if (glfwGetKey(win,key) == GLFW_RELEASE) {
			switch(key) {
				case GLFW_KEY_SPACE:
					mZoomMode = false;
					break;
			}
		}
	}
    mChanged = true;
}


void
Camera::mouseButtonCallback(int button, int action)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) mMouseDown = true;
        else if (action == GLFW_RELEASE) mMouseDown = false;
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            mMouseDown = true;
            mZoomMode = true;
        } else if (action == GLFW_RELEASE) {
            mMouseDown = false;
            mZoomMode = false;
        }
    }
    if (action == GLFW_RELEASE) mMouseDown = false;

    mStartTumbling = true;
    mChanged = true;
}


void
Camera::mousePosCallback(int x, int y)
{
    if (mStartTumbling) {
        mMouseXPos = x;
        mMouseYPos = y;
        mStartTumbling = false;
    }

    double dx, dy;
    dx = x - mMouseXPos;
    dy = y - mMouseYPos;

    if (mMouseDown && !mZoomMode) {
        mNeedsDisplay = true;
		mRw=
				openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::Y_AXIS,-dx*mTumblingSpeed*sDeg2rad)*
				openvdb::math::rotation<openvdb::Mat4s>(openvdb::math::Axis::X_AXIS,-dy*mTumblingSpeed*sDeg2rad)*mRw;
    } else if (mMouseDown && mZoomMode) {
        mNeedsDisplay = true;
        openvdb::Vec3d mUp=mRw.row(1).getVec3();
        openvdb::Vec3d mRight=mRw.row(0).getVec3();
        mLookAt +=(mRight*dx-dy*mUp) * mStrafeSpeed;

    }
    mMouseXPos = x;
    mMouseYPos = y;

    mChanged = true;
}

bool Camera::savePose(const std::string& file){
	if(!mChanged&&!mNeedsDisplay)return false;
	FILE* f = fopen(file.c_str(), "wb");
	if(f!=NULL){
		CameraPose p;
		p.mRModel=mRm;
		p.mRWorld=mRw;
		p.mTModel=mCameraTrans;
		p.mTWorld=mLookAt;
		p.mDistance=mDistance;
		fwrite(&p, sizeof(CameraPose), 1, f);
		fclose(f);
	return true;
	} else return false;
}
bool Camera::loadPose(const std::string& file){
	FILE* f = fopen(file.c_str(), "rb");
	if(f!=NULL){
		CameraPose p;
		fread(&p,sizeof(CameraPose),1,f);
		mRm=p.mRModel;
		mRw=p.mRWorld;
		mCameraTrans=p.mTModel;
		mLookAt=p.mTWorld;
		mDistance=p.mDistance;
		fclose(f);
		mChanged=true;
		mNeedsDisplay=true;
	return true;
	} else return false;
}
void
Camera::mouseWheelCallback(double pos)
{
    mDistance =(1-pos*mZoomSpeed)*mDistance;
    mChanged = true;
    mNeedsDisplay = true;
}

} // namespace openvdb_viewer

// Copyright (c) 2012-2013 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
