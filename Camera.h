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
namespace imagesci{

class Camera
{
public:
    Camera();


    void aim(GLFWwindow* win);
    void aim(int x,int y,int width,int height);
    void beginShader();
    inline void endShader(){
    	mShader.end();
    }
    void setPose(const openvdb::Mat4s& m){
    	M=m;
    }
    void init();
    void lookAt(const openvdb::Vec3d& p, double dist = 1.0);
    void lookAtTarget();

    void setTarget(const openvdb::Vec3d& p, double dist = 1.0);

    void setNearFarPlanes(double n, double f) { mNearPlane = n; mFarPlane = f; }
    void setFieldOfView(double degrees) { mFov = degrees; }
    void setSpeed(double zoomSpeed, double strafeSpeed, double tumblingSpeed);

    void keyCallback(GLFWwindow* win,int key, int action);
    void mouseButtonCallback(int button, int action);
    void mousePosCallback(int x, int y);
    void mouseWheelCallback(double pos);
    float nearPlane(){return mNearPlane;}
    float farPlane(){return mFarPlane;}
    bool needsDisplay() const { return mNeedsDisplay; }
    openvdb::Vec3s transform(openvdb::Vec3s& pt){
    	openvdb::Vec4s ptp(pt[0],pt[1],pt[2],1.0f);
    	openvdb::Vec4s p=P*V*M*ptp;
    	return openvdb::Vec3s(p[0]/p[3],p[1]/p[3],p[2]/p[3]);
    }
protected:
    GLShader mShader;
    // Camera parameters
    double mFov, mNearPlane, mFarPlane;
    openvdb::Vec3d mTarget, mLookAt, mUp, mForward, mRight, mEye;
    double mTumblingSpeed, mZoomSpeed, mStrafeSpeed;
    double mHead, mPitch, mTargetDistance, mDistance;
    openvdb::Mat4s P,V,M;
    // Input states
    bool mMouseDown, mStartTumbling, mZoomMode, mChanged, mNeedsDisplay;
    double mMouseXPos, mMouseYPos;
    int mWheelPos;

    static const double sDeg2rad;
}; // class Camera

} // namespace openvdb_viewer

#endif // OPENVDB_VIEWER_CAMERA_HAS_BEEN_INCLUDED

// Copyright (c) 2012-2013 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
