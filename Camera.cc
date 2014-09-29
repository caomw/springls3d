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
namespace imagesci {

const double Camera::sDeg2rad = M_PI / 180.0;


Camera::Camera()
    : mFov(65.0)
    , mNearPlane(0.1)
    , mFarPlane(10000.0)
    , mTarget(openvdb::Vec3d(0.0))
    , mLookAt(mTarget)
    , mUp(openvdb::Vec3d(0.0, 1.0, 0.0))
    , mForward(openvdb::Vec3d(0.0, 0.0, 1.0))
    , mRight(openvdb::Vec3d(1.0, 0.0, 0.0))
    , mEye(openvdb::Vec3d(0.0, 0.0, -1.0))
    , mTumblingSpeed(0.5)
    , mZoomSpeed(0.2)
    , mStrafeSpeed(0.05)
    , mHead(30.0)
    , mPitch(45.0)
    , mTargetDistance(25.0)
    , mDistance(mTargetDistance)
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
    mNeedsDisplay = true;
}


void
Camera::lookAtTarget()
{
    mLookAt = mTarget;
    mDistance = mTargetDistance;
    mNeedsDisplay = true;
}


void
Camera::setSpeed(double zoomSpeed, double strafeSpeed, double tumblingSpeed)
{
    mZoomSpeed = std::max(0.0001, zoomSpeed);
    mStrafeSpeed = std::max(0.0001, strafeSpeed);
    mTumblingSpeed = std::max(0.2, tumblingSpeed);
    mTumblingSpeed = std::min(1.0, tumblingSpeed);
}


void
Camera::setTarget(const openvdb::Vec3d& p, double dist)
{
    mTarget = p;
    mTargetDistance = dist;
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

        mEye[0] = mLookAt[0] + mDistance * std::cos(mHead * sDeg2rad) * std::cos(mPitch * sDeg2rad);
        mEye[1] = mLookAt[1] + mDistance * std::sin(mHead * sDeg2rad);
        mEye[2] = mLookAt[2] + mDistance * std::cos(mHead * sDeg2rad) * std::sin(mPitch * sDeg2rad);

        mForward = mLookAt - mEye;
        mForward.normalize();

        mUp[1] = std::cos(mHead * sDeg2rad) > 0 ? 1.0 : -1.0;
        mRight = mForward.cross(mUp);
    }
    P=perspectiveMatrix(mFov,aspectRatio,mNearPlane,mFarPlane).transpose();
    V=lookAtMatrix(mEye,mLookAt,mUp);
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "P"), 1,GL_TRUE, P.asPointer());
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "V"), 1,GL_TRUE, V.asPointer());
    glUniformMatrix4fv(glGetUniformLocation(shader.GetProgramHandle(), "M"), 1,GL_TRUE, M.asPointer());
    mNeedsDisplay = false;
}

void Camera::keyCallback(GLFWwindow* win,int key, int )
{
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
        mHead += dy * mTumblingSpeed;
        mPitch += dx * mTumblingSpeed;
    } else if (mMouseDown && mZoomMode) {
        mNeedsDisplay = true;
        mLookAt += (dy * mUp - dx * mRight) * mStrafeSpeed;
    }

    mMouseXPos = x;
    mMouseYPos = y;
    mChanged = true;
}


void
Camera::mouseWheelCallback(double pos)
{
    mDistance += pos * mZoomSpeed;
     setSpeed(mDistance * 0.1, mDistance * 0.002, mDistance * 0.02);
/*
	double speed = std::abs(prevPos - pos);
    std::cout<<pos<<" "<<prevPos<<" "<<mDistance<<std::endl;
    if (prevPos < pos) {
        mDistance += speed * mZoomSpeed;
        setSpeed(mDistance * 0.1, mDistance * 0.002, mDistance * 0.02);
    } else {
        double temp = mDistance - speed * mZoomSpeed;
        mDistance = std::max(0.0, temp);
        setSpeed(mDistance * 0.1, mDistance * 0.002, mDistance * 0.02);
    }
*/
    mChanged = true;
    mNeedsDisplay = true;
}

} // namespace openvdb_viewer

// Copyright (c) 2012-2013 DreamWorks Animation LLC
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
