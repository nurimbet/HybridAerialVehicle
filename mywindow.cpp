#include "mywindow.h"

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.010;
    mTranslate = true;
}

void MyWindow::drawSkels() {
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    Eigen::Matrix3d rot;
    rot = Eigen::AngleAxisd(-M_PI/4.0, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ());//*Eigen::AngleAxisd(M_PI/8.0, Eigen::Vector3d::UnitY());
    Eigen::Quaterniond quat(rot);    

    mTrackBall.setQuaternion(quat);
    //mEye = viewTrack+ Eigen::Vector3d(10, 10, 10);
    //mUp = viewTrack - Eigen::Vector3d(10, 10, 10);
    SimWindow::drawSkels();
   /* 
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        mWorld->getSkeleton(i)->draw(mRI);
    }*/

}

void MyWindow::setViewTrack(const Eigen::Vector3d& v)
{
    mTrans = -v*1000.0-Eigen::Vector3d(20,20,20);
}
