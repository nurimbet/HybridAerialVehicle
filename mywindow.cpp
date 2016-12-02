#include "mywindow.h"

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.040;
    mTranslate = true;
//    mPersp = 45.0;
}

void MyWindow::drawSkels() {
    std::lock_guard<std::mutex> lock(readMutex);
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    //glFrustum(-10, 10, -10, 10, 15, 200);
    //Eigen::Matrix3d rot;
    //rot = Eigen::AngleAxisd(-M_PI/4.0, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitZ());//*Eigen::AngleAxisd(-M_PI/4.0, Eigen::Vector3d::UnitX());
    //Eigen::Quaterniond quat(rot);    

    //mTrackBall.setQuaternion(quat);
    //mEye = viewTrack+ Eigen::Vector3d(10, 10, 10);
    //mUp = viewTrack - Eigen::Vector3d(10, 10, 10);
//    gluPerspective(mPersp,
//                 static_cast<double>(mWinWidth)/static_cast<double>(mWinHeight),
//                 0.1, 100.0);
    SimWindow::drawSkels();
   /* 
    for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
        mWorld->getSkeleton(i)->draw(mRI);
    }*/

}

void MyWindow::setViewTrack(const Eigen::Vector3d& v, const Eigen::AngleAxisd& rot)
{
    std::lock_guard<std::mutex> lock(readMutex);
    mTrans = -v*1000.0;//-Eigen::Vector3d(20,20,20);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(-3*M_PI/8.0, Eigen::Vector3d::UnitX())*rot);    

//    mTrackBall.setQuaternion(quat);
}
