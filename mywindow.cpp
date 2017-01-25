#include "mywindow.h"
#include <fstream>
#include <iostream>

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    mZoom = 0.00040;
    mTranslate = true;
// TODO add dart version detection here
//    mZnear = 0.01;
//    mZfar = 1000.0;
//    mPersp = 45.0;
}

void MyWindow::drawSkels() {
    mTrans = -Eigen::Vector3d(1000,1000,500)*1000.0;//-Eigen::Vector3d(20,20,20);
    std::lock_guard<std::mutex> lock(readMutex);
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
   
    glLineWidth(3); 
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);

    std::ifstream fin("result.txt");
    float x, y, z, ign;
    float angz, angx;
    fin >> x >> y >> z >> ign >> angz >> ign >>
    angx >> ign >> ign >> ign >> ign >> ign;  // >> ign >> ign >> ign;
    glVertex3f(x, y, z);
    while(!fin.eof()){
    //float rx, ry, rz, rw;
    fin >> x >> y >> z >> ign >> angz >> ign >>
    angx >> ign >> ign >> ign >> ign >> ign;  // >> ign >> ign >> ign;

    glVertex3f(x, y, z);
    glVertex3f(x, y, z);

    }
    glEnd();
    
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
}

void MyWindow::setViewTrack(const Eigen::Vector3d& v, const Eigen::Quaterniond& rot)
{
    std::lock_guard<std::mutex> lock(readMutex);
    //mTrans = -v*1000.0;//-Eigen::Vector3d(20,20,20);
    //mTrans = -Eigen::Vector3d(1000,1000,500)*1000.0;//-Eigen::Vector3d(20,20,20);
    //mTrans = -Eigen::Vector3d(25,25,25)*1000;
    Eigen::Quaterniond quat(Eigen::AngleAxisd(-3*M_PI/8.0, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())*rot);

//    mTrackBall.setQuaternion(quat);
}
