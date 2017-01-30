#include "mywindow.h"
#include <fstream>
#include <iostream>

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    //mZoom = 0.00040;
    mZoom = 0.00075;
    mTranslate = true;
    // TODO add dart version detection here
}

void MyWindow::drawSkels() {
    mTrans = -Eigen::Vector3d(1500,2600,200)*1000.0;//-Eigen::Vector3d(20,20,20);
    std::lock_guard<std::mutex> lock(readMutex);
    // Make sure lighting is turned on and that polygons get filled in
    glEnable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glLineWidth(3); 
    glColor3f(0.4, 0.8, 0.3);
    glBegin(GL_LINES);

    std::ifstream fin("result.txt");
    float x, y, z, ign;
    float angz, angx;
    fin >> x >> y >> z >> ign >> angz >> ign >>
        angx >> ign >> ign >> ign >> ign >> ign;  
    glVertex3f(x, y, z);
    while(!fin.eof()){
        fin >> x >> y >> z >> ign >> angz >> ign >>
            angx >> ign >> ign >> ign >> ign >> ign;  

        glVertex3f(x, y, z);
        glVertex3f(x, y, z);

    }
    glEnd();
/*  
    glLineWidth(2); 
    glColor3f(0.33, 0.66, 0.99);
    glBegin(GL_LINES);

    int x1, y1;
    int xmax = 3000;
    int ymax = 4000;

    for (int ii = 0; ii <= xmax; ii+=10) 
    {
        glVertex3f(ii, 0, 10);
        glVertex3f(ii, ymax, 10);
    }
    for (int ii = 0; ii <= ymax; ii+=10) 
    {
        glVertex3f(0, ii, 10);
        glVertex3f(xmax, ii, 10);
    }
        //float rx, ry, rz, rw;


    glEnd();
*/

    SimWindow::drawSkels();
}

void MyWindow::setViewTrack(const Eigen::Vector3d& v, const Eigen::Quaterniond& rot)
{
    std::lock_guard<std::mutex> lock(readMutex);
    //mTrans = -v*1000.0;//-Eigen::Vector3d(20,20,20);
    Eigen::Quaterniond quat(Eigen::AngleAxisd(-3*M_PI/8.0, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ())*rot);

    //    mTrackBall.setQuaternion(quat);
}
