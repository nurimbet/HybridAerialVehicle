#include "mywindow.h"
#include <fstream>
#include <iostream>

MyWindow::MyWindow(const ds::WorldPtr& world) 
{ 
    setWorld(world); 
    //mZoom = 0.00040;
    mTranslate = true;
    double xw,yw,zw,zoomw;
    std::ifstream finw("window.txt");
    finw >> xw >> yw >> zw >> zoomw;
    mZoom = zoomw;
    mTrans = -Eigen::Vector3d(xw, yw, zw)*1000.0;//-Eigen::Vector3d(20,20,20);
    // TODO add dart version detection here
}

void MyWindow::drawSkels() {
    double ign, ptlr, ptlg, ptlb, cr, cg, cb, pwidth, treer, treeg, treeb, path_bool;
    std::ifstream finw("window.txt");
    finw >> ign >> ign >> ign >> ign >> pwidth >> ptlr >> ptlg >> ptlb >> cr >> cg >> cb >> treer >> treeg >> treeb >> path_bool;
    //std::cout << xw << " "<< yw << " " << zw << " " <<  zoomw;

    //mZoom = 0.00075;
    //mTrans = -Eigen::Vector3d(1500,2600,200)*1000.0;//-Eigen::Vector3d(20,20,20);
    std::lock_guard<std::mutex> lock(readMutex);
    // Make sure lighting is turned on and that polygons get filled in
    glPolygonMode(GL_FRONT, GL_FILL);
    //glDisable(GL_LIGHT0);
    //glShadeModel(GL_SMOOTH);

    glBegin(GL_LINES);

    float x, y, z;
    float angz, angx;
    float x1, y1, z1;
    //glLineWidth(1); 
    if (path_bool == 0 || path_bool > 1)
    {
    glColor3f(treer, treeg, treeb);
    std::ifstream edges("edges_fx.txt");
    while(!edges.eof()){
        edges >> x >> y >> z >> ign >> ign >> ign >> ign >> ign >>
            x1 >> y1 >> z1 >> ign >> ign >> ign >> ign >> ign;  
        //std::cout << x << std::endl;
        glVertex3f(x, y, z);
        glVertex3f(x1, y1, z1);

    }
    glEnd();
    }
    if (path_bool >= 1)
    { 

    glLineWidth(4); 
    glBegin(GL_LINES);
    glColor3f(ptlr, ptlg, ptlb);
    std::ifstream fin_to("take_off.txt");
    fin_to >> x >> y >> z >> ign >> angz >> ign >>
        angx >> ign >> ign >> ign >> ign >> ign;  
    glVertex3f(x, y, z+10);
    while(!fin_to.eof()){
        fin_to >> x >> y >> z >> ign >> angz >> ign >>
            angx >> ign >> ign >> ign >> ign >> ign;  

        glVertex3f(x, y, z+10);
        glVertex3f(x, y, z+10);

    }

    glColor3f(cr, cg, cb);
    std::ifstream fin_c("cruise.txt");
    fin_c >> x >> y >> z >> ign >> angz >> ign >>
        angx >> ign >> ign >> ign >> ign >> ign;  
    while(!fin_c.eof()){
        fin_c >> x >> y >> z >> ign >> angz >> ign >>
            angx >> ign >> ign >> ign >> ign >> ign;  

        glVertex3f(x, y, z+10);
        glVertex3f(x, y, z+10);

    }

    glColor3f(ptlr, ptlg, ptlb);
    std::ifstream fin_l("landing.txt");
    fin_l >> x >> y >> z >> ign >> angz >> ign >>
        angx >> ign >> ign >> ign >> ign >> ign;  
    while(!fin_l.eof()){
        fin_l >> x >> y >> z >> ign >> angz >> ign >>
            angx >> ign >> ign >> ign >> ign >> ign;  

        glVertex3f(x, y, z+10);
        glVertex3f(x, y, z+10);

    }
    glVertex3f(x, y, z+10);


    glEnd();
    }

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
