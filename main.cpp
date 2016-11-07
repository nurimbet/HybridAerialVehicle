#include <dart/dart.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <Eigen/Eigen> 
#include <iostream> 
#include <fstream> 
#include <thread> 
#include <chrono>

#include "config.h"
#include "mywindow.h"
#include "util.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;
namespace du = dart::utils;

constexpr double kMaxWidth = 3000;
constexpr double kMaxLength = 4000;
constexpr double kMaxHeight = 400;

#define DIM 3

class Simple3DEnvironment {
    public:
        Simple3DEnvironment() {


            ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();

            space->addDimension(0.0, kMaxWidth);
            space->addDimension(0.0, kMaxLength);
            space->addDimension(0.0, kMaxHeight);

            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&Simple3DEnvironment::isStateValid,
                        this, std::placeholders::_1));
            space->setup();
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(
            //        1.0 / space->getMaximumExtent());
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
                    1e-4);

            ss_->setPlanner(ob::PlannerPtr(new og::PRM(ss_->getSpaceInformation())));

        }

        bool plan(const Eigen::Vector3d &init, const Eigen::Vector3d & final) {
            if (!ss_) return false;

            ob::ScopedState<> start(ss_->getStateSpace());
            for (std::size_t i = 0; i < DIM; ++i) start[i] = init[i];

            ob::ScopedState<> goal(ss_->getStateSpace());
            for (std::size_t i = 0; i < DIM; ++i) goal[i] = final[i];

            ss_->setStartAndGoalStates(start, goal);

            // this will run the algorithm for one second
            ss_->solve(100);

            // ss_->solve(1000); // it will run for 1000 seconds

            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                ss_->simplifySolution();
                og::PathGeometric &p = ss_->getSolutionPath();
                ss_->getPathSimplifier()->simplifyMax(p);
                ss_->getPathSimplifier()->smoothBSpline(p);
                return true;
            } else
                return false;
        }

        void recordSolution() {
            if (!ss_ || !ss_->haveSolutionPath()) return;
            og::PathGeometric &p = ss_->getSolutionPath();
            p.interpolate();
            std::ofstream resultfile;
            resultfile.open("result.txt", std::ios::trunc);
            for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
            {
                const double x = std::min(kMaxWidth, (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0]);
                const double y = std::min(kMaxLength, (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1]);
                const double z = std::min(kMaxHeight, (double)p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[2]);
                resultfile << x << " " << y << " " << z << std::endl; 
            }
            resultfile.close();

            //return true;  // stub
            //
            // ADD CODE HERE
            //
        }

        void setWorld(const ds::WorldPtr &world) { world_ = world; }

    private:
        bool isStateValid(const ob::State *state) const {

            double x = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]; 
            double y = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]; 
            double z = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]; 

            Eigen::Isometry3d tf;
            tf = Eigen::Isometry3d::Identity();
            tf.translation() = Eigen::Vector3d(x,y,z);

            dd::SkeletonPtr uavball = world_->getSkeleton("ball1");
            //uavball->getJoint()->setTransformFromParentBodyNode(tf);
            moveSkeleton(uavball, tf);

            //
            // ADD CODE HERE
            //
            //

            //return true;  // stub
            return !world_->checkCollision();
        }

        og::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
};


int main(int argc, char *argv[]) {
    ds::WorldPtr world = std::make_shared<ds::World>();
    world->getConstraintSolver()->setCollisionDetector(
            new dc::BulletCollisionDetector());
    //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

    //dd::SkeletonPtr chicago =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/Chicago.sdf"));
    dd::SkeletonPtr chicago =du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/Chicago.sdf"));
    dd::SkeletonPtr ball1 = dd::Skeleton::create("ball1");
    //dd::SkeletonPtr ball1 =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/uav.sdf"));

    //du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

    //dd::SkeletonPtr ball1 = dd::Skeleton::create("ball1");

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf.translation() = Eigen::Vector3d(0, 0, 0);
    createBall(ball1, Eigen::Vector3d(6, 6, 6), tf) ;
    world->addSkeleton(ball1);
    ball1 = du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

    //    ball1->getJoint(0)->setTransformFromParentBodyNode(tf);
    //moveSkeleton(ball1, tf);

    //createBox(ball1, size,tf) ; 

    setAllColors(ball1, Eigen::Vector3d(1,0.2,0.2));
    //
    // ADD CODE HERE
    // or use OOP if you want
    //

    //std::cout << "collision detected " << world->checkCollision() << std::endl;
    world->addSkeleton(chicago);
    world->addSkeleton(ball1);

    Simple3DEnvironment env;
    env.setWorld(world);

    Eigen::Vector3d start(0.0,0.0,15.0);
    Eigen::Vector3d finish(2000,3000,70);

    if(env.plan(start,finish))
    {
        env.recordSolution();
    }


    tf=Eigen::Isometry3d::Identity();
    // tf.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitY()));
    tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    tf.translation() = start;
    moveSkeleton(ball1, tf);

    MyWindow window(world);

    double oldx, oldy, oldz,angleRot,angleRotOld = 0.0;
    std::thread t([&]()
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        while(true)
        {
            std::ifstream fin("result.txt");
            
           
            while(!fin.eof())
            {

                double x,y,z;
                fin >> x >> y >> z;
                
                oldx = x - oldx;
                oldy = y - oldy;
                angleRot = -atan2(oldx, oldy);
                //std::cout << angleRot <<std::endl; 
                Eigen::Isometry3d tf=Eigen::Isometry3d::Identity();
                tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
                
                if (abs(angleRot - angleRotOld) < M_PI/9) {
                    angleRot = angleRotOld;
                }
                tf.rotate(Eigen::AngleAxisd(angleRot, Eigen::Vector3d::UnitY())); 
                tf.translation() = Eigen::Vector3d(x,y,z);
               
                moveSkeleton(ball1, tf);
                    
                window.setViewTrack(Eigen::Vector3d(x,y,z), Eigen::AngleAxisd(-angleRot, Eigen::Vector3d::UnitZ()));
                std::this_thread::sleep_for(std::chrono::milliseconds(10));			
                
                oldx = x; oldy = y; oldz = z; 
                angleRotOld = angleRot;
            }
        }
    });


    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "SDF");
    window.refreshTimer(5);
    glutMainLoop();

    t.join();

    return EXIT_SUCCESS;
}
