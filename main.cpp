#include <dart/dart.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
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
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
                    1.0 / space->getMaximumExtent());
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
            uavball->getJoint(0)->setTransformFromParentBodyNode(tf);

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
    //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

    dd::SkeletonPtr chicago =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/Chicago.sdf"));
    dd::SkeletonPtr ball1 = dd::Skeleton::create("ball1");

    /*
       Eigen::Isometry3f tf(Eigen::Isometry3f::Identity());
       tf.translation() = Eigen::Vector3f(0, 0, 0);

       createBall(ball1, Eigen::Vector3f(0.1, 0.1, 0.1),tf) ;
     */
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d size(10,10,10);

    createBall(ball1, size,tf) ; 

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

    Eigen::Vector3d start(0.0,0.0,50.0);
    Eigen::Vector3d finish(2000,2000,20);

    if(env.plan(start,finish))
    {
        env.recordSolution();
    }


    tf=Eigen::Isometry3d::Identity();
    tf.translation() = start;
    ball1->getJoint(0)->setTransformFromParentBodyNode(tf);
    MyWindow window(world);

    std::thread t([&](){
            std::this_thread::sleep_for(std::chrono::seconds(5));

            while(true){
            std::ifstream fin("result.txt");
            Eigen::Isometry3d tf=Eigen::Isometry3d::Identity();
            while(!fin.eof())
            {
            double x,y,z;
            fin >> x >> y >> z;
            tf.translation() = Eigen::Vector3d(x,y,z);
            window.setViewTrack(Eigen::Vector3d(x,y,z));
            ball1->getJoint(0)->setTransformFromParentBodyNode(tf);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));			
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