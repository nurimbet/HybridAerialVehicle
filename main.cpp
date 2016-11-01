#include <dart/dart.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <Eigen/Eigen>

#include "config.h"
#include "mywindow.h"
#include "util.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;
namespace dc = dart::collision;

const double kMaxWidth = 10;
const double kMaxLength = 10;
const double kMaxHeight = 10;

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
            ss_->solve();

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

            //
            // ADD CODE HERE
            //
        }

        void setWorld(const ds::WorldPtr &world) { world_ = world; }

    private:
        bool isStateValid(const ob::State *state) const {

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

SkeletonPtr loadChicago()
{

    ///return chicago;
}

int main(int argc, char *argv[]) {
    ds::WorldPtr world = std::make_shared<ds::World>();
//    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

//    dd::SkeletonPtr chicago (SdfParser::readSkeleton(dc::Uri::createFromString("/home/nurimbet/Downloads/Chicago.sdf")));
    dd::SkeletonPtr ball1 = dd::Skeleton::create("ball1");
/*
    Eigen::Isometry3f tf(Eigen::Isometry3f::Identity());
    tf.translation() = Eigen::Vector3f(0, 0, 0);

    createBall(ball1, Eigen::Vector3f(0.1, 0.1, 0.1),tf) ;
*/
   Eigen::Isometry3d tf(Eigen::Isometry3d::Identity(), Eigen::DontAlign);
    tf.translation() = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d size(0.1,0.1,0.1);
    
    createBall(ball1, size,tf) ; 
    //
    // ADD CODE HERE
    // or use OOP if you want
    //

    //std::cout << "collision detected " << world->checkCollision() << std::endl;
  //  world->addSkeleton(chicago);
        world->addSkeleton(ball1);
    MyWindow window(world);
    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "SDF");
    window.refreshTimer(5);
    glutMainLoop();
}
