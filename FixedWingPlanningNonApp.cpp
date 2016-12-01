#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <iostream>
#include <valarray>
#include <limits>
#include <Eigen/Eigen> 
#include <iostream> 
#include <fstream> 
#include <thread> 
#include <chrono>
#include "config.h"
#include "mywindow.h"
#include "util.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;
namespace du = dart::utils;
ob::StateSpacePtr FWspace;
constexpr double kMaxWidth = 3000;
constexpr double kMaxLength = 4000;
constexpr double kMaxHeight = 400;


class FWControlSpace : public oc::RealVectorControlSpace
{
    public:

        FWControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 3)
    {
    }
};


class FixedWingEnvironment{
    public:
        FixedWingEnvironment() {

            FWspace =constructFWStateSpace();

            oc::ControlSpacePtr cspace(new FWControlSpace(FWspace));

            ob::RealVectorBounds velbounds(1), omegabounds(1),anglebounds(1), controlbounds(3);

            velbounds.setLow(10);
            velbounds.setHigh(30);
            FWspace->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->setBounds(velbounds);
            //omegabounds.setLow(-.2);
            //omegabounds.setHigh(.2);
            //space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3)->setBounds(omegabounds);

            anglebounds.setLow(-1000);
            anglebounds.setHigh(1000);
            FWspace->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2)->setBounds(anglebounds);
            controlbounds.setLow(-0.3); // V dot
            controlbounds.setHigh(0.3);
            controlbounds.setLow(1,-0.1); // Z Dot
            controlbounds.setHigh(1,0.1);
            controlbounds.setLow(2,-.2); // PhiDot
            controlbounds.setHigh(2,.2);
            cspace->as<FWControlSpace>()->setBounds(controlbounds);



            //oc::SimpleSetup ss(cspace);
            ss_.reset(new oc::SimpleSetup(cspace));

            oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss_->getSpaceInformation(),std::bind( &FixedWingEnvironment::FixedWingODE, this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3)));
            ss_->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, std::bind(&FixedWingEnvironment::FixedWingPostIntegration, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3, std::placeholders::_4)));
            ss_->getSpaceInformation()->setPropagationStepSize(1);

            ob::RealVectorBounds bounds(3);
            bounds.setLow(0);
            bounds.setHigh(3000);
            bounds.setLow(1,0);
            bounds.setHigh(1,4000);
            bounds.setLow(2,15);
            bounds.setHigh(400);



            ss_->getStateSpace()->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0)->setBounds(bounds);
            ss_->setStateValidityChecker(std::bind(&FixedWingEnvironment::isStateValid,
            this, std::placeholders::_1));
            FWspace->setup();

            /*
               ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();

               space->addDimension(0.0, kMaxWidth);
               space->addDimension(0.0, kMaxLength);
               space->addDimension(0.0, kMaxHeight);

               ss_.reset(new oc::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&FixedWingEnvironment::isStateValid,
            this, std::placeholders::_1));
            space->setup();
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(
            //        1.0 / space->getMaximumExtent());
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
            1e-4);

             */
            ss_->setPlanner(ob::PlannerPtr(new oc::SST(ss_->getSpaceInformation())));

        }

        bool plan(const Eigen::Vector4d &init, const Eigen::Vector4d & final) {
            if (!ss_) return false;

            ob::ScopedState<ob::CompoundStateSpace> start(ss_->getStateSpace());

            start->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            start->as<ob::SE3StateSpace::StateType>(0)->setX(init[0]);
            start->as<ob::SE3StateSpace::StateType>(0)->setY(init[1]);
            start->as<ob::SE3StateSpace::StateType>(0)->setZ(init[2]);
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = init[3];

            ob::ScopedState<ob::CompoundStateSpace> goal(ss_->getStateSpace());

            goal->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            goal->as<ob::SE3StateSpace::StateType>(0)->setX(final[0]);
            goal->as<ob::SE3StateSpace::StateType>(0)->setY(final[1]);
            goal->as<ob::SE3StateSpace::StateType>(0)->setZ(final[2]);
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = final[3];

            ss_->setStartAndGoalStates(start, goal);

            // this will run the algorithm for one second
            ss_->solve(60);

            // ss_->solve(1000); // it will run for 1000 seconds

            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                oc::PathControl &p = ss_->getSolutionPath();
                p.interpolate();

                //p.printAsMatrix(std::cout);
                std::ofstream resultfile;
                resultfile.open("result.txt", std::ios::trunc);
                //ss_->getSolutionPath().asGeometric().printAsMatrix(resultfile);
                p.printAsMatrix(resultfile);
                return true;
            } else
                return false;

        }

        void recordSolution() {
            if (!ss_ || !ss_->haveSolutionPath()) return;
            oc::PathControl &p = ss_->getSolutionPath();
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
                const auto& s = state->as<ob::CompoundStateSpace>(0)->as<ob::SE3StateSpace::StateType>();
            
                double x = s->getX();
                /*
                std::cout <<  x << " "<< y << " "<<z<<std::endl;

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
                */
                return true;
            }
        void FixedWingODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
        {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

            qdot.resize (q.size (), 0);

            qdot[0] = q[7]*cos(q[8]);
            qdot[1] = q[7]*sin(q[8]);
            qdot[2] = q[7]*u[1];

            //qdot[3] = q[8];

            qdot[7] = u[0];
            qdot[8] = u[2];
            //qdot[9] = u[2];

        }

        ob::StateSpacePtr constructFWStateSpace(void)
        {
            ob::StateSpacePtr stateSpace = ob::StateSpacePtr(new ob::CompoundStateSpace());

            stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
            stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
            stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
            //stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
            stateSpace->as<ob::CompoundStateSpace>()->lock();
            return stateSpace;
        }

        void FixedWingPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
        {
            // Normalize orientation between 0 and 2*pi
            ob::CompoundStateSpace::StateType& s = *result->as<ob::CompoundStateSpace::StateType>();
            ob::SE3StateSpace::StateType& pose = *s.as<ob::SE3StateSpace::StateType>(0);


            FWspace->as<ob::CompoundStateSpace>()->getSubspace(1)->enforceBounds(s[1]);

            // Enforce steering bounds
            FWspace->as<ob::CompoundStateSpace>()->getSubspace(2)->enforceBounds(s[2]);
            //space->as<ob::CompoundStateSpace>()->getSubspace(3)->enforceBounds(s[3]);
            ob::ScopedState<ob::CompoundStateSpace> start(FWspace);

            double angle = start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] ;
            pose.rotation().setIdentity();
            pose.rotation().setAxisAngle(0,0,1, angle);

        }

        oc::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
};


int main(int argc, char *argv[])
{
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

    Eigen::Vector3d start(0.0,0.0,50.0);
    Eigen::Vector3d finish(2000.0, 3000.0, 100.0);
    Eigen::Vector4d start1(0.0,0.0,50.0, 12);
    Eigen::Vector4d finish1(2000.0, 3000.0, 100.0, 10);
    FixedWingEnvironment env;
        env.setWorld(world);
          env.plan(start1, finish1);
    /*
       if(env.plan(start,finish))
       {
       env.recordSolution();
       }
     */
    tf=Eigen::Isometry3d::Identity();
    // tf.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitY()));
    tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    tf.translation() = start;
    moveSkeleton(ball1, tf);

    MyWindow window(world);
    double oldx, oldy, oldz,angleRot,angleRotOld = 0.0;
    std::thread t([&]()
            {
            std::this_thread::sleep_for(std::chrono::seconds(3));
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

            if (abs(angleRot - angleRotOld) < M_PI/18) {
                angleRot = angleRotOld;
            }
            tf.rotate(Eigen::AngleAxisd(angleRot, Eigen::Vector3d::UnitY())); 
            tf.translation() = Eigen::Vector3d(x,y,z);

            moveSkeleton(ball1, tf);

            window.setViewTrack(Eigen::Vector3d(x,y,z), Eigen::AngleAxisd(-angleRot, Eigen::Vector3d::UnitZ()));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));			

            oldx = x; oldy = y; oldz = z; 
            angleRotOld = angleRot;
            }
            }
            });

    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "SDF");
    //planWithSimpleSetup();
    //window.refreshTimer(5);
    glutMainLoop();

    //    t.join();

    //std::cout << "OMPL version: " << OMPL_VERSION << std::endl;


    return 0;
}
