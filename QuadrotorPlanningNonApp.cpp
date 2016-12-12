//Default Projection est, pdst, KPIECE
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h> 
#include <ompl/config.h> 
#include <ompl/control/planners/rrt/RRT.h> 
#include <ompl/control/planners/kpiece/KPIECE1.h>
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
constexpr double kMaxWidth = 3000;
constexpr double kMaxLength = 4000;
constexpr double kMaxHeight = 400;


class QControlSpace : public oc::RealVectorControlSpace
{
    public:

        QControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 4)
    {
    }
};


class QuadrotorEnvironment{
    public:
        QuadrotorEnvironment() {


            Qspace = ob::StateSpacePtr(new ob::CompoundStateSpace());

            Qspace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
            Qspace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(6)), .3);
            //stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
            Qspace->as<ob::CompoundStateSpace>()->lock();

            oc::ControlSpacePtr cspace(new QControlSpace(Qspace));


            ob::RealVectorBounds velbounds(6), controlbounds(4);

            velbounds.setLow(-100);
            velbounds.setHigh(100);
            Qspace->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(1)->setBounds(velbounds);
            //omegabounds.setLow(-.2);
            //omegabounds.setHigh(.2);
            //space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3)->setBounds(omegabounds);

            //Qspace->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2)->setBounds(anglebounds);
            controlbounds.setLow(-1);
            controlbounds.setHigh(1);
            controlbounds.setLow(0, -20);
            controlbounds.setHigh(0, 20);
            cspace->as<QControlSpace>()->setBounds(controlbounds);



            //oc::SimpleSetup ss(cspace);
            ss_.reset(new oc::SimpleSetup(cspace));

            oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<> (ss_->getSpaceInformation(),std::bind( &QuadrotorEnvironment::QuadrotorODE, this,std::placeholders::_1, std::placeholders::_2,std::placeholders::_3)));
            ss_->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, std::bind(&QuadrotorEnvironment::QuadrotorPostIntegration, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3, std::placeholders::_4)));
            ss_->getSpaceInformation()->setPropagationStepSize(0.1);
            ss_->getSpaceInformation()->setMinMaxControlDuration(10,10);

            ob::RealVectorBounds bounds(3);
            bounds.setLow(0);
            bounds.setHigh(kMaxWidth);
            bounds.setLow(1,0);
            bounds.setHigh(1,kMaxLength);
            bounds.setLow(2,0);

            bounds.setHigh(2,kMaxHeight);



            ss_->getStateSpace()->as<ob::CompoundStateSpace>()->as<ob::SE3StateSpace>(0)->setBounds(bounds);

            Qspace->setup();

            /*
               ob::RealVectorStateSpace *space = new ob::RealVectorStateSpace();

               space->addDimension(0.0, kMaxWidth);
               space->addDimension(0.0, kMaxLength);
               space->addDimension(0.0, kMaxHeight);

               ss_.reset(new oc::SimpleSetup(ob::StateSpacePtr(space)));

            // set state validity checking for this space
            ss_->setStateValidityChecker(std::bind(&QuadrotorEnvironment::isStateValid,
            this, std::placeholders::_1));
            space->setup();
            //ss_->getSpaceInformation()->setStateValidityCheckingResolution(
            //        1.0 / space->getMaximumExtent());
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(
            1e-4);

             */
            ss_->setPlanner(ob::PlannerPtr(new oc::RRT(ss_->getSpaceInformation())));
            ss_->setStateValidityChecker(std::bind(&QuadrotorEnvironment::isStateValid,
                        this, std::placeholders::_1));

        }

        bool plan(const Eigen::Vector3d &init, const Eigen::Vector3d & final) {
            if (!ss_) return false;

            ob::ScopedState<ob::CompoundStateSpace> start(ss_->getStateSpace());

            start->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            start->as<ob::SE3StateSpace::StateType>(0)->setX(init[0]);
            start->as<ob::SE3StateSpace::StateType>(0)->setY(init[1]);
            start->as<ob::SE3StateSpace::StateType>(0)->setZ(init[2]);
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[2] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[3] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[4] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[5] = 0;


            ob::ScopedState<ob::CompoundStateSpace> goal(ss_->getStateSpace());

            goal->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            goal->as<ob::SE3StateSpace::StateType>(0)->setX(final[0]);
            goal->as<ob::SE3StateSpace::StateType>(0)->setY(final[1]);
            goal->as<ob::SE3StateSpace::StateType>(0)->setZ(final[2]);
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 2;
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 2;
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[2] = 2;
            //goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = final[3];

            ss_->setStartAndGoalStates(start, goal,5);
            ss_->setup();

            // this will run the algorithm for one second
            ss_->solve(60*1);

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
        bool isStateValid(const ob::State *state) {

            //const ob::ScopedState<ob::CompoundStateSpace> st(ss_->getStateSpace(), state);
            //const ob::SE3StateSpace::StateType* s = st->as<ompl::base::SE3StateSpace::StateType>(0);
            //double x = s->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
            const ob::SE3StateSpace::StateType* s = state->as<ob::CompoundStateSpace::StateType>()
                ->components[0]->as<ob::SE3StateSpace::StateType>();
            //const ob::RealVectorStateSpace::StateType *pos = pose->as<ob::RealVectorStateSpace::StateType>(0);
            double x = s->getX();
            double y = s->getY();
            double z = s->getZ();

            //std::cout <<  x << " "<< y << " "<<z<<std::endl;
            Eigen::Isometry3d tf; 
            tf = Eigen::Isometry3d::Identity(); 
            tf.translation() = Eigen::Vector3d(x,y,z); 
            dd::SkeletonPtr uavball = world_->getSkeleton("huav");
            //uavball->getJoint()->setTransformFromParentBodyNode(tf);
            moveSkeleton(uavball, tf);

            //
            // ADD CODE HERE
            //
            //

            //return true;  // stub
            return !world_->checkCollision() &&  ss_->getSpaceInformation()->satisfiesBounds(state);
            //return true;
        }
        void QuadrotorODE (const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
        {
            const double *u = control->as<oc::RealVectorControlSpace::ControlType>()->values;

            // zero out qdot
            qdot.resize (q.size (), 0);

            // derivative of position
            qdot[0] = q[7];
            qdot[1] = q[8];
            qdot[2] = q[9];

            // derivative of orientation
            // 1. First convert omega to quaternion: qdot = omega * q / 2
            ob::SO3StateSpace::StateType qomega;
            qomega.w = 0;
            qomega.x = .5*q[10];
            qomega.y = .5*q[11];
            qomega.z = .5*q[12];

            // 2. We include a numerical correction so that dot(q,qdot) = 0. This constraint is
            // obtained by differentiating q * q_conj = 1
            double delta = q[3] * qomega.x + q[4] * qomega.y + q[5] * qomega.z;

            // 3. Finally, set the derivative of orientation
            qdot[3] = qomega.x - delta * q[3];
            qdot[4] = qomega.y - delta * q[4];
            qdot[5] = qomega.z - delta * q[5];
            qdot[6] = qomega.w - delta * q[6];

            // derivative of velocity
            // the z-axis of the body frame in world coordinates is equal to
            // (2(wy+xz), 2(yz-wx), w^2-x^2-y^2+z^2).
            // This can be easily verified by working out q * (0,0,1).
            double massInv_ = 1.0;
            double beta_ = 0.1;
            qdot[7] = massInv_ * (-2*u[0]*(q[6]*q[4] + q[3]*q[5]) - beta_ * q[7]);
            qdot[8] = massInv_ * (-2*u[0]*(q[4]*q[5] - q[6]*q[3]) - beta_ * q[8]);
            qdot[9] = massInv_ * (  -u[0]*(q[6]*q[6]-q[3]*q[3]-q[4]*q[4]+q[5]*q[5]) - beta_ * q[9]) - 9.81;

            // derivative of rotational velocity
            qdot[10] = u[1];
            qdot[11] = u[2];
            qdot[12] = u[3];

        }
        /*
           ob::StateSpacePtr constructQStateSpace()
           {
           ob::StateSpacePtr stateSpace = ob::StateSpacePtr(new ob::CompoundStateSpace());

           stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
           stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
           stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
        //stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
        stateSpace->as<ob::CompoundStateSpace>()->lock();
        return stateSpace;
        }
         */
        void QuadrotorPostIntegration (const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State *result)
        {
            // Normalize orientation between 0 and 2*pi

            const ob::CompoundStateSpace* cs = Qspace->as<ob::CompoundStateSpace>();
            const ob::SO3StateSpace* SO3 = cs->as<ob::SE3StateSpace>(0)->as<ob::SO3StateSpace>(1);
            ob::CompoundStateSpace::StateType& csState = *result->as<ob::CompoundStateSpace::StateType>();
            ob::SO3StateSpace::StateType& so3State = csState.as<ob::SE3StateSpace::StateType>(0)->rotation();

            // Normalize the quaternion representation for the quadrotor
            SO3->enforceBounds(&so3State);
            // Enforce velocity bounds
            cs->getSubspace(1)->enforceBounds(csState[1]);
        }

        oc::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
        ob::StateSpacePtr Qspace;
};


int main(int argc, char *argv[])
{
    ds::WorldPtr world = std::make_shared<ds::World>();
    world->getConstraintSolver()->setCollisionDetector(
            new dc::BulletCollisionDetector());
    //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

    dd::SkeletonPtr chicago =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/Chicago.sdf"));
    //dd::SkeletonPtr chicago =du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/Chicago.sdf"));
    setAllColors(chicago, Eigen::Vector3d(0.57,0.6,0.67));
    dd::SkeletonPtr huav = dd::Skeleton::create("huav");
    //dd::SkeletonPtr huav =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/uav.sdf"));

    //du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

    //dd::SkeletonPtr huav = dd::Skeleton::create("huav");

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf.translation() = Eigen::Vector3d(0, 0, 0);
    createBall(huav, Eigen::Vector3d(6, 6, 6), tf) ;
    world->addSkeleton(huav);
    huav = du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/uav.sdf"));
    //huav = du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

    //    huav->getJoint(0)->setTransformFromParentBodyNode(tf);
    //moveSkeleton(huav, tf);

    //createBox(huav, size,tf) ; 

    setAllColors(huav, Eigen::Vector3d(1,1,0));
    //
    // ADD CODE HERE
    // or use OOP if you want
    //

    //std::cout << "collision detected " << world->checkCollision() << std::endl;
    world->addSkeleton(chicago);
    world->addSkeleton(huav);

    Eigen::Vector3d start(10.0,10.0,30.0);
    Eigen::Vector3d finish(20.0, 30.0, 100.0);
    Eigen::Vector3d start1(10.0,10.0,30.0);
    Eigen::Vector3d finish1(40.0, 40.0, 100.0);

    QuadrotorEnvironment env;
    env.setWorld(world);
    env.plan(start1, finish1);

    dd::SkeletonPtr uavball = world->getSkeleton("huav");
    world->removeSkeleton(uavball);

    tf=Eigen::Isometry3d::Identity();
    // tf.rotate(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitY()));
    //tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    tf.translation() = start;

    MyWindow window(world);
    moveSkeleton(huav, tf);
    double oldx, oldy, oldz,angleRot,angleRotOld = 0.0;
    std::thread t([&]()
            {
            //std::this_thread::sleep_for(std::chrono::seconds(1));
            while(true)
            {
            std::ifstream fin("result.txt");


            while(!fin.eof())
            {

            double x,y,z,ign;
            double rx,ry,rz,rw;
            //fin >> x >> y >> z >> ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign;
            fin >> x >> y >> z >> rx>>ry>>rz>> rw>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign;

            oldx = x - oldx;
            oldy = y - oldy;
            angleRot = -atan2(oldx, oldy);
            //std::cout << angleRot <<std::endl; 
            //std::cout << rx << " " <<ry<< " "<<rz << " "<<rw<<std::endl;
            Eigen::Isometry3d tf=Eigen::Isometry3d::Identity();
            //tf.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));

            if (angleRot == -0) {
                angleRot = angleRotOld;
            }
            Eigen::Quaterniond quat(rw, rx,ry,rz);
            //tf.rotate(Eigen::AngleAxisd(angleRot, Eigen::Vector3d::UnitZ()));
            tf.rotate(quat);
            tf.translation() = Eigen::Vector3d(x,y,z);

            moveSkeleton(huav, tf);

            window.setViewTrack(Eigen::Vector3d(x,y,z), Eigen::AngleAxisd(-angleRot, Eigen::Vector3d::UnitZ()));
            std::this_thread::sleep_for(std::chrono::milliseconds(50));			

            oldx = x; oldy = y; oldz = z; 
            angleRotOld = angleRot;
            }
    std::this_thread::sleep_for(std::chrono::seconds(2));
            }
            });

    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "SDF");
    //planWithSimpleSetup();
    //window.refreshTimer(5);
    glutMainLoop();

    t.join();

    //std::cout << "OMPL version: " << OMPL_VERSION << std::endl;


    return 0;
}
