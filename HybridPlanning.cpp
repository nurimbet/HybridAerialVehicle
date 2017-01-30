// Default Projection est, pdst, KPIECE
#include <dart/dart.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <Eigen/Eigen>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iostream>
#include <limits>
#include <thread>
#include <valarray>
#include "config.h"
#include "mywindow.h"
#include "util.h"

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace ds = dart::simulation;
namespace dd = dart::dynamics;
namespace dc = dart::collision;
namespace du = dart::utils;
constexpr double kMaxWidth = 3000.0;
constexpr double kMaxLength = 4000.0;
constexpr double kMaxHeight = 500.0;

class QControlSpace : public oc::RealVectorControlSpace {
    public:
        QControlSpace(const ob::StateSpacePtr& stateSpace)
            : oc::RealVectorControlSpace(stateSpace, 4) {}
};
enum class QuadrotorType : char {TakeOff, Landing};
QuadrotorType typeGlob;
class QuadrotorEnvironment {
    public:
        QuadrotorEnvironment(QuadrotorType type) {
            typeGlob = type;
            Qspace = ob::StateSpacePtr(new ob::CompoundStateSpace());

            if(type == QuadrotorType::TakeOff) {
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 0.75);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.5);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);
            }
            else{
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 1.);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.5);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.5);
                Qspace->as<ob::CompoundStateSpace>()->addSubspace(
                        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.5);
            }
            Qspace->as<ob::CompoundStateSpace>()->lock();
            // control input is made up of the 3 states
            oc::ControlSpacePtr cspace(new QControlSpace(Qspace));

            ob::RealVectorBounds velboundx(1),velboundz(1),angleboundz(1),angleboundx(1), controlbounds(4);

            velboundz.setLow(0);
            velboundz.setHigh(15);
            Qspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(1)
                ->setBounds(velboundz);
            velboundx.setLow(-10);
            velboundx.setHigh(10);
            Qspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(3)
                ->setBounds(velboundx);
            angleboundz.setLow(-0.785*4);
            angleboundz.setHigh(0.785*4);
            Qspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(2)
                ->setBounds(angleboundz);
            angleboundx.setLow(-0.785);
            angleboundx.setHigh(0.785);
            Qspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(4)
                ->setBounds(angleboundx);

            controlbounds.setLow(-0.1);
            controlbounds.setHigh(0.1);

            controlbounds.setLow(1, -5);
            controlbounds.setHigh(1, 5);

            controlbounds.setLow(2, -5);
            controlbounds.setHigh(2, 5);

            cspace->as<QControlSpace>()->setBounds(controlbounds);

            ss_.reset(new oc::SimpleSetup(cspace));

            oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(
                        ss_->getSpaceInformation(),
                        std::bind(&QuadrotorEnvironment::QuadrotorODE, this,
                            std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3)));
            ss_->setStatePropagator(oc::ODESolver::getStatePropagator(
                        odeSolver, std::bind(&QuadrotorEnvironment::QuadrotorPostIntegration,
                            this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3, std::placeholders::_4)));
            ss_->getSpaceInformation()->setPropagationStepSize(0.1);
            ss_->getSpaceInformation()->setMinMaxControlDuration(10, 10);

            ob::RealVectorBounds bounds(3);
            bounds.setLow(0);
            bounds.setHigh(kMaxWidth );
            bounds.setLow(1, 0);
            bounds.setHigh(1, kMaxLength );
            bounds.setLow(2, 0);

            bounds.setHigh(2, kMaxHeight);

            ss_->getStateSpace()
                ->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(0)
                ->setBounds(bounds);

            Qspace->setup();

            ss_->setPlanner(ob::PlannerPtr(new oc::RRT(ss_->getSpaceInformation())));
            ss_->setStateValidityChecker(std::bind(&QuadrotorEnvironment::isStateValid,
                        this, std::placeholders::_1));
        }

        bool plan(const Eigen::Vector3d& init, const Eigen::Vector3d& final,const Eigen::Vector2d& ainit, const Eigen::Vector2d& afinal ){
            if (!ss_) return false;

            std::cout << init << std::endl;
            std::cout << final << std::endl;
            ob::RealVectorBounds bounds(3);
            if(typeGlob == QuadrotorType::TakeOff) {
                bounds.setLow(init[0] - 200);
                bounds.setHigh(init[0] + 200);
                bounds.setLow(1, init[1] - 200);
                bounds.setHigh(1,  init[1] + 200);
            }
            else{
                bounds.setLow(final[0] - 200);
                bounds.setHigh(final[0] + 200);
                bounds.setLow(1, final[1] - 200);
                bounds.setHigh(1,  final[1] + 200);
                if(final[0] - init[0] < 150 && final[0] - init[0] > 0)
                {
                    bounds.setLow(0,init[0] - 50); 
                    bounds.setHigh(0,final[0] + 50); 
                }
                else if(-final[0] + init[0] < 150 && -final[0] + init[0] > 0)
                {
                    bounds.setLow(0,final[0] - 50); 
                    bounds.setHigh(0,init[0] + 50); 

                }
                if(final[1] - init[1] < 150 && final[1] - init[1] > 0)
                {
                    bounds.setLow(1,init[1] - 50); 
                    bounds.setHigh(1,final[1] + 50); 
                }
                else if(-final[1] + init[1] < 150 && -final[1] + init[1] > 0)
                {
                    bounds.setLow(1,final[1] - 50); 
                    bounds.setHigh(1,init[1] + 50); 

                }

            }
            bounds.setLow(2, 0);
            bounds.setHigh(2, kMaxHeight);

            ss_->getStateSpace()
                ->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(0)
                ->setBounds(bounds);
            ob::ScopedState<ob::CompoundStateSpace> start(ss_->getStateSpace());

            //start->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = init[0];
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = init[1];
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = init[2];
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = ainit[0]; // NOTE
            start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = ainit[1];
            start->as<ob::RealVectorStateSpace::StateType>(4)->values[0] = 0;

            ob::ScopedState<ob::CompoundStateSpace> goal(ss_->getStateSpace());

            //goal->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = final[0];
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = final[1];
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = final[2];
            //std::cout << final[2] << std::endl;
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = afinal[0]; // NOTE
            goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = 0;
            goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = afinal[1];
            goal->as<ob::RealVectorStateSpace::StateType>(4)->values[0] = 0;

            ss_->setStartAndGoalStates(start, goal, 0.05);
            ss_->setup();

            // this will run the algorithm for one second
            if(typeGlob == QuadrotorType::TakeOff) {
                ss_->solve(60 * 1 * 20);
            }
            else{
                ss_->solve(60 * 4 * 7.5);
            }


            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                oc::PathControl& p = ss_->getSolutionPath();
                p.interpolate();

                // p.printAsMatrix(std::cout);
                std::ofstream vertices("vertices.txt");
                std::ofstream resultfile;
                resultfile.open("result.txt", std::ios::app);
                // ss_->getSolutionPath().asGeometric().printAsMatrix(resultfile);
                p.printAsMatrix(resultfile);
                ob::PlannerData pdat(ss_->getSpaceInformation());
                ss_->getPlannerData(pdat);
                for (std::size_t i = 0; i < pdat.numVertices(); ++i) {
                    // std::cout << pdat.getVertex(i)<<std::endl;

                    printEdge(vertices, ss_->getStateSpace(), pdat.getVertex(i));
                    vertices << std::endl;
                }
                return true;
            } else {
                return false;
            }
        }

        void setWorld(const ds::WorldPtr& world) { world_ = world; }

    private:
        bool isStateValid(const ob::State* state) {
            const ob::RealVectorStateSpace::StateType* s =
                state->as<ob::CompoundStateSpace::StateType>()
                ->components[0]
                ->as<ob::RealVectorStateSpace::StateType>();
            double x = s->values[0];
            double y = s->values[1];
            double z = s->values[2];

            Eigen::Isometry3d tf;
            tf = Eigen::Isometry3d::Identity();
            tf.translation() = Eigen::Vector3d(x, y, z);
            dd::SkeletonPtr uavball = world_->getSkeleton("huav");
            moveSkeleton(uavball, tf);

            return !world_->checkCollision() &&
                ss_->getSpaceInformation()->satisfiesBounds(state);
        }

        void QuadrotorODE(const oc::ODESolver::StateType& q,
                const oc::Control* control,
                oc::ODESolver::StateType& qdot) {
            const double* u =
                control->as<oc::RealVectorControlSpace::ControlType>()->values;

            qdot.resize(q.size(), 0);

            qdot[0] = q[3] * cos(q[4]) + q[5] * sin(q[4]) * sin(q[6]);
            qdot[1] = q[3] * sin(q[4]) - q[5] * cos(q[4]) * sin(q[6]);
            qdot[2] = q[5] * cos(q[6]);


            qdot[3] = u[1]; // vx
            qdot[5] = u[2]; // vz
            qdot[4] = u[3]; // wz
            qdot[6] = u[0]; // wx
        }

        void QuadrotorPostIntegration(const ob::State* /*state*/,
                const oc::Control* /*control*/,
                const double /*duration*/, ob::State* result) {
            // Normalize orientation between 0 and 2*pi

            ob::CompoundStateSpace::StateType& s =
                *result->as<ob::CompoundStateSpace::StateType>();

            Qspace->as<ob::CompoundStateSpace>()->getSubspace(1)->enforceBounds(s[1]);

            // Enforce steering bounds
            Qspace->as<ob::CompoundStateSpace>()->getSubspace(2)->enforceBounds(s[2]);
            ob::ScopedState<ob::CompoundStateSpace> start(Qspace);

        }

        void printEdge(std::ostream& os, const ob::StateSpacePtr& space,
                const ob::PlannerDataVertex& vertex) {
            std::vector<double> reals;
            if (vertex != ob::PlannerData::NO_VERTEX) {
                space->copyToReals(reals, vertex.getState());
                for (size_t j(0); j < reals.size(); ++j) os << " " << reals[j];
            }
        }

        oc::SimpleSetupPtr ss_;
        ds::WorldPtr world_;
        ob::StateSpacePtr Qspace;
};

class FWControlSpace : public oc::RealVectorControlSpace {
    public:
        FWControlSpace(const ob::StateSpacePtr& stateSpace)
            : oc::RealVectorControlSpace(stateSpace, 3) {}
};

class FixedWingEnvironment {
    public:
        FixedWingEnvironment() {
            FWspace = ob::StateSpacePtr(new ob::CompoundStateSpace());

            FWspace->as<ob::CompoundStateSpace>()->addSubspace(
                    ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 0.75);
            FWspace->as<ob::CompoundStateSpace>()->addSubspace(
                    ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 1.0);
            FWspace->as<ob::CompoundStateSpace>()->addSubspace(
                    ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), 0.0);
            FWspace->as<ob::CompoundStateSpace>()->addSubspace(
                    ob::StateSpacePtr(new ob::RealVectorStateSpace(3)), 0.0);
            FWspace->as<ob::CompoundStateSpace>()->lock();

            oc::ControlSpacePtr cspace(new FWControlSpace(FWspace));

            ob::RealVectorBounds velbounds(1), omegabounds(1), anglebounds(1), extrabounds(3),
                controlbounds(3);

            velbounds.setLow(10);
            velbounds.setHigh(30);
            FWspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(1)
                ->setBounds(velbounds);

            anglebounds.setLow(-4000);
            anglebounds.setHigh(4000);
            FWspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(2)
                ->setBounds(anglebounds);
            extrabounds.setLow(-4000);
            extrabounds.setHigh(4000);
            FWspace->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(3)
                ->setBounds(extrabounds);
            controlbounds.setLow(-1);  // V dot
            controlbounds.setHigh(1);
            controlbounds.setLow(1, -0.02);  // Z Dot
            controlbounds.setHigh(1, 0.02);
            controlbounds.setLow(2, -.06);  // PhiDot
            controlbounds.setHigh(2, .06);
            cspace->as<FWControlSpace>()->setBounds(controlbounds);

            ss_.reset(new oc::SimpleSetup(cspace));

            oc::ODESolverPtr odeSolver(new oc::ODEBasicSolver<>(
                        ss_->getSpaceInformation(),
                        std::bind(&FixedWingEnvironment::FixedWingODE, this,
                            std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3)));
            ss_->setStatePropagator(oc::ODESolver::getStatePropagator(
                        odeSolver, std::bind(&FixedWingEnvironment::FixedWingPostIntegration,
                            this, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3, std::placeholders::_4)));
            ss_->getSpaceInformation()->setPropagationStepSize(0.1);
            ss_->getSpaceInformation()->setMinMaxControlDuration(100, 100);

            ob::RealVectorBounds bounds(3);
            bounds.setLow(0);
            bounds.setHigh(kMaxWidth);
            bounds.setLow(1, 0);
            bounds.setHigh(1, kMaxLength);
            bounds.setLow(2, 0);
            bounds.setHigh(2, kMaxHeight);

            ss_->getStateSpace()
                ->as<ob::CompoundStateSpace>()
                ->as<ob::RealVectorStateSpace>(0)
                ->setBounds(bounds);

            FWspace->setup();

            ss_->setPlanner(ob::PlannerPtr(new oc::RRT(ss_->getSpaceInformation())));
            ss_->setStateValidityChecker(std::bind(&FixedWingEnvironment::isStateValid,
                        this, std::placeholders::_1));
        }

        bool plan(const Eigen::Vector3d& init, const Eigen::Vector3d& final,const Eigen::Vector2d& ainit, const Eigen::Vector2d& afinal ){
            if (!ss_) return false;

            ob::ScopedState<ob::CompoundStateSpace> start(ss_->getStateSpace());

            start->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = init[0];
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = init[1];
            start->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = init[2];
            start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = ainit[0]; // NOTE
            start->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = ainit[1];
            start->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = 0;
            start->as<ob::RealVectorStateSpace::StateType>(3)->values[2] = 0;

            ob::ScopedState<ob::CompoundStateSpace> goal(ss_->getStateSpace());

            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[0] = final[0];
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[1] = final[1];
            goal->as<ob::RealVectorStateSpace::StateType>(0)->values[2] = final[2];
            goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = afinal[0]; // NOTE
            goal->as<ob::RealVectorStateSpace::StateType>(2)->values[0] = afinal[1];
            goal->as<ob::RealVectorStateSpace::StateType>(3)->values[0] = 0;
            goal->as<ob::RealVectorStateSpace::StateType>(3)->values[1] = 0;
            goal->as<ob::RealVectorStateSpace::StateType>(3)->values[2] = 0;

            ss_->setStartAndGoalStates(start, goal, 0.05);
            ss_->setup();

            // this will run the algorithm for one second
            ss_->solve(60 * 1* 20);


            const std::size_t ns = ss_->getProblemDefinition()->getSolutionCount();
            OMPL_INFORM("Found %d solutions", (int)ns);
            if (ss_->haveSolutionPath()) {
                oc::PathControl& p = ss_->getSolutionPath();
                p.interpolate();

                // p.printAsMatrix(std::cout);
                std::ofstream resultfile;
                resultfile.open("result.txt", std::ios::app);
                // ss_->getSolutionPath().asGeometric().printAsMatrix(resultfile);
                p.printAsMatrix(resultfile);
                return true;
            } else
                return false;
        }

        void setWorld(const ds::WorldPtr& world) { world_ = world; }

    private:
        bool isStateValid(const ob::State* state) {
            const ob::RealVectorStateSpace::StateType* s =
                state->as<ob::CompoundStateSpace::StateType>()
                ->components[0]
                ->as<ob::RealVectorStateSpace::StateType>();
            double x = s->values[0];
            double y = s->values[1];
            double z = s->values[2];

            Eigen::Isometry3d tf;
            tf = Eigen::Isometry3d::Identity();
            tf.translation() = Eigen::Vector3d(x, y, z);

            dd::SkeletonPtr uavball = world_->getSkeleton("huav");
            moveSkeleton(uavball, tf);

            return !world_->checkCollision() &&
                ss_->getSpaceInformation()->satisfiesBounds(state);
        }

        void FixedWingODE(const oc::ODESolver::StateType& q,
                const oc::Control* control,
                oc::ODESolver::StateType& qdot) {
            const double* u =
                control->as<oc::RealVectorControlSpace::ControlType>()->values;

            qdot.resize(q.size(), 0);

            qdot[0] = q[3] * cos(q[4]);
            qdot[1] = q[3] * sin(q[4]);
            qdot[2] = q[3] * u[1];


            qdot[3] = u[0];
            qdot[4] = u[2];
        }
        /*
           ob::StateSpacePtr constructFWStateSpace() {
           ob::StateSpacePtr stateSpace =
           ob::StateSpacePtr(new ob::CompoundStateSpace());

           stateSpace->as<ob::CompoundStateSpace>()->addSubspace(
           ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
           stateSpace->as<ob::CompoundStateSpace>()->addSubspace(
           ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
           ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
// stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
// ob::RealVectorStateSpace(1)), .3);
stateSpace->as<ob::CompoundStateSpace>()->lock();
return stateSpace;
}
         */
void FixedWingPostIntegration(const ob::State* /*state*/,
        const oc::Control* /*control*/,
        const double /*duration*/, ob::State* result) {
    // Normalize orientation between 0 and 2*pi
    ob::CompoundStateSpace::StateType& s =
        *result->as<ob::CompoundStateSpace::StateType>();

    FWspace->as<ob::CompoundStateSpace>()->getSubspace(1)->enforceBounds(s[1]);

    // Enforce steering bounds
    FWspace->as<ob::CompoundStateSpace>()->getSubspace(2)->enforceBounds(s[2]);
    ob::ScopedState<ob::CompoundStateSpace> start(FWspace);

}

oc::SimpleSetupPtr ss_;
ds::WorldPtr world_;
ob::StateSpacePtr FWspace;
};

std::istream& ignoreline(std::ifstream& in, std::ifstream::pos_type& pos) {
    pos = in.tellg();
    return in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

std::string getLastLine(std::ifstream& in) {
    std::ifstream::pos_type pos = in.tellg();

    std::ifstream::pos_type lastPos;
    while (in >> std::ws && ignoreline(in, lastPos)) pos = lastPos;

    in.clear();
    in.seekg(pos);

    std::string line;
    std::getline(in, line);
    return line;
}

std::string getWorkingDirectory() {
    char buff[1024] = {0};

    getcwd(buff, 1024);

    return std::string(buff);
}

int main(int argc, char* argv[]) 
{
    ds::WorldPtr world = std::make_shared<ds::World>();
    world->getConstraintSolver()->setCollisionDetector(
            new dc::BulletCollisionDetector());

    std::string prefix = getWorkingDirectory();

    dd::SkeletonPtr chicago =
        du::SdfParser::readSkeleton(prefix + std::string("/Chicago.sdf"));
    setAllColors(chicago, Eigen::Vector3d(0.57, 0.6, 0.67));

    dd::SkeletonPtr huav = dd::Skeleton::create("huav");
    dd::SkeletonPtr huavball = dd::Skeleton::create("huavball");

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

    tf.translation() = Eigen::Vector3d(0, 0, 0);
    createBall(huav, Eigen::Vector3d(4, 4, 4), tf);
    createBall(huavball, Eigen::Vector3d(4, 4, 4), tf);

    world->addSkeleton(huav);
    huav = du::SdfParser::readSkeleton(prefix + std::string("/uav.sdf"));

    setAllColors(huav, Eigen::Vector3d(1, 0, 0));

    world->addSkeleton(chicago);
    world->addSkeleton(huav);



    double xs, ys, zs, vxs; 
    double xf, yf, zf, vxf;

    std::ifstream posfile("positions.txt");
    std::string posline;
    std::getline(posfile, posline);
    std::istringstream posisss(posline); 
    posisss >> xs >> ys >> zs >> vxs;
    std::getline(posfile, posline);
    std::istringstream posissf(posline); 
    posissf >> xf >> yf >> zf >> vxf;


    dd::SkeletonPtr heightbox = dd::Skeleton::create("heightbox");
    tf = Eigen::Isometry3d::Identity();
    int kk = 2;
    int heightResolution = 10;

    tf.translation() = Eigen::Vector3d(xs, ys, kk*heightResolution);
    createBox(heightbox, Eigen::Vector3d(400, 400, 5), tf);
    setAllColors(heightbox, Eigen::Vector3d(1.0, 0.0, 0.0));
    world->addSkeleton(heightbox);

    bool colcheck = world->checkCollision();
    while(colcheck && kk < ((kMaxHeight - 1 )/ heightResolution))
    {
        kk = kk + 1;
        tf.translation() = Eigen::Vector3d(xs, ys, kk*heightResolution);
        moveSkeleton(heightbox, tf);

        colcheck = world->checkCollision();

    }
    double maxHeightStart = kk*heightResolution;
    kk = 2;

    tf.translation() = Eigen::Vector3d(xf, yf, kk*heightResolution);
    moveSkeleton(heightbox, tf);

    colcheck = world->checkCollision();
    while(colcheck && kk < ((kMaxHeight - 1 )/ heightResolution))
    {
        kk = kk + 1;
        tf.translation() = Eigen::Vector3d(xf, yf, kk*heightResolution);
        moveSkeleton(heightbox, tf);

        colcheck = world->checkCollision();

    }
    double maxHeightFinish = 200;
    double raduis = 150.0;

    world->removeSkeleton(heightbox);

    std::cout << maxHeightStart << " " << maxHeightFinish << std::endl;

    posisss >> xs >> ys >> zs >>  vxs;
    Eigen::Quaterniond quats(1, 0,0,0);
    Eigen::Quaterniond quatf(1,0,0,0);

    Eigen::Vector3d start(xs, ys, zs);
    double anglerad =atan2((yf - ys) , (xf - xs));

    std::cout << anglerad << " " << xs + raduis*cos(anglerad) <<" " <<ys + raduis*sin(anglerad)<<  std::endl;
    Eigen::Vector3d start1(xs, ys, zs);
    Eigen::Vector3d finish1(xs + raduis*cos(anglerad), ys + raduis*sin(anglerad), maxHeightFinish);

    Eigen::Vector2d astart1(0, 0);
    Eigen::Vector2d afinish1(10, anglerad);

    Eigen::Vector3d start2(0, 0, 0);
    Eigen::Vector3d finish2(xf - raduis*cos(anglerad), yf - raduis*sin(anglerad), maxHeightFinish);
    Eigen::Vector2d astart2(0, 0);
    Eigen::Vector2d afinish2(0, 0);

    Eigen::Vector3d start3(0, 0, 0);
    Eigen::Vector3d finish3(xf, yf, zf);

    Eigen::Vector2d astart3(0, 0);
    Eigen::Vector2d afinish3(0, 0);

    if (argc < 2) {
        std::ofstream resultfile1;
        resultfile1.open("result.txt", std::ios::trunc);
        resultfile1.close();

        QuadrotorEnvironment env(QuadrotorType::TakeOff);
        env.setWorld(world);
        env.plan(start1, finish1, astart1, afinish1);

        std::ifstream file("result.txt");
        std::string line = getLastLine(file);
        file.close();

        std::cout << line << '\n';
        std::string delimiter = " ";

        size_t pos = 0;
        std::string token;
        int i = 0;
        int arsize = 6;
        float linear[arsize];
        while ((pos = line.find(delimiter)) != std::string::npos && i < arsize) {
            token = line.substr(0, pos);
            linear[i] = std::stof(token);
            line.erase(0, pos + delimiter.length());
            i++;
        }

        start2(0) = linear[0];
        start2(1) = linear[1];
        start2(2) = linear[2];
        if (linear[3] >= 10)
        {
            astart2(0) = linear[3];
        } 
        else{
            astart2(0) = 10;
        }
        astart2(1) = linear[4];
        afinish2(0) = 10;
        afinish2(1) = linear[4];
        finish2(2) = start2(2); 

        FixedWingEnvironment env1;
        env1.setWorld(world);

        env1.plan(start2, finish2,astart2, afinish2);


        std::ifstream file1("result.txt");
        line = getLastLine(file1);
        file1.close();

        std::cout << line << '\n';
        pos = 0;
        i = 0;

        while ((pos = line.find(delimiter)) != std::string::npos && i < arsize) {
            token = line.substr(0, pos);
            linear[i] = std::stof(token);
            line.erase(0, pos + delimiter.length());
            i++;
        }
        start3(0) = linear[0];
        start3(1) = linear[1];
        start3(2) = linear[2];
        if(linear[3] <= 15)
        {
            astart3(0) = linear[3];
        }
        else{
            astart3(0) = 10;
        }
        astart3(1) = linear[4];
        afinish3(0) = 0;
        afinish3(1) = linear[4];


        QuadrotorEnvironment env2(QuadrotorType::Landing);
        env2.setWorld(world);
        env2.plan(start3, finish3, astart3, afinish3);
    }

    dd::SkeletonPtr uavball = world->getSkeleton("huav");
    world->removeSkeleton(uavball);
    MyWindow window(world);

    //std::thread t([&]()
    //{
        //while (true) 
        //{
            std::ifstream fin("result.txt");
            int cnt = 0;

            while (!fin.eof()) 
            {
                cnt ++;
                float x, y, z, ign;
                float angz, angx;
                fin >> x >> y >> z >> ign >> angz >> ign >>
                angx >> ign >> ign >> ign >> ign >> ign;  

                Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();


                Eigen::Quaterniond quat(Eigen::AngleAxisd(angz, Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(angx, Eigen::Vector3d::UnitX()));

                tf.rotate(quat);
                tf.translation() = Eigen::Vector3d(x, y, z);
                
                if (cnt % 100 == 1){ 
                    
                    dd::SkeletonPtr huav1 = huav->clone();
                    world->addSkeleton(huav1);
                    moveSkeleton(huav1, tf);
                }

                window.setViewTrack(Eigen::Vector3d(x, y, z), Eigen::Quaterniond(1,0,0,0));
                //std::this_thread::sleep_for(std::chrono::milliseconds(5));

            }
            //std::this_thread::sleep_for(std::chrono::seconds(2));
        //}
    //}); 


    glutInit(&argc, argv);
    window.initWindow(640 * 2, 480 * 2, "SDF");
    glutMainLoop();

//   t.join();

    return 0;
}
