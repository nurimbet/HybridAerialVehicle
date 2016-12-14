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
constexpr double kMaxWidth = 3000;
constexpr double kMaxLength = 4000;
constexpr double kMaxHeight = 400;

class QControlSpace : public oc::RealVectorControlSpace {
 public:
  QControlSpace(const ob::StateSpacePtr& stateSpace)
      : oc::RealVectorControlSpace(stateSpace, 4) {}
};

class QuadrotorEnvironment {
 public:
  QuadrotorEnvironment() {
    Qspace = ob::StateSpacePtr(new ob::CompoundStateSpace());

    Qspace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
    Qspace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::RealVectorStateSpace(6)), 0.3);
    // stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
    // ob::RealVectorStateSpace(1)), .3);
    Qspace->as<ob::CompoundStateSpace>()->lock();

    oc::ControlSpacePtr cspace(new QControlSpace(Qspace));

    ob::RealVectorBounds velbounds(6), controlbounds(4);

    velbounds.setLow(-10);
    velbounds.setHigh(10);
    Qspace->as<ob::CompoundStateSpace>()
        ->as<ob::RealVectorStateSpace>(1)
        ->setBounds(velbounds);
    // omegabounds.setLow(-.2);
    // omegabounds.setHigh(.2);
    // space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3)->setBounds(omegabounds);

    // Qspace->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(2)->setBounds(anglebounds);
    controlbounds.setLow(-0.1);
    controlbounds.setHigh(0.1);
    controlbounds.setLow(0, 0);
    controlbounds.setHigh(0, 20);
    cspace->as<QControlSpace>()->setBounds(controlbounds);

    // oc::SimpleSetup ss(cspace);
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
    bounds.setHigh(kMaxWidth);
    bounds.setLow(1, 0);
    bounds.setHigh(1, kMaxLength);
    bounds.setLow(2, 0);

    bounds.setHigh(2, kMaxHeight);

    ss_->getStateSpace()
        ->as<ob::CompoundStateSpace>()
        ->as<ob::SE3StateSpace>(0)
        ->setBounds(bounds);

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

  bool plan(const Eigen::Vector3d& init, const Eigen::Vector3d& final) {
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
    // goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = 20;
    // goal->as<ob::RealVectorStateSpace::StateType>(1)->values[1] = 0;
    // goal->as<ob::RealVectorStateSpace::StateType>(1)->values[2] = 0;
    // goal->as<ob::RealVectorStateSpace::StateType>(1)->values[2] = 0;
    // goal->as<ob::RealVectorStateSpace::StateType>(1)->values[0] = final[3];

    ss_->setStartAndGoalStates(start, goal, 0.1);
    ss_->setup();

    // this will run the algorithm for one second
    ss_->solve(60 * 1);

    // ss_->solve(1000); // it will run for 1000 seconds

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
    // const ob::ScopedState<ob::CompoundStateSpace> st(ss_->getStateSpace(),
    // state);
    // const ob::SE3StateSpace::StateType* s =
    // st->as<ompl::base::SE3StateSpace::StateType>(0);
    // double x = s->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    const ob::SE3StateSpace::StateType* s =
        state->as<ob::CompoundStateSpace::StateType>()
            ->components[0]
            ->as<ob::SE3StateSpace::StateType>();
    // const ob::RealVectorStateSpace::StateType *pos =
    // pose->as<ob::RealVectorStateSpace::StateType>(0);
    double x = s->getX();
    double y = s->getY();
    double z = s->getZ();

    // std::cout <<  x << " "<< y << " "<<z<<std::endl;
    Eigen::Isometry3d tf;
    tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(x, y, z);
    dd::SkeletonPtr uavball = world_->getSkeleton("huav");
    // uavball->getJoint()->setTransformFromParentBodyNode(tf);
    moveSkeleton(uavball, tf);

    //
    // ADD CODE HERE
    //
    //

    // return true;  // stub
    return !world_->checkCollision() &&
           ss_->getSpaceInformation()->satisfiesBounds(state);
    // return true;
  }
  void QuadrotorODE(const oc::ODESolver::StateType& q,
                    const oc::Control* control,
                    oc::ODESolver::StateType& qdot) {
    const double* u =
        control->as<oc::RealVectorControlSpace::ControlType>()->values;

    // zero out qdot
    qdot.resize(q.size(), 0);

    // derivative of position
    qdot[0] = q[7];
    qdot[1] = q[8];
    qdot[2] = q[9];

    // derivative of orientation
    // 1. First convert omega to quaternion: qdot = omega * q / 2
    ob::SO3StateSpace::StateType qomega;
    qomega.w = 0;
    qomega.x = .5 * q[10];
    qomega.y = .5 * q[11];
    qomega.z = .5 * q[12];

    // 2. We include a numerical correction so that dot(q,qdot) = 0. This
    // constraint is
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
    qdot[7] =
        massInv_ * (2 * u[0] * (q[6] * q[4] + q[3] * q[5]) - beta_ * q[7]);
    qdot[8] =
        massInv_ * (2 * u[0] * (q[4] * q[5] - q[6] * q[3]) - beta_ * q[8]);
    qdot[9] =
        massInv_ *
            (u[0] * (q[6] * q[6] - q[3] * q[3] - q[4] * q[4] + q[5] * q[5]) -
             beta_ * q[9]) -
        9.81;  // - 9.81;

    // derivative of rotational velocity
    qdot[10] = u[1];
    qdot[11] = u[2];
    qdot[12] = u[3];
  }
  /*
     ob::StateSpacePtr constructQStateSpace()
     {
     ob::StateSpacePtr stateSpace = ob::StateSpacePtr(new
     ob::CompoundStateSpace());

     stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
     ob::SE3StateSpace()), 1.);
     stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
     ob::RealVectorStateSpace(1)), .3);
     stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
     ob::RealVectorStateSpace(1)), .3);
  //stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
  ob::RealVectorStateSpace(1)), .3);
  stateSpace->as<ob::CompoundStateSpace>()->lock();
  return stateSpace;
  }
   */
  void QuadrotorPostIntegration(const ob::State* /*state*/,
                                const oc::Control* /*control*/,
                                const double /*duration*/, ob::State* result) {
    // Normalize orientation between 0 and 2*pi

    const ob::CompoundStateSpace* cs = Qspace->as<ob::CompoundStateSpace>();
    const ob::SO3StateSpace* SO3 =
        cs->as<ob::SE3StateSpace>(0)->as<ob::SO3StateSpace>(1);
    ob::CompoundStateSpace::StateType& csState =
        *result->as<ob::CompoundStateSpace::StateType>();
    ob::SO3StateSpace::StateType& so3State =
        csState.as<ob::SE3StateSpace::StateType>(0)->rotation();

    // Normalize the quaternion representation for the quadrotor
    SO3->enforceBounds(&so3State);
    // Enforce velocity bounds
    cs->getSubspace(1)->enforceBounds(csState[1]);
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
        ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
    FWspace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
    FWspace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
    // stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
    // ob::RealVectorStateSpace(1)), .3);
    FWspace->as<ob::CompoundStateSpace>()->lock();

    oc::ControlSpacePtr cspace(new FWControlSpace(FWspace));

    ob::RealVectorBounds velbounds(1), omegabounds(1), anglebounds(1),
        controlbounds(3);

    velbounds.setLow(10);
    velbounds.setHigh(30);
    FWspace->as<ob::CompoundStateSpace>()
        ->as<ob::RealVectorStateSpace>(1)
        ->setBounds(velbounds);
    // omegabounds.setLow(-.2);
    // omegabounds.setHigh(.2);
    // space->as<ob::CompoundStateSpace>()->as<ob::RealVectorStateSpace>(3)->setBounds(omegabounds);

    anglebounds.setLow(-4000);
    anglebounds.setHigh(4000);
    FWspace->as<ob::CompoundStateSpace>()
        ->as<ob::RealVectorStateSpace>(2)
        ->setBounds(anglebounds);
    controlbounds.setLow(-0.3);  // V dot
    controlbounds.setHigh(0.3);
    controlbounds.setLow(1, -0.06);  // Z Dot
    controlbounds.setHigh(1, 0.06);
    controlbounds.setLow(2, -.06);  // PhiDot
    controlbounds.setHigh(2, .06);
    cspace->as<FWControlSpace>()->setBounds(controlbounds);

    // oc::SimpleSetup ss(cspace);
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
    bounds.setLow(2, 15);
    bounds.setHigh(2, kMaxHeight);

    ss_->getStateSpace()
        ->as<ob::CompoundStateSpace>()
        ->as<ob::SE3StateSpace>(0)
        ->setBounds(bounds);

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
    ss_->setPlanner(ob::PlannerPtr(new oc::RRT(ss_->getSpaceInformation())));
    ss_->setStateValidityChecker(std::bind(&FixedWingEnvironment::isStateValid,
                                           this, std::placeholders::_1));
  }

  bool plan(const Eigen::Vector4d& init, const Eigen::Vector4d& final) {
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

    ss_->setStartAndGoalStates(start, goal, 0.5);
    ss_->setup();

    // this will run the algorithm for one second
    ss_->solve(60);

    // ss_->solve(1000); // it will run for 1000 seconds

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
    // const ob::ScopedState<ob::CompoundStateSpace> st(ss_->getStateSpace(),
    // state);
    // const ob::SE3StateSpace::StateType* s =
    // st->as<ompl::base::SE3StateSpace::StateType>(0);
    // double x = s->as<ob::RealVectorStateSpace::StateType>(0)->values[0];
    const ob::SE3StateSpace::StateType* s =
        state->as<ob::CompoundStateSpace::StateType>()
            ->components[0]
            ->as<ob::SE3StateSpace::StateType>();
    // const ob::RealVectorStateSpace::StateType *pos =
    // pose->as<ob::RealVectorStateSpace::StateType>(0);
    double x = s->getX();
    double y = s->getY();
    double z = s->getZ();

    // std::cout <<  x << " "<< y << " "<<z<<std::endl;

    Eigen::Isometry3d tf;
    tf = Eigen::Isometry3d::Identity();
    tf.translation() = Eigen::Vector3d(x, y, z);

    dd::SkeletonPtr uavball = world_->getSkeleton("huav");
    // uavball->getJoint()->setTransformFromParentBodyNode(tf);
    moveSkeleton(uavball, tf);

    //
    // ADD CODE HERE
    //
    //

    // return true;  // stub
    return !world_->checkCollision() &&
           ss_->getSpaceInformation()->satisfiesBounds(state);
    // return true;
  }
  void FixedWingODE(const oc::ODESolver::StateType& q,
                    const oc::Control* control,
                    oc::ODESolver::StateType& qdot) {
    const double* u =
        control->as<oc::RealVectorControlSpace::ControlType>()->values;

    qdot.resize(q.size(), 0);

    qdot[0] = q[7] * cos(q[8]);
    qdot[1] = q[7] * sin(q[8]);
    qdot[2] = q[7] * u[1];

    // qdot[3] = q[8];

    qdot[7] = u[0];
    qdot[8] = u[2];
    // qdot[9] = u[2];
  }

  ob::StateSpacePtr constructFWStateSpace() {
    ob::StateSpacePtr stateSpace =
        ob::StateSpacePtr(new ob::CompoundStateSpace());

    stateSpace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::SE3StateSpace()), 1.);
    stateSpace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
    stateSpace->as<ob::CompoundStateSpace>()->addSubspace(
        ob::StateSpacePtr(new ob::RealVectorStateSpace(1)), .3);
    // stateSpace->as<ob::CompoundStateSpace>()->addSubspace(ob::StateSpacePtr(new
    // ob::RealVectorStateSpace(1)), .3);
    stateSpace->as<ob::CompoundStateSpace>()->lock();
    return stateSpace;
  }

  void FixedWingPostIntegration(const ob::State* /*state*/,
                                const oc::Control* /*control*/,
                                const double /*duration*/, ob::State* result) {
    // Normalize orientation between 0 and 2*pi
    ob::CompoundStateSpace::StateType& s =
        *result->as<ob::CompoundStateSpace::StateType>();
    ob::SE3StateSpace::StateType& pose = *s.as<ob::SE3StateSpace::StateType>(0);
    ob::RealVectorStateSpace::StateType& angleRVSP =
        *s.as<ob::RealVectorStateSpace::StateType>(2);
    double angle = angleRVSP.values[0];

    FWspace->as<ob::CompoundStateSpace>()->getSubspace(1)->enforceBounds(s[1]);

    // Enforce steering bounds
    FWspace->as<ob::CompoundStateSpace>()->getSubspace(2)->enforceBounds(s[2]);
    // space->as<ob::CompoundStateSpace>()->getSubspace(3)->enforceBounds(s[3]);
    ob::ScopedState<ob::CompoundStateSpace> start(FWspace);

    // double angle =
    // start->as<ob::RealVectorStateSpace::StateType>(1)->values[0] ;
    // const ob::SE3StateSpace::StateType* s =
    // state->as<ob::CompoundStateSpace::StateType>()
    //        ->components[0]->as<ob::SE3StateSpace::StateType>();
    pose.rotation().setIdentity();
    pose.rotation().setAxisAngle(0, 0, 1, angle);
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

int main(int argc, char* argv[]) {
  ds::WorldPtr world = std::make_shared<ds::World>();
  world->getConstraintSolver()->setCollisionDetector(
      new dc::BulletCollisionDetector());
  //    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));

  dd::SkeletonPtr chicago =
      du::SdfParser::readSkeleton(dart::common::Uri::createFromRelativeUri(
          std::string("."), std::string("Chicago.sdf")));
  // dd::SkeletonPtr chicago
  // =du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/Chicago.sdf"));
  setAllColors(chicago, Eigen::Vector3d(0.57, 0.6, 0.67));
  dd::SkeletonPtr huav = dd::Skeleton::create("huav");
  dd::SkeletonPtr huavball = dd::Skeleton::create("huavball");

  // dd::SkeletonPtr huav
  // =du::SdfParser::readSkeleton(("/home/arms/Downloads/HybridAerialVehicle/uav.sdf"));

  // du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

  // dd::SkeletonPtr huav = dd::Skeleton::create("huav");

  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());

  tf.translation() = Eigen::Vector3d(0, 0, 0);
  createBall(huav, Eigen::Vector3d(3, 3, 3), tf);
  createBall(huavball, Eigen::Vector3d(3, 3, 3), tf);

  world->addSkeleton(huav);
  // world->addSkeleton(huavball);
  huav = du::SdfParser::readSkeleton(dart::common::Uri::createFromRelativeUri(
      std::string(""), std::string("uav.sdf")));
  // huav =
  // du::SdfParser::readSkeleton(("/home/nurimbet/Research/HybridAerialVehicle/uav.sdf"));

  //    huav->getJoint(0)->setTransformFromParentBodyNode(tf);
  // moveSkeleton(huav, tf);

  // createBox(huav, size,tf) ;

  setAllColors(huav, Eigen::Vector3d(1, 1, 0));
  //
  // ADD CODE HERE
  // or use OOP if you want
  //

  // std::cout << "collision detected " << world->checkCollision() << std::endl;
  world->addSkeleton(chicago);
  world->addSkeleton(huav);

  Eigen::Vector3d start(10.0, 10.0, 30.0);
  Eigen::Vector3d finish(20.0, 30.0, 100.0);
  Eigen::Vector3d start1(20.0, 20.0, 20.0);
  Eigen::Vector3d finish1(50.0, 50.0, 100.0);
  Eigen::Vector3d finish2(2000.0, 3000.0, 100.0);
  Eigen::VectorXf start2(8);

#define PLAN
#ifdef PLAN
  std::ofstream resultfile1;
  resultfile1.open("result.txt", std::ios::trunc);
  resultfile1.close();

  QuadrotorEnvironment env;
  env.setWorld(world);
  env.plan(start1, finish1);

  std::ifstream file("result.txt");

  std::string line = getLastLine(file);
  // float x,y,z;
  // file >> x>>y>>z;
  //    std::cout << line << '\n';
  std::string delimiter = " ";

  size_t pos = 0;
  std::string token;
  int i = 0;
  int arsize = 10;
  float linear[arsize];
  while ((pos = line.find(delimiter)) != std::string::npos && i < arsize) {
    token = line.substr(0, pos);
    // std::cout << token << std::endl;
    linear[i] = std::stof(token);
    line.erase(0, pos + delimiter.length());
    i++;
    // std::cout << token << std::endl;
  }
  for (i = 0; i < 8; i++) {
    // std::cout << linear[i] << std::endl;
    start2(i) = linear[i];
  }
  std::cout << start2 << std::endl;
//   std::cout << x << " "<< y << " " << z << std::endl;
#endif
  dd::SkeletonPtr uavball = world->getSkeleton("huav");
  world->removeSkeleton(uavball);

  tf = Eigen::Isometry3d::Identity();
  // tf.rotate(Eigen::AngleAxisd(-M_PI/2,
  // Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-M_PI,
  // Eigen::Vector3d::UnitY()));
  // tf.rotate(Eigen::AngleAxisd(M_PI,
  // Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2,
  // Eigen::Vector3d::UnitX()));
  tf.translation() = start;

  MyWindow window(world);
  moveSkeleton(huav, tf);
  moveSkeleton(huavball, tf);
  double oldx, oldy, oldz, angleRot, angleRotOld = 0.0;
  std::thread t([&]() {
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    while (true) {
      std::ifstream fin("result.txt");

      while (!fin.eof()) {
        double x, y, z, ign;
        double rx, ry, rz, rw;
        // fin >> x >> y >> z >>
        // ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign>>ign;
        fin >> x >> y >> z >> rx >> ry >> rz >> rw >> ign >> ign >> ign >>
            ign >> ign >> ign >> ign >> ign >> ign >> ign >> ign;

        oldx = x - oldx;
        oldy = y - oldy;
        angleRot = -atan2(oldx, oldy);
        // std::cout << angleRot <<std::endl;
        // std::cout << rx << " " <<ry<< " "<<rz << " "<<rw<<std::endl;
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        // tf.rotate(Eigen::AngleAxisd(M_PI,
        // Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-M_PI/2,
        // Eigen::Vector3d::UnitX()));

        if (angleRot == -0) {
          angleRot = angleRotOld;
        }
        Eigen::Quaterniond quat(rw, rx, ry, rz);
        Eigen::Quaterniond quat1(rw, -rx, -ry, -rz);
        // tf.rotate(Eigen::AngleAxisd(angleRot, Eigen::Vector3d::UnitZ()));
        tf.rotate(quat);
        tf.translation() = Eigen::Vector3d(x, y, z);

        moveSkeleton(huav, tf);
        // moveSkeleton(huavball, tf);

        window.setViewTrack(Eigen::Vector3d(x, y, z), quat1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        oldx = x;
        oldy = y;
        oldz = z;
        angleRotOld = angleRot;
      }
      std::this_thread::sleep_for(std::chrono::seconds(2));
    }
  });

  glutInit(&argc, argv);
  window.initWindow(640 * 2, 480 * 2, "SDF");
  // planWithSimpleSetup();
  // window.refreshTimer(5);
  glutMainLoop();

  t.join();

  // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

  return 0;
}
