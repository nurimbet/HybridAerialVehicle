
const double default_speed_increment = 0.5;

const int default_ik_iterations = 4500;

const double default_force =  50.0; // N
const int default_countdown = 100;  // Number of timesteps for applying force
const double default_shape_height  = 1;  // m
const double default_shape_density = 1000; // kg/m^3
const double default_shape_width   = 0.3; // m
const double default_skin_thickness = 1e-3; // m
const double default_restitution = 0.6;
const double default_damping_coefficient = 0.001;

#include "dart/dart.h"
#include "dart/dynamics/Skeleton.h"
#include "dart/simulation/World.h"
#include "dart/utils/sdf/SdfParser.h"
#include "dart/collision/CollisionDetector.h"
#define EIGEN_DONT_ALIGN_STATICALLY 1
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;
namespace dc = dart::collision;

class MyWindow : public SimWindow
{
    public:
        /// Constructor
        MyWindow(const WorldPtr& world)
//            : 
    {
        setWorld(world);
        mZoom = 0.005;

        //mController = std::unique_ptr<Controller>
        //    (new Controller(mWorld->getSkeleton("chicago")));
    }

        /// Handle keyboard input

        void drawSkels() {
            glEnable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    /*        static double alpha = 0;
            Eigen::Matrix3d mat;
            mat = Eigen::AngleAxisd(alpha / 181.0 * M_PI, Eigen::Vector3d::UnitZ());
            alpha += 0.5;

            Eigen::Quaterniond quat(mat);
            mTrackBall.setQuaternion(quat);
    */


            for (unsigned int i = 0; i < mWorld->getNumSkeletons(); i++) {
                mWorld->getSkeleton(i)->draw(mRI);
            }
        }
    protected:
        /// Number of iterations before clearing a force entry

};

    template<class JointType>
BodyNode* addRigidBody(const SkeletonPtr& chain, const std::string& name,
        Shape::ShapeType type, BodyNode* parent = nullptr)
{
    // Set the Joint properties
    typename JointType::Properties properties;
    properties.mName = name+"_joint";
    if(parent)
    {
        // If the body has a parent, we should position the joint to be in the
        // middle of the centers of the two bodies
        Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
        tf.translation() = Eigen::Vector3d(0, 0, default_shape_height / 2.0);
        properties.mT_ParentBodyToJoint = tf;
        properties.mT_ChildBodyToJoint = tf.inverse();
    }

    // Create the Joint and Body pair
    BodyNode* bn = chain->createJointAndBodyNodePair<JointType>(
            parent, properties, BodyNode::Properties(name)).second;

    // Make the shape based on the requested Shape type
    ShapePtr shape;
    /*if(Shape::BOX == type)
      {
      shape = std::make_shared<BoxShape>(Eigen::Vector3d(
      default_shape_width,
      default_shape_width,
      default_shape_height));
      }
      else if(Shape::CYLINDER == type)
      {
      shape = std::make_shared<CylinderShape>(default_shape_width/2.0,
      default_shape_height);
      }
      else */if(Shape::ELLIPSOID == type)
    {
        shape = std::make_shared<EllipsoidShape>(
                default_shape_height*Eigen::Vector3d(1,1,1));
    }

    bn->addVisualizationShape(shape);
    bn->addCollisionShape(shape);

    // Setup the inertia for the body
    /*Inertia inertia;
      double mass = default_shape_density * shape->getVolume();
      inertia.setMass(mass);
      inertia.setMoment(shape->computeInertia(mass));
      bn->setInertia(inertia);
     */
    // Set the coefficient of restitution to make the body more bouncy
    bn->setRestitutionCoeff(default_restitution);

    // Set damping to make the simulation more stable
    if(parent)
    {
        Joint* joint = bn->getParentJoint();
        for(size_t i=0; i < joint->getNumDofs(); ++i)
            joint->getDof(i)->setDampingCoefficient(default_damping_coefficient);
    }

    return bn;
}
void setAllColors(const SkeletonPtr& object, const Eigen::Vector3d& color)
{
    // Set the color of all the shapes in the object
    for(size_t i=0; i < object->getNumBodyNodes(); ++i)
    {
        BodyNode* bn = object->getBodyNode(i);
        for(size_t j=0; j < bn->getNumVisualizationShapes(); ++j)
            bn->getVisualizationShape(j)->setColor(color);
    }
}
// Load a chicago model and enable joint limits and self-collision
SkeletonPtr loadChicago()
{
    // Lesson 1

    // Create the world with a skeleton
    //WorldPtr world = SkelParser::readWorld(DART_DATA_PATH"skel/chicago.skel");
    //  WorldPtr world
    //      = SdfParser::readSdfFile(
    //           DART_DATA_PATH"/sdf/test/mesh.sdf");
    //"/home/nurimbet/Downloads/Chicago.sdf");
    SkeletonPtr chicago = SdfParser::readSkeleton("/home/nurimbet/Downloads/Chicago.sdf");
    //world->addSkeleton(skel);
    //assert(world != nullptr);

    //SkeletonPtr chicago = world->getSkeleton("chicago");
    //SkeletonPtr chicago = world->getSkeleton(0);

    return chicago;
}

SkeletonPtr createBall()
{
    /*
       SkeletonPtr ball = Skeleton::create("rigid_ball");

    // Give the ball a body
    addRigidBody<FreeJoint>(ball, "rigid ball", Shape::ELLIPSOID);

    setAllColors(ball, dart::Color::Red());
    Eigen::Vector6d positions(Eigen::Vector6d::Zero());


    shape = std::make_shared<EllipsoidShape>(
    default_shape_height*Eigen::Vector3d(1,1,1));
    // If randomization is on, we will randomize the starting y-location
    //ball->getJoint(0)->setPositions(positions);
     */
    // Add the object to the world
    SkeletonPtr ball = Skeleton::create("ball");

    // Give the floor a body
    BodyNodePtr body =
        ball->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    std::shared_ptr<EllipsoidShape> ball1(
            new EllipsoidShape(default_shape_height*Eigen::Vector3d(1, 1, 1)));
    //box->setColor(dart::Color::Black());

    body->addVisualizationShape(ball1);
    body->addCollisionShape(ball1);

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(1180.0,1125.0, 200.0);
    //tf.translation() = Eigen::Vector3d(15.0,12.0, 5.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    setAllColors(ball, dart::Color::Red());

    return ball;
}

SkeletonPtr createFloor()
{
    SkeletonPtr floor = Skeleton::create("floor");

    // Give the floor a body
    BodyNodePtr body =
        floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 1000.0;
    double floor_height = 10;
    std::shared_ptr<BoxShape> box(
            new BoxShape(Eigen::Vector3d(3*floor_width, 4*floor_width, floor_height)));
    //box->setColor(dart::Color::Black());

    body->addVisualizationShape(box);
    body->addCollisionShape(box);

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d(3*floor_width/2, 4*floor_width/2, -floor_height/2);
    //tf.translation() = Eigen::Vector3d(0, 0, 0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    setAllColors(floor, dart::Color::Black());

    return floor;
}

int main(int argc, char* argv[])
{
    SkeletonPtr floor = createFloor();

    // Lesson 1
    SkeletonPtr chicago = loadChicago();
    SkeletonPtr ball = createBall();


    WorldPtr world = std::make_shared<World>();
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.8));
    //world->getConstraintSolver()->setCollisionDetector(new dc::DARTCollisionDetector());
    world->getConstraintSolver()->setCollisionDetector(
      new dc::BulletCollisionDetector());
#ifdef HAVE_BULLET_COLLISION
    //world->getConstraintSolver()->setCollisionDetector(
    //         new dart::collision::BulletCollisionDetector());
#endif

//    world->addSkeleton(floor);
    world->addSkeleton(ball);
    world->addSkeleton(chicago);
    dart::collision::CollisionDetector* detector =
        world->getConstraintSolver()->getCollisionDetector();
    detector->detectCollision(true, true);
    size_t collisionCount = detector->getNumContacts();
    if(collisionCount > 0)
    {
        std::cout << "There has been a collision" << std::endl;
    }
    if(world->checkCollision())
    {

        std::cout << "There has been a collision" << std::endl;

    }
    // Create a window for rendering the world and handling user input
    

    int ii;
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    int maxnum = 100;
    for(ii = 0; ii < maxnum; ii++)
    {
        tf.translation() = Eigen::Vector3d(rand()%1000, rand()%1000, rand()%200);
        ball->getJoint(0)->setTransformFromParentBodyNode(tf);
        world->checkCollision();
      //  detector->detectCollision(true, true);
    }     
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
    std::cout << "Time to collision check "<<maxnum<<"  = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << " milliseconds (total)"<<std::endl;

 
   MyWindow window(world);

    // Print instructions
    /*  std::cout << "'.': forward push" << std::endl;
        std::cout << "',': backward push" << std::endl;
        std::cout << "'s': increase skateboard forward speed" << std::endl;
        std::cout << "'a': increase skateboard backward speed" << std::endl;
        std::cout << "space bar: simulation on/off" << std::endl;
        std::cout << "'p': replay simulation" << std::endl;
        std::cout << "'v': Turn contact force visualization on/off" << std::endl;
        std::cout << "'[' and ']': replay one frame backward and forward" << std::endl;
     */ 
    // Initialize glut, initialize the window, and begin the glut event loop
    glutInit(&argc, argv);
    window.initWindow(640*2, 480*2, "Chicago Skyline");
    window.refreshTimer(5);
    glutMainLoop();
}
