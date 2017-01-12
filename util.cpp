#include "util.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;

void setAllColors(const dd::SkeletonPtr& object, const Eigen::Vector3d& color) {
  // Set the color of all the shapes in the object
  for (size_t i = 0; i < object->getNumBodyNodes(); ++i) {
    dd::BodyNode* bn = object->getBodyNode(i);
    for (size_t j = 0; j < bn->getNumVisualizationShapes(); ++j)
      bn->getVisualizationShape(j)->setColor(color);
  }
}

void createBall(dd::SkeletonPtr& skl, const Eigen::Vector3d& size,
                const Eigen::Isometry3d& tf) {
  dd::BodyNode* bn = skl->createJointAndBodyNodePair<dd::FreeJoint>().second;

  std::shared_ptr<dd::EllipsoidShape> shape =
      std::make_shared<dd::EllipsoidShape>(size);
  shape->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  bn->getParentJoint()->setTransformFromParentBodyNode(tf);
}

void createBox(dd::SkeletonPtr& skl, const Eigen::Vector3d& size,
               const Eigen::Isometry3d& tf) {
  dd::BodyNode* bn = skl->createJointAndBodyNodePair<dd::FreeJoint>().second;

  std::shared_ptr<dd::BoxShape> shape = std::make_shared<dd::BoxShape>(size);
  shape->setColor(Eigen::Vector3d(0.8, 0.8, 0.8));

  bn->addCollisionShape(shape);
  bn->addVisualizationShape(shape);

  bn->getParentJoint()->setTransformFromParentBodyNode(tf);
}


void moveSkeleton(dd::SkeletonPtr& skl, const Eigen::Isometry3d& pose) {
  dd::FreeJoint::setTransform(skl.get(), pose);
}
