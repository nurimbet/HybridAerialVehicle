#ifndef UTIL_H
#define UTIL_H
#include "dart/dart.h"

namespace dd = dart::dynamics;
namespace ds = dart::simulation;
/**
 * @brief createBall method creates a spherical shape
 * @param size
 * @param tf
 * @return
 */
void createBall(dd::SkeletonPtr& skl, const Eigen::Vector3d& size,
                const Eigen::Isometry3d& tf);

/**
 * @brief createBox method creates a rectangular shape
 * @param size
 * @param tf
 * @return
 */
void createBox(dd::SkeletonPtr& skl, const Eigen::Vector3d& size,
               const Eigen::Isometry3d& tf);

/**
 * @brief moveSkeleton is a method to move skeleton
 * @param skl
 * @param pose
 */
void moveSkeleton(dd::SkeletonPtr& skl, const Eigen::Isometry3d& pose);

/**
 * @brief setAllColors
 * @param object
 * @param color
 */
void setAllColors(const dd::SkeletonPtr& object, const Eigen::Vector3d& color);


#endif  // UTIL_H
