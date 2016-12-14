#ifndef MYWINDOW_H
#define MYWINDOW_H

#include <dart/dart.h>
#include <Eigen/Eigen>

namespace ds = dart::simulation;
namespace dg = dart::gui;

/**
 * @brief The MyWindow class renders the simulation, might be used to demonstrate the results
 */
class MyWindow : public dart::gui::SimWindow {
 public:
  /// ctor
  MyWindow(const ds::WorldPtr& world);

  /// fixes rendering
  void drawSkels() override;
  void setViewTrack(const Eigen::Vector3d& v, const Eigen::Quaterniond &rot);
 private:
 std::mutex readMutex;
};

#endif  // MYWINDOW_H
