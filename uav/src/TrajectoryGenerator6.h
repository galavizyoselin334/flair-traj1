// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryGenerator6.h
 * \brief Trajectory generator wrapper (pimpl)
 * \author Custom Implementation
 * \date 2025/11/03
 * \version 1.1 (dynamic target API)
 */

#ifndef TRAJECTORYGENERATOR6_H
#define TRAJECTORYGENERATOR6_H

#include <IODevice.h>
#include <Vector3D.h>



namespace flair {
namespace core {
class Matrix;
class io_data;     
}
namespace gui {
class LayoutPosition;
}
}

class TrajectoryGenerator6_impl;

namespace flair {
namespace filter {

class TrajectoryGenerator6 : public core::IODevice {
public:
  TrajectoryGenerator6(const gui::LayoutPosition *position,
                       std::string name);
  ~TrajectoryGenerator6();

  void StartTraj(const core::Vector3Df &start_pos,
                 const core::Vector3Df &end_pos,
                 const core::Vector3Df &start_vel,
                 float start_yaw);

  void StartTraj(const core::Vector3Df &start_pos);

  void StopTraj(void);
  void FinishTraj(void);

  void SetMaxVelocity(float value);
  void SetAcceleration(float value);

  void Update(core::Time time);

  void GetPosition(core::Vector3Df &point) const;
  void GetSpeed(core::Vector3Df &point) const;
  void GetAcceleration(core::Vector3Df &point) const;

  // Compatibilidad (viejo)
  void updateTarget(core::Vector3Df posTarget, float yawTarget);

  // NUEVO: target dinámico (pos + vel + yaw)
  void updateTargetState(core::Vector3Df posTarget,
                         core::Vector3Df velTarget,
                         float yawTarget);

  float GetYaw(void) const;

  core::Matrix *GetMatrix(void) const;
  bool IsRunning(void) const;

private:
  void UpdateFrom(const core::io_data *data) override { (void)data; }
  TrajectoryGenerator6_impl *pimpl_;
};

} // end namespace filter
} // end namespace flair

#endif // TRAJECTORYGENERATOR6_H
