// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryGenerator6.h
 * \brief Class generating a linear trajectory in 3D
 * \author Custom Implementation
 * \date 2025/11/03
 * \version 1.0
 */

#ifndef TRAJECTORYGENERATOR6_H
#define TRAJECTORYGENERATOR6_H

#include <IODevice.h>
#include <Vector3D.h>

namespace flair {
namespace core {
class Matrix;
}
namespace gui {
class LayoutPosition;
}
}

class TrajectoryGenerator6_impl;

namespace flair {
namespace filter {
/*! \class TrajectoryGenerator6
*
* \brief Class generating a linear trajectory in 3D
*
* This class generates position, velocity and acceleration references
* for a straight line trajectory in 3D space with smooth acceleration
* and deceleration profiles.
*/
class TrajectoryGenerator6 : public core::IODevice {
public:
  /*!
  * \brief Constructor
  *
  * Construct a TrajectoryGenerator6 at position.
  *
  * \param position position to display settings
  * \param name name
  */
  TrajectoryGenerator6(const gui::LayoutPosition *position,
                            std::string name);

  /*!
  * \brief Destructor
  *
  */
  ~TrajectoryGenerator6();

  /*!
  * \brief Start trajectory
  *
  * \param start_pos start position
  * \param end_pos end position
  */
  void StartTraj(const core::Vector3Df &start_pos,
                 const core::Vector3Df &end_pos,
                const core::Vector3Df &start_vel);

  /*!
  * \brief Start trajectory using UI target values
  *
  * \param start_pos start position
  */
  void StartTraj(const core::Vector3Df &start_pos);

  /*!
  * \brief Stop trajectory
  *
  * Stop abruptly the trajectory.
  */
  void StopTraj(void);

  /*!
  * \brief Finish trajectory
  *
  * Finish smoothly the trajectory with deceleration.
  */
  void FinishTraj(void);

  /*!
  * \brief Set maximum velocity
  *
  * \param value maximum velocity in m/s
  */
  void SetMaxVelocity(float value);

  /*!
  * \brief Set acceleration
  *
  * \param value acceleration in m/s²
  */
  void SetAcceleration(float value);

  /*!
  * \brief Update using provided time
  *
  * \param time time of the update
  */
  void Update(core::Time time);

  /*!
  * \brief Position
  *
  * \param point returned position
  */
  void GetPosition(core::Vector3Df &point) const;

  /*!
  * \brief Speed
  *
  * \param point returned speed
  */
  void GetSpeed(core::Vector3Df &point) const;
  void updateTarget(core::Vector3Df posTarget);

  /*!
  * \brief Acceleration
  *
  * \param point returned acceleration
  */
  void GetAcceleration(core::Vector3Df &point) const;

  /*!
  * \brief Output matrix
  *
  * \return matrix
  */
  core::Matrix *GetMatrix(void) const;

  /*!
  * \brief Is trajectory running?
  *
  * \return true if trajectory is running
  */
  bool IsRunning(void) const;

private:
  /*!
  * \brief Update using provided datas
  *
  * Reimplemented from IODevice.
  *
  * \param data data from the parent to process
  */
  void UpdateFrom(const core::io_data *data) override{};

  TrajectoryGenerator6_impl *pimpl_;
};
} // end namespace filter
} // end namespace flair
#endif // TrajectoryGenerator6_H