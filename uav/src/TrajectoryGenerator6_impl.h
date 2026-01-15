// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryGenerator6_impl.h
 * \brief Geometric trajectory through 4 points, XY quintic spline (C4), Z polynomial 
 * \date 2025/11/05
 */

#ifndef TRAJECTORYGENERATOR6_IMPL_H
#define TRAJECTORYGENERATOR6_IMPL_H

#include <Object.h>
#include <Vector3D.h>
#include <Eigen/Dense>
#include <vector>
#include <string>

namespace flair {
namespace core {
class Matrix;
class io_data;
}
namespace gui {
class LayoutPosition;
class DoubleSpinBox;
}
namespace filter {
class TrajectoryGenerator6;
}
}

class TrajectoryGenerator6_impl {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryGenerator6_impl(
      flair::filter::TrajectoryGenerator6 *self,
      const flair::gui::LayoutPosition *position, std::string name);
  ~TrajectoryGenerator6_impl();

  void Update(flair::core::Time time);
  void StartTraj(const flair::core::Vector3Df &start_pos,
                 const flair::core::Vector3Df &end_pos,
                 const flair::core::Vector3Df &start_vel,
                 float start_yaw);
  void StartTraj(const flair::core::Vector3Df &start_pos);
  void FinishTraj(void);

  void setTargetPosition(flair::core::Vector3Df posTarget, float yawTarget);
  float GetYaw(void) const;

  bool is_running;
  flair::core::Matrix *output;

  flair::gui::DoubleSpinBox *T;
  flair::gui::DoubleSpinBox *tobj_ui;
  flair::gui::DoubleSpinBox *Gripper;

private:
  // ========= spline (XY) =========
  struct QuinticSpline1D {
    std::vector<double> T;                         // segment durations
    std::vector<Eigen::Matrix<double,6,1>> c;      // coeffs per segment
  };

  QuinticSpline1D BuildSplineC4_1D(const std::vector<double>& P,
                                  const std::vector<double>& Tseg,
                                  double v0, double a0,
                                  double vf, double af);

  void EvalSpline1D(const QuinticSpline1D& spl, double t,
                    double &p, double &v, double &a) const;

  QuinticSpline1D sx, sy; // NOTE: Z will NOT use spline (kept polynomial)

  // ========= Z (original polynomial behavior) =========
  void CalculateCoefficientsZ(double zi, double zm, double zf,
                              double ti_traj, double tobj, double tf_traj);

  // ========= yaw (as before) =========
  Eigen::Matrix<double,6,1> CalculateCoefficientsYaw(double yaw0, double yawTarget,
                                                     double ti, double tfac);

  double EvaluatePosition(double t, const Eigen::VectorXd& coef);
  double EvaluateVelocity(double t, const Eigen::VectorXd& coef);
  double EvaluateAcceleration(double t, const Eigen::VectorXd& coef);

  // Time
  flair::core::Time previous_time;
  float CurrentTime;
  bool first_update, is_finishing;

  // Timing
  double tf;        // total time for XY spline (T0+T1+T2)
  double tfac;      // yaw transition time (end of seg0)
  double tobj;      // capture time (end of seg1)

  // Z timing (original)
  double ti_traj;
  double tf_traj;

  // Points
  flair::core::Vector3Df start_pos, des_pos, targetPosition;
  flair::core::Vector3Df P0, P1, P2, P3;

  // Segment times (XY)
  double T0, T1, T2;

  // Z coefficients (7th with 7 coeffs)
  Eigen::Matrix<double,7,1> coefficients;
  Eigen::Matrix<double,7,1> coefficients_z;

  // Yaw
  Eigen::Matrix<double,6,1> coefficients_yaw;
  float computed_yaw;
  bool yaw_frozen;
  float frozen_yaw;
  float targetYaw;
  float yaw0;
};

#endif // TRAJECTORYGENERATOR6_IMPL_H
