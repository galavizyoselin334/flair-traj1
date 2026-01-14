// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryGenerator6_impl.h
 * \brief Class generating a 6th grade polynomial trajectory in 3D
 * \author Custom Implementation - Based on Python sim_quad6.py
 * \date 2025/11/05
 * \version 2.1 (dynamic replanning)
 */

#ifndef TRAJECTORYGENERATOR6_IMPL_H
#define TRAJECTORYGENERATOR6_IMPL_H

#include <Object.h>
#include <Vector3D.h>
#include <Eigen/Dense>

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

  // Compatibilidad
  void setTargetPosition(flair::core::Vector3Df posTarget, float yawTarget);

  // NUEVO: target dinámico (pos + vel + yaw)
  void setTargetState(flair::core::Vector3Df posTarget,
                      flair::core::Vector3Df velTarget,
                      float yawTarget);

  float GetYaw(void) const;

  bool is_running;
  flair::core::Matrix *output;

  // UI controls - trajectory timing
  flair::gui::DoubleSpinBox *T;
  flair::gui::DoubleSpinBox *tobj_ui;   // Time to pick object (horizon base)
  flair::gui::DoubleSpinBox *Gripper;   // Z offset (hook length)

private:
  // === planning helper (nuevo) ===
  void PlanFromState(const flair::core::Vector3Df &p0,
                     const flair::core::Vector3Df &v0,
                     float yaw0_now,
                     const flair::core::Vector3Df &objPos,
                     float objYaw);

  // === coef generation ===
  void CalculateCoefficientsZ(double zi, double zm, double zf,
                              double ti_traj, double tobj, double tf_traj);

  Eigen::Matrix<double,6,1> CalculateCoefficientsXY(double xi, double xobj, double xbef, double xf, double tfac,
                                                    double ti, double tobj, double tf,
                                                    double vi, double vf);

  Eigen::Matrix<double,6,1> CalculateCoefficientsYaw(double yaw0, double yawTarget,
                                                     double ti, double tfac);

  double EvaluatePosition(double t, const Eigen::VectorXd& coef);
  double EvaluateVelocity(double t, const Eigen::VectorXd& coef);
  double EvaluateAcceleration(double t, const Eigen::VectorXd& coef);

  // === Timing ===
  flair::core::Time previous_time;
  float CurrentTime;
  bool first_update, is_finishing;

  // Trajectory timing parameters
  double ti_traj;   // inicia trayectoria en z
  double tf;        // horizonte del segmento
  double tf_traj;   // termina trayectoria en z
  double tobj;      // "tiempo de pickup" nominal (define horizonte)
  double tfac;      // tiempo de facing

  // === State ===
  flair::core::Vector3Df start_pos, end_pos, des_pos;

  // Target state
  flair::core::Vector3Df targetPosition;
  flair::core::Vector3Df targetVelocity;  // NUEVO

  // Coeficientes
  Eigen::Matrix<double, 7, 1> coefficients;     // temporal
  Eigen::Matrix<double, 7, 1> coefficients_z;
  Eigen::Matrix<double, 6, 1> coefficients_x;
  Eigen::Matrix<double, 6, 1> coefficients_y;
  Eigen::Matrix<double, 6, 1> coefficients_yaw;

  double vx_linear;
  double x_offset;
  double vy_linear;
  double y_offset;

  // === Yaw ===
  float computed_yaw;
  bool yaw_frozen;
  float frozen_yaw;
  float targetYaw;
  float yaw0;

  double replan_min_dt;       // s, mínimo entre replans
  double time_since_replan;   // s, acumulado
  double delta_pred_thr;      // m, umbral de cambio del objetivo predicho
  double r_capture;           // m, radio de captura (hook)
  double r_freeze;            // m, radio para congelar replanning cerca del target
  bool captured;
  bool have_last_pred;
  flair::core::Vector3Df last_pred;
};

#endif // TRAJECTORYGENERATOR6_IMPL_H
