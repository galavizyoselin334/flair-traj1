// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
/*!
 * \file TrajectoryGenerator6_impl.h
 * \brief Class generating a 6th grade polynomial trajectory in 3D
 * \author Custom Implementation - Based on Python sim_quad6.py
 * \date 2025/11/05
 * \version 2.0
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
  TrajectoryGenerator6_impl(
      flair::filter::TrajectoryGenerator6 *self,
      const flair::gui::LayoutPosition *position, std::string name);
  ~TrajectoryGenerator6_impl();
  
  void Update(flair::core::Time time);
  void StartTraj(const flair::core::Vector3Df &start_pos, 
                 const flair::core::Vector3Df &end_pos,
                const flair::core::Vector3Df &start_vel);
  void StartTraj(const flair::core::Vector3Df &start_pos);
  void FinishTraj(void);
  void setTargetPosition(flair::core::Vector3Df posTarget);
    //metodo para obtener yaw 
  float GetYaw(void) const; 
  bool is_running;
  flair::core::Matrix *output;
  
  // UI controls - trajectory timing 
  flair::gui::DoubleSpinBox *T;
  flair::gui::DoubleSpinBox *tobj_ui;     // Time to pick object 
  flair::gui::DoubleSpinBox *Gripper;     // Gripper length
  flair::gui::DoubleSpinBox *Xf;    // Posicion final en x 
  flair::gui::DoubleSpinBox *Yf;   // Posicion final en y


private:
  // Métodos para calcular y evaluar el polinomio de 6to grado
  void CalculateCoefficientsZ(double zi, double zm, double zf,
                              double ti_traj, double tobj, double tf_traj);
  double EvaluatePosition(double t, const Eigen::Matrix<double, 7, 1>& coef);
  double EvaluateVelocity(double t, const Eigen::Matrix<double, 7, 1>& coef);
  double EvaluateAcceleration(double t, const Eigen::Matrix<double, 7, 1>& coef);
  //metodos para calcular y evaluar el polinomio de 3 grado para xy 
  Eigen::Matrix<double,5,1> CalculateCoefficientsXY(double xi, double xobj, double xf,
                                                  double ti, double tobj, double tf, double vi);
  double EvaluatePositionXY(double t, const Eigen::Matrix<double, 5, 1>& coef);
  double EvaluateVelocityXY(double t, const Eigen::Matrix<double, 5, 1>& coef);
  double EvaluateAccelerationXY(double t, const Eigen::Matrix<double, 5, 1>& coef);

  // Variables de tiempo
  flair::core::Time previous_time;
  float CurrentTime;
  bool first_update, is_finishing;
  
  // Trajectory timing parameters
  double ti_traj;   // inicia la trayectoria en z
  double tf; // tiempo final de la simulacion
  double tf_traj;   // termina la trayectoria en z
  double tobj;      // tiempo en el que se va a llegar al objeto
  
  // Posiciones
  flair::core::Vector3Df start_pos, end_pos, des_pos, targetPosition;
  
  // Coeficientes del polinomio de 6to grado para Z
  // Forma: Z(t) = a*t^6 + b*t^5 + c*t^4 + d*t^3 + e*t^2 + f*t + h
  Eigen::Matrix<double, 7, 1> coefficients;     // Temporal coefficients
  Eigen::Matrix<double, 7, 1> coefficients_z;   // Z axis coefficients
  Eigen::Matrix<double, 5,1> coefficients_x;
  Eigen::Matrix<double, 5,1> coefficients_y;
  
  // Parámetros de X lineal
  double vx_linear;  // Velocidad constante en X
  double x_offset;   // Offset inicial de X
  double vy_linear; 
  double y_offset; 
  
  //para la orientacion 
  float computed_yaw;         // Yaw calculado hacia el target
  bool yaw_frozen;            // Flag para congelar yaw después de tpick
  float frozen_yaw;           // Último yaw antes de tpick
};

#endif // TRAJECTORYGENERATOR6_IMPL_H