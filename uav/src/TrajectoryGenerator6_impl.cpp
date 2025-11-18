// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2025/11/03
//  filename:   TrajectoryGenerator6_impl.cpp
//
//  author:     Custom Implementation - 6th Grade Polynomial Trajectory
//    
//
//  version:    $Id: $
//
//  purpose:    Class generating a 6th grade polynomial trajectory in 3D
//              with intermediate waypoint constraint
//

#include "TrajectoryGenerator6_impl.h"
#include "TrajectoryGenerator6.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <cmath>
#include <iostream>

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

TrajectoryGenerator6_impl::TrajectoryGenerator6_impl(
    TrajectoryGenerator6 *self, const LayoutPosition *position,
    string name) {
  first_update = true;
  is_running = false;
  is_finishing = false;

  // init UI
  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",
                        0, 1, 0.01);
  //tiempo para recoger el objeto
  tobj_ui = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Object pick time (tobj)",
                               " s", 0, 100, 1, 1);

  // Target position (object position)  
  /*
  target_x = new DoubleSpinBox(reglages_groupbox->NewRow(), "target X (xobj)",
                               " m", -10, 10, 0.1, 1, 0);
  target_y = new DoubleSpinBox(reglages_groupbox->NewRow(), "target Y (xobj)",
                               " m", -10, 10, 0.1, 1, 0);                  
  target_z = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "target Z (zobj)",
                               " m", -2, 2, 0.1, 1, 0);
  */
  // init matrix - 3 rows (position, velocity, acceleration) x 3 cols (x, y, z)
  MatrixDescriptor *desc = new MatrixDescriptor(3, 3);
  desc->SetElementName(0, 0, "pos.x");
  desc->SetElementName(0, 1, "pos.y");
  desc->SetElementName(0, 2, "pos.z");
  desc->SetElementName(1, 0, "vel.x");
  desc->SetElementName(1, 1, "vel.y");
  desc->SetElementName(1, 2, "vel.z");
  desc->SetElementName(2, 0, "acc.x");
  desc->SetElementName(2, 1, "acc.y");
  desc->SetElementName(2, 2, "acc.z");
  output = new Matrix(self, desc, floatType, name);
  delete desc;
}

TrajectoryGenerator6_impl::~TrajectoryGenerator6_impl() {
  delete output;
}

// Función para calcular coeficientes de polinomio de 4er grado
// boundary conditions
// x(ti) = Xi, X'(ti) = 0
// x(tobj) = xobj, X(tf) = xf, X´(tf)=0
Eigen::Matrix<double,5,1> TrajectoryGenerator6_impl::CalculateCoefficientsXY(double xi, double xobj, double xf,
                                                                             double ti, double tobj, double tf, double vi) {
  Eigen::Matrix<double,5,5> A = Eigen::Matrix<double,5,5>::Zero();
   
  A << pow(ti,4),   pow(ti,3),   pow(ti,2),   ti,   1,
       pow(tobj,4),   pow(tobj,3),   pow(tobj,2),   tobj,   1,
       pow(tf,4),   pow(tf,3),   pow(tf,2),   tf,   1,
       4*pow(ti,3), 3*pow(ti,2),  2*ti,  1,   0,
       4*pow(tf,3), 3*pow(tf,2),  2*tf,  1,   0;
  Eigen::Matrix<double,5,1> b;
  b << xi, xobj, xf, vi, 0;

  Eigen::Matrix<double,5,1> coef = A.colPivHouseholderQr().solve(b);

  double error = (A * coef - b).norm() / b.norm();
  if (error > 1e-6)
      std::cerr << "Warning: Solution error = " << error << std::endl;

  return coef;
}

// Evaluar posicion en el polinomio de 3ER grado
// X(t) = a*t^4 + b*t^3 + c*t² + d*t + e
double TrajectoryGenerator6_impl::EvaluatePositionXY(double t, 
                                                    const Eigen::Matrix<double, 5, 1>& coef) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t; 
  
  return coef(0) * t4 +    // a*t^4
         coef(1) * t3 +    // b*t^3
         coef(2) * t2 +     // c*t²
         coef(3) * t  +      //d*t
         coef(4);         //e 
}

// Evaluar velocidad (derivada del polinomio)

double TrajectoryGenerator6_impl::EvaluateVelocityXY(double t,
                                                    const Eigen::Matrix<double, 5, 1>& coef) {
  double t2 = t * t;
  double t3 = t2 * t;

  return 4 * coef(0) * t3 +
         3 * coef(1) * t2 +
         2 * coef(2) * t +
         coef (3);
}

// Evaluar aceleración (segunda derivada del polinomio)
// X''(t) = 6*a*t + 2*b
double TrajectoryGenerator6_impl::EvaluateAccelerationXY(double t,
                                                        const Eigen::Matrix<double, 5, 1>& coef) {
  double t2 = t * t;
  return 12 * coef(0) * t2 +
         6 * coef(1)  * t +
         2 * coef(2);
}


// Función para calcular coeficientes de polinomio de 6to grado
// boundary conditions
// Z(ti_traj) = zi, Z'(ti_traj) = 0, Z''(ti_traj) = 0
// Z(tobj) = zm, Z'(tobj) = 0
// Z(tf_traj) = zf, Z'(tf_traj) = 0
void TrajectoryGenerator6_impl::CalculateCoefficientsZ(double zi, double zm, double zf,
                                                       double ti_traj, double tobj, double tf_traj) {
  // Z(t) = a*t^6 + b*t^5 + c*t^4 + d*t^3 + e*t^2 + f*t + h
  Eigen::Matrix<double,7,7> A{
    {pow(ti_traj,6), pow(ti_traj,5), pow(ti_traj,4), pow(ti_traj,3), pow(ti_traj,2), ti_traj, 1},       // Z(ti)=zi
    {pow(tobj,6),     pow(tobj,5),     pow(tobj,4),     pow(tobj,3),     pow(tobj,2),     tobj,     1}, // Z(tobj)=zm
    {6*pow(tobj,5),   5*pow(tobj,4),   4*pow(tobj,3),   3*pow(tobj,2),   2*tobj,          1,        0}, // Z'(tobj)=0
    {pow(tf_traj,6),  pow(tf_traj,5),  pow(tf_traj,4),  pow(tf_traj,3),  pow(tf_traj,2),  tf_traj,  1}, // Z(tf)=zf
    {6*pow(ti_traj,5),5*pow(ti_traj,4),4*pow(ti_traj,3),3*pow(ti_traj,2),2*ti_traj,       1,        0}, // Z'(ti)=0
    {6*pow(tf_traj,5),5*pow(tf_traj,4),4*pow(tf_traj,3),3*pow(tf_traj,2),2*tf_traj,       1,        0}, // Z'(tf)=0
    {30*pow(ti_traj,4),20*pow(ti_traj,3),12*pow(ti_traj,2),6*ti_traj,2,0,0}                              // Z''(ti)=0
  };

  Eigen::Matrix<double,7,1> b{zi, zm, 0, zf, 0, 0, 0};

  coefficients = A.colPivHouseholderQr().solve(b);

  double error = (A * coefficients - b).norm() / b.norm();
  if (error > 1e-6)
      std::cerr << "Warning: Solution error = " << error << std::endl;
}

// Evaluar posicion en el polinomio de 6to grado
// Z(t) = a*t^6 + b*t^5 + c*t^4 + d*t^3 + e*t^2 + f*t + h
double TrajectoryGenerator6_impl::EvaluatePosition(double t, 
                                                    const Eigen::Matrix<double, 7, 1>& coef) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  double t6 = t5 * t;
  
  return coef(0) * t6 +    // a*t^6
         coef(1) * t5 +    // b*t^5
         coef(2) * t4 +    // c*t^4
         coef(3) * t3 +    // d*t^3
         coef(4) * t2 +    // e*t^2
         coef(5) * t +     // f*t
         coef(6);          // h
}

// Evaluar velocidad (derivada del polinomio)
// Z'(t) = 6*a*t^5 + 5*b*t^4 + 4*c*t^3 + 3*d*t^2 + 2*e*t + f
double TrajectoryGenerator6_impl::EvaluateVelocity(double t,
                                                    const Eigen::Matrix<double, 7, 1>& coef) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  double t5 = t4 * t;
  
  return 6 * coef(0) * t5 +
         5 * coef(1) * t4 +
         4 * coef(2) * t3 +
         3 * coef(3) * t2 +
         2 * coef(4) * t +
         coef(5);
}

// Evaluar aceleración (segunda derivada del polinomio)
// Z''(t) = 30*a*t^4 + 20*b*t^3 + 12*c*t^2 + 6*d*t + 2*e
double TrajectoryGenerator6_impl::EvaluateAcceleration(double t,
                                                        const Eigen::Matrix<double, 7, 1>& coef) {
  double t2 = t * t;
  double t3 = t2 * t;
  double t4 = t3 * t;
  
  return 30 * coef(0) * t4 +
         20 * coef(1) * t3 +
         12 * coef(2) * t2 +
         6 * coef(3) * t +
         2 * coef(4);
}

void TrajectoryGenerator6_impl::StartTraj(const Vector3Df &start,
                                          const Vector3Df &end,
                                          const Vector3Df &start_vel) {
  is_running = true;
  first_update = true;
  is_finishing = false;

  start_pos = start;
  des_pos = start;
  CurrentTime = 0;
  
  // Obtener parámetros de UI
  double tobj = tobj_ui->Value();
  double ti=0; //tiempo inicial de la simulacion 
  double tf=tobj*2;  //variable auxiliar para definir lo demas
  double tf_traj = (tobj*2)-(tf/5);
  double ti_traj = tf/5;
  //estos luego los voy a cambiar a que sean los del ninja
  double xobj = targetPosition.x; //signo negativo porque en la simulacion esta al reves 
  double yobj = targetPosition.y;
  double zobj = targetPosition.z - 0.3;
  //double xobj = target_x->Value();
  //double yobj = target_y->Value();
  //double zobj = target_z->Value();

  // Calcular posiciones clave
  double xi = start_pos.x;
  double zi = start_pos.z;
  double yi = start_pos.y; 
  double vxi = start_vel.x;
  double vyi = start_vel.y;
  double zf = zi;  // Volver a la altura inicial
  double zm = zobj;  // Altura del objeto + largo del gripper
  //la posicion final de ambos tiene q ser el doble de la posicion del objeto porq quiero que la trayectoria sea 
  //simetrica 
  double xf = 2 * xobj - xi; 
  double yf = 2 * yobj - yi; 
 
  // Para Z: polinomio de 6to grado
  CalculateCoefficientsZ(zi, zm, zf, ti_traj, tobj, tf_traj);
  coefficients_z = coefficients;
  //para XY: polinomioo de 3er grado 
  coefficients_x = CalculateCoefficientsXY(xi, xobj, xf, ti, tobj, tf, vxi);
  coefficients_y = CalculateCoefficientsXY(yi, yobj, yf, ti, tobj, tf, vyi);
  
  // Guardar tiempos*
  this->ti_traj = ti_traj;
  this->tf_traj = tf_traj;
  this->tobj = tobj;
}

void TrajectoryGenerator6_impl::StartTraj(const Vector3Df &start) {
  Vector3Df dummy_end;  
  Vector3Df zero_vel(0, 0, 0);  // Asume velocidad inicial cero
  StartTraj(start, dummy_end, zero_vel);
}

void TrajectoryGenerator6_impl::FinishTraj(void) {
  is_running = false;
  is_finishing = false;
}

void TrajectoryGenerator6_impl::Update(Time time) {
  float delta_t;
  Vector3Df vel, acc;
  
  if (T->Value() == 0) {
    if (first_update) {
      first_update = false;
      previous_time = time;
      return;
    } else {
      delta_t = (float)(time - previous_time) / 1000000000.;
    }
  } else {
    delta_t = T->Value();
  }
  previous_time = time;
  CurrentTime += delta_t;

  if (is_running) {
    //posicion velocidad y aceleracion polinomio 3er grado
    des_pos.x = EvaluatePositionXY(CurrentTime, coefficients_x);
    vel.x = EvaluateVelocityXY(CurrentTime, coefficients_x);
    acc.x = EvaluateAccelerationXY(CurrentTime, coefficients_x);
    des_pos.y = EvaluatePositionXY(CurrentTime, coefficients_y);
    vel.y = EvaluateVelocityXY(CurrentTime, coefficients_y);
    acc.y = EvaluateAccelerationXY(CurrentTime, coefficients_y);
    
    
    // posicion en z(polinomio)
    if (CurrentTime <= ti_traj) {
      // Antes de iniciar la trayectoria Z (aqui solo avanzamos en x)
      des_pos.z = start_pos.z;
      vel.z = 0.0;
      acc.z = 0.0;
    } else if (CurrentTime <= tf_traj) {
      // Durante la trayectoria polinomial
      des_pos.z = EvaluatePosition(CurrentTime, coefficients_z);
      vel.z = EvaluateVelocity(CurrentTime, coefficients_z);
      acc.z = EvaluateAcceleration(CurrentTime, coefficients_z);
    } else {
      // Después de terminar la trayectoria
      des_pos.z = start_pos.z;  // Volver a altura inicial (zf)
      vel.z = 0.0;
      acc.z = 0.0;
    }
    
    // Verificar si terminamos completamente
    if (CurrentTime >= tf_traj + 1.0) {  // Darle 1 segundo extra
      is_running = false;
    }
    
  } else {
    // No está corriendo, mantener posición actual
    vel.x = vel.y = vel.z = 0;
    acc.x = acc.y = acc.z = 0;
  }

  // Actualizar matriz de salida
  output->GetMutex();
  output->SetValueNoMutex(0, 0, des_pos.x);
  output->SetValueNoMutex(0, 1, des_pos.y);
  output->SetValueNoMutex(0, 2, des_pos.z);
  output->SetValueNoMutex(1, 0, vel.x);
  output->SetValueNoMutex(1, 1, vel.y);
  output->SetValueNoMutex(1, 2, vel.z);
  output->SetValueNoMutex(2, 0, acc.x);
  output->SetValueNoMutex(2, 1, acc.y);
  output->SetValueNoMutex(2, 2, acc.z);
  output->ReleaseMutex();

  output->SetDataTime(time);
}

void TrajectoryGenerator6_impl::setTargetPosition(Vector3Df posTarget){
  this->targetPosition = posTarget;
}