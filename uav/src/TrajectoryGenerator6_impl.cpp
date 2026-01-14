// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2025/11/03
//  filename:   TrajectoryGenerator6_impl.cpp
//
//  author:     Custom Implementation - 6th Grade Polynomial Trajectory
//
//  purpose:    Class generating a 6th grade polynomial trajectory in 3D
//              with intermediate waypoint constraint
//              + dynamic replanning for moving target
//

#include "TrajectoryGenerator6_impl.h"
#include "TrajectoryGenerator6.h"
#include <Matrix.h>
#include <Layout.h>
#include <GroupBox.h>
#include <DoubleSpinBox.h>
#include <cmath>
#include <iostream>
#include <algorithm> // std::max/min

using std::string;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::filter;

static inline double clampd(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}
static inline double wrapToPi(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

TrajectoryGenerator6_impl::TrajectoryGenerator6_impl(
    TrajectoryGenerator6 *self, const LayoutPosition *position,
    string name) {
  first_update = true;
  is_running = false;
  is_finishing = false;

  // yaw
  yaw_frozen = false;
  frozen_yaw = 0.0f;
  computed_yaw = 0.0f;
  yaw0 = 0.0f;
  targetYaw = 0.0f;

  // target vel (nuevo)
  targetVelocity = Vector3Df(0,0,0);

  // replanning params (nuevo)
  replan_min_dt = 0.10;       // 10 Hz
  time_since_replan = 0.0;
  delta_pred_thr = 0.08;      // 8 cm
  r_capture = 0.15;           // hook capture
  r_freeze  = 0.20;           // freeze replanning near
  captured = false;
  have_last_pred = false;
  last_pred = Vector3Df(0,0,0);

  // init UI
  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",
                        0, 1, 0.01);

  tobj_ui = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Object pick time (tobj)",
                               " s", 0, 100, 0.5, 1);

  Gripper = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(), "Offset from the objective in Z",
                               "m", 0, 3, 0.1, 1);

  // output matrix - 4 rows x 3 cols
  MatrixDescriptor *desc = new MatrixDescriptor(4, 3);
  desc->SetElementName(0, 0, "pos.x");
  desc->SetElementName(0, 1, "pos.y");
  desc->SetElementName(0, 2, "pos.z");

  desc->SetElementName(1, 0, "vel.x");
  desc->SetElementName(1, 1, "vel.y");
  desc->SetElementName(1, 2, "vel.z");

  desc->SetElementName(2, 0, "acc.x");
  desc->SetElementName(2, 1, "acc.y");
  desc->SetElementName(2, 2, "acc.z");

  desc->SetElementName(3, 0, "yaw");
  desc->SetElementName(3, 1, "yaw_rate");
  desc->SetElementName(3, 2, "yaw_acc");

  output = new Matrix(self, desc, floatType, name);
  delete desc;
}

TrajectoryGenerator6_impl::~TrajectoryGenerator6_impl() {
  delete output;
}

// ===================== COEFS =====================

Eigen::Matrix<double,6,1> TrajectoryGenerator6_impl::CalculateCoefficientsYaw(
    double yaw0, double yawTarget, double ti, double tfac) {

  Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Zero();

  A << pow(ti,5),     pow(ti,4),     pow(ti,3),    pow(ti,2),   ti,  1,
       pow(tfac,5),   pow(tfac,4),   pow(tfac,3),  pow(tfac,2), tfac,1,
       5*pow(ti,4),   4*pow(ti,3),   3*pow(ti,2),  2*ti,        1,   0,
       5*pow(tfac,4), 4*pow(tfac,3), 3*pow(tfac,2),2*tfac,      1,   0,
       20*pow(ti,3),  12*pow(ti,2),  6*ti,         2,           0,   0,
       20*pow(tfac,3),12*pow(tfac,2),6*tfac,       2,           0,   0;

  Eigen::Matrix<double,6,1> b;
  b << yaw0, yawTarget, 0, 0, 0, 0;

  Eigen::Matrix<double,6,1> coef = A.colPivHouseholderQr().solve(b);

  double denom = b.norm() + 1e-12;
  double error = (A * coef - b).norm() / denom;
  if (error > 1e-6)
      std::cerr << "Warning: Yaw solution error = " << error << std::endl;

  return coef;
}

Eigen::Matrix<double,6,1> TrajectoryGenerator6_impl::CalculateCoefficientsXY(
    double xi, double xobj, double xbef, double xf, double tfac,
    double ti, double tobj, double tf, double vi, double vf) {

  Eigen::Matrix<double,6,6> A = Eigen::Matrix<double,6,6>::Zero();

  A << pow(ti,5),   pow(ti,4),   pow(ti,3),   pow(ti,2),   ti,   1,
       pow(tfac,5), pow(tfac,4), pow(tfac,3), pow(tfac,2), tfac, 1,
       pow(tobj,5), pow(tobj,4), pow(tobj,3), pow(tobj,2), tobj, 1,
       pow(tf,5),   pow(tf,4),   pow(tf,3),   pow(tf,2),   tf,   1,
       5*pow(ti,4), 4*pow(ti,3), 3*pow(ti,2), 2*ti,        1,    0,
       5*pow(tf,4), 4*pow(tf,3), 3*pow(tf,2), 2*tf,        1,    0;

  Eigen::Matrix<double,6,1> b;
  b << xi, xbef, xobj, xf, vi, vf;

  Eigen::Matrix<double,6,1> coef = A.colPivHouseholderQr().solve(b);

  double denom = b.norm() + 1e-12;
  double error = (A * coef - b).norm() / denom;
  if (error > 1e-6)
      std::cerr << "Warning: XY solution error = " << error << std::endl;

  return coef;
}

double TrajectoryGenerator6_impl::EvaluatePosition(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double resultado = 0;
  for (int i = 0; i <= grado; i++) {
    resultado += coef(i) * pow(t, grado - i);
  }
  return resultado;
}

double TrajectoryGenerator6_impl::EvaluateVelocity(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double resultado = 0;
  for (int i = 0; i <= grado - 1; i++) {
    resultado += (grado - i) * coef(i) * pow(t, grado - i - 1);
  }
  return resultado;
}

double TrajectoryGenerator6_impl::EvaluateAcceleration(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double resultado = 0;
  for (int i = 0; i <= grado - 2; i++) {
    resultado += (grado - i - 1) * (grado - i) * coef(i) * pow(t, grado - i - 2);
  }
  return resultado;
}

void TrajectoryGenerator6_impl::CalculateCoefficientsZ(double zi, double zm, double zf,
                                                       double ti_traj, double tobj, double tf_traj) {
  Eigen::Matrix<double,7,7> A{
    {pow(ti_traj,6), pow(ti_traj,5), pow(ti_traj,4), pow(ti_traj,3), pow(ti_traj,2), ti_traj, 1},
    {pow(tobj,6),     pow(tobj,5),     pow(tobj,4),     pow(tobj,3),     pow(tobj,2),     tobj,     1},
    {6*pow(tobj,5),   5*pow(tobj,4),   4*pow(tobj,3),   3*pow(tobj,2),   2*tobj,          1,        0},
    {pow(tf_traj,6),  pow(tf_traj,5),  pow(tf_traj,4),  pow(tf_traj,3),  pow(tf_traj,2),  tf_traj,  1},
    {6*pow(ti_traj,5),5*pow(ti_traj,4),4*pow(ti_traj,3),3*pow(ti_traj,2),2*ti_traj,       1,        0},
    {6*pow(tf_traj,5),5*pow(tf_traj,4),4*pow(tf_traj,3),3*pow(tf_traj,2),2*tf_traj,       1,        0},
    {30*pow(ti_traj,4),20*pow(ti_traj,3),12*pow(ti_traj,2),6*ti_traj,2,0,0}
  };

  Eigen::Matrix<double,7,1> b{zi, zm, 0, zf, 0, 0, 0};

  coefficients = A.colPivHouseholderQr().solve(b);

  double denom = b.norm() + 1e-12;
  double error = (A * coefficients - b).norm() / denom;
  if (error > 1e-6)
      std::cerr << "Warning: Z solution error = " << error << std::endl;
}


void TrajectoryGenerator6_impl::setTargetPosition(Vector3Df posTarget, float yawTarget){
  this->targetPosition = posTarget;
  this->targetYaw = yawTarget;
}

// NUEVO
void TrajectoryGenerator6_impl::setTargetState(Vector3Df posTarget, Vector3Df velTarget, float yawTarget){
  this->targetPosition = posTarget;
  this->targetVelocity = velTarget;
  this->targetYaw = yawTarget;
}


void TrajectoryGenerator6_impl::PlanFromState(const Vector3Df &p0,
                                              const Vector3Df &v0,
                                              float yaw0_now,
                                              const Vector3Df &objPos,
                                              float objYaw) {
  // tiempos
  double tobj = tobj_ui->Value();
  double ti = 0.0;
  double tf = tobj * 2.0;
  double tf_traj = (tobj * 2.0) - (tf / 5.0);
  double ti_traj = tf / 5.0;
  double tfac = tobj * 0.8;

  this->tf = tf;
  this->ti_traj = ti_traj;
  this->tf_traj = tf_traj;
  this->tobj = tobj;
  this->tfac = tfac;

  // target
  double gripper = Gripper->Value();
  double xobj = objPos.x;
  double yobj = objPos.y;
  double zobj = objPos.z - gripper;

  // start
  start_pos = p0;
  des_pos = p0;
  yaw0 = yaw0_now;
  yaw_frozen = false;

  double xi = p0.x;
  double yi = p0.y;
  double zi = p0.z;
  double vxi = v0.x;
  double vyi = v0.y;

  double zf = zi;
  double zm = zobj;

  // lado relativo
  double dx_rel = xi - xobj;
  double dy_rel = yi - yobj;
  double projection = dx_rel * cos(objYaw) + dy_rel * sin(objYaw);
  double dis = std::sqrt(dx_rel*dx_rel + dy_rel*dy_rel);

  double xbef, ybef, xf, yf;
  float yaw_final;

  if (projection > 0) {
    double target_approach_x = xobj + (0.4 * dis) * cos(objYaw);
    double target_approach_y = yobj + (0.4 * dis) * sin(objYaw);

    xbef = xi + 0.80 * (target_approach_x - xi);
    ybef = yi + 0.80 * (target_approach_y - yi);

    xf = xobj - 1.5 * cos(objYaw);
    yf = yobj - 1.5 * sin(objYaw);

    yaw_final = objYaw + (float)M_PI;
  } else {
    double target_approach_x = xobj - (0.4 * dis) * cos(objYaw);
    double target_approach_y = yobj - (0.4 * dis) * sin(objYaw);

    xbef = xi + 0.80 * (target_approach_x - xi);
    ybef = yi + 0.80 * (target_approach_y - yi);

    xf = xobj + 1.5 * cos(objYaw);
    yf = yobj + 1.5 * sin(objYaw);

    yaw_final = objYaw;
  }

  // Z
  CalculateCoefficientsZ(zi, zm, zf, ti_traj, tobj, tf_traj);
  coefficients_z = coefficients;

  double vxf = v0.x;
  double vyf = v0.y;

  coefficients_x = CalculateCoefficientsXY(xi, xobj, xbef, xf, tfac, ti, tobj, tf, vxi, vxf);
  coefficients_y = CalculateCoefficientsXY(yi, yobj, ybef, yf, tfac, ti, tobj, tf, vyi, vyf);

  // yaw unwrap (camino corto)
  double dyaw = wrapToPi((double)yaw_final - (double)yaw0_now);
  double yaw_final_unwrapped = (double)yaw0_now + dyaw;

  coefficients_yaw = CalculateCoefficientsYaw((double)yaw0_now, yaw_final_unwrapped, ti, tfac);

  // reset local time of segment
  CurrentTime = 0.0f;
}

void TrajectoryGenerator6_impl::StartTraj(const Vector3Df &start,
                                          const Vector3Df &end,
                                          const Vector3Df &start_vel,
                                          float start_yaw) {
  (void)end;

  is_running = true;
  first_update = true;
  is_finishing = false;

  captured = false;
  have_last_pred = false;
  time_since_replan = 0.0;

  // primer plan hacia target actual
  PlanFromState(start, start_vel, start_yaw, targetPosition, targetYaw);
}

void TrajectoryGenerator6_impl::StartTraj(const Vector3Df &start) {
  Vector3Df dummy_end;
  Vector3Df zero_vel(0, 0, 0);
  float zero_yaw = 0.0f;
  StartTraj(start, dummy_end, zero_vel, zero_yaw);
}

void TrajectoryGenerator6_impl::FinishTraj(void) {
  is_running = false;
  is_finishing = false;
}


void TrajectoryGenerator6_impl::Update(Time time) {
  float delta_t;
  Vector3Df vel, acc;

  // init seguro
  vel.x = vel.y = vel.z = 0.0f;
  acc.x = acc.y = acc.z = 0.0f;

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

  float yaw_rate = 0.0f;
  float yaw_acc  = 0.0f;

  if (is_running) {

    time_since_replan += delta_t;

    // estado actual del plan (mejor continuidad)
    Vector3Df p0 = des_pos;
    Vector3Df v0;
    v0.x = (CurrentTime <= tf) ? (float)EvaluateVelocity(CurrentTime, coefficients_x) : 0.0f;
    v0.y = (CurrentTime <= tf) ? (float)EvaluateVelocity(CurrentTime, coefficients_y) : 0.0f;

    if (CurrentTime > ti_traj && CurrentTime <= tf_traj) {
      v0.z = (float)EvaluateVelocity(CurrentTime, coefficients_z);
    } else {
      v0.z = 0.0f;
    }

    float yaw0_now = computed_yaw;

    // distancia al target actual (hook)
    double gripper = Gripper->Value();
    double dx = targetPosition.x - p0.x;
    double dy = targetPosition.y - p0.y;
    double dz = (targetPosition.z - gripper) - p0.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    if (dist < r_capture) captured = true;
    bool freeze = (dist < r_freeze);

    // predicción con velocidad
    double v_nom = 0.7; 
    double dist_xy = std::sqrt(dx*dx + dy*dy);
    double Th = clampd(dist_xy / v_nom, 0.8, 1.8);

    Vector3Df pred = targetPosition;
    pred.x += targetVelocity.x * Th;
    pred.y += targetVelocity.y * Th;
    pred.z += targetVelocity.z * Th;

    double d_pred = 0.0;
    if (have_last_pred) {
      double ex = pred.x - last_pred.x;
      double ey = pred.y - last_pred.y;
      double ez = pred.z - last_pred.z;
      d_pred = std::sqrt(ex*ex + ey*ey + ez*ez);
    }

    if (!captured && !freeze &&
        time_since_replan >= replan_min_dt &&
        (!have_last_pred || d_pred >= delta_pred_thr)) {

      PlanFromState(p0, v0, yaw0_now, pred, targetYaw);

      last_pred = pred;
      have_last_pred = true;
      time_since_replan = 0.0;
    }


    // X,Y
    if (CurrentTime <= tf) {
      des_pos.x = (float)EvaluatePosition(CurrentTime, coefficients_x);
      vel.x     = (float)EvaluateVelocity(CurrentTime, coefficients_x);
      acc.x     = (float)EvaluateAcceleration(CurrentTime, coefficients_x);

      des_pos.y = (float)EvaluatePosition(CurrentTime, coefficients_y);
      vel.y     = (float)EvaluateVelocity(CurrentTime, coefficients_y);
      acc.y     = (float)EvaluateAcceleration(CurrentTime, coefficients_y);
    } else {
      vel.x = acc.x = 0.0f;
      vel.y = acc.y = 0.0f;
    }

    // Z
    if (CurrentTime <= ti_traj) {
      des_pos.z = start_pos.z;
      vel.z = 0.0f;
      acc.z = 0.0f;
    } else if (CurrentTime <= tf_traj) {
      des_pos.z = (float)EvaluatePosition(CurrentTime, coefficients_z);
      vel.z     = (float)EvaluateVelocity(CurrentTime, coefficients_z);
      acc.z     = (float)EvaluateAcceleration(CurrentTime, coefficients_z);
    } else {
      des_pos.z = start_pos.z;
      vel.z = 0.0f;
      acc.z = 0.0f;
    }

    // Yaw
    if (CurrentTime <= tfac) {
      computed_yaw = (float)EvaluatePosition(CurrentTime, coefficients_yaw);
      yaw_rate     = (float)EvaluateVelocity(CurrentTime, coefficients_yaw);
      yaw_acc      = (float)EvaluateAcceleration(CurrentTime, coefficients_yaw);
    } else {
      if (!yaw_frozen) {
        yaw_frozen = true;
        frozen_yaw = (float)EvaluatePosition(tfac, coefficients_yaw);
      }
      computed_yaw = frozen_yaw;
      yaw_rate = 0.0f;
      yaw_acc  = 0.0f;
    }


  } else {
    vel.x = vel.y = vel.z = 0.0f;
    acc.x = acc.y = acc.z = 0.0f;
    yaw_rate = 0.0f;
    yaw_acc  = 0.0f;
  }

  // output matrix
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

  output->SetValueNoMutex(3, 0, computed_yaw);
  output->SetValueNoMutex(3, 1, yaw_rate);
  output->SetValueNoMutex(3, 2, yaw_acc);
  output->ReleaseMutex();

  output->SetDataTime(time);
}

float TrajectoryGenerator6_impl::GetYaw(void) const {
  return computed_yaw;
}
