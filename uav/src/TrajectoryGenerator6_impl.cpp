// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
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

// ===============================
// Quintic spline helper (1D)
// ===============================
static inline void poly_row(double tau, double row[6]) {
  row[0]=1; row[1]=tau; row[2]=tau*tau; row[3]=tau*tau*tau;
  row[4]=row[3]*tau; row[5]=row[4]*tau;
}
static inline void d1_row(double tau, double row[6]) {
  row[0]=0; row[1]=1; row[2]=2*tau; row[3]=3*tau*tau;
  row[4]=4*tau*tau*tau; row[5]=5*tau*tau*tau*tau;
}
static inline void d2_row(double tau, double row[6]) {
  row[0]=0; row[1]=0; row[2]=2; row[3]=6*tau;
  row[4]=12*tau*tau; row[5]=20*tau*tau*tau;
}
static inline void d3_row(double tau, double row[6]) {
  row[0]=0; row[1]=0; row[2]=0; row[3]=6;
  row[4]=24*tau; row[5]=60*tau*tau;
}
static inline void d4_row(double tau, double row[6]) {
  row[0]=0; row[1]=0; row[2]=0; row[3]=0;
  row[4]=24; row[5]=120*tau;
}

static inline double eval_poly0(const Eigen::Matrix<double,6,1>& c, double tau){
  double t2=tau*tau, t3=t2*tau, t4=t3*tau, t5=t4*tau;
  return c(0) + c(1)*tau + c(2)*t2 + c(3)*t3 + c(4)*t4 + c(5)*t5;
}
static inline double eval_poly1(const Eigen::Matrix<double,6,1>& c, double tau){
  double t2=tau*tau, t3=t2*tau, t4=t3*tau;
  return c(1) + 2*c(2)*tau + 3*c(3)*t2 + 4*c(4)*t3 + 5*c(5)*t4;
}
static inline double eval_poly2(const Eigen::Matrix<double,6,1>& c, double tau){
  double t2=tau*tau, t3=t2*tau;
  return 2*c(2) + 6*c(3)*tau + 12*c(4)*t2 + 20*c(5)*t3;
}

static inline double dist3(const Vector3Df& a, const Vector3Df& b){
  double dx = (double)b.x - (double)a.x;
  double dy = (double)b.y - (double)a.y;
  double dz = (double)b.z - (double)a.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}


TrajectoryGenerator6_impl::TrajectoryGenerator6_impl(
    TrajectoryGenerator6 *self, const LayoutPosition *position,
    string name) {

  first_update = true;
  is_running = false;
  is_finishing = false;

  yaw_frozen = false;
  frozen_yaw = 0.0f;
  computed_yaw = 0.0f;
  yaw0 = 0.0f;
  targetYaw = 0.0f;

  // init UI
  GroupBox *reglages_groupbox = new GroupBox(position, name);
  T = new DoubleSpinBox(reglages_groupbox->NewRow(), "period, 0 for auto", " s",
                        0, 1, 0.01);

  tobj_ui = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),
                             "Object pick time (tobj)", " s",
                             0, 100, 0.5, 1);

  Gripper = new DoubleSpinBox(reglages_groupbox->LastRowLastCol(),
                             "Offset from the objective in Z", "m",
                             0, 3, 0.1, 1);

  // output matrix
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

// ===============================
// Build spline C4 (1D) 
// ===============================
TrajectoryGenerator6_impl::QuinticSpline1D
TrajectoryGenerator6_impl::BuildSplineC4_1D(const std::vector<double>& P,
                                            const std::vector<double>& Tseg,
                                            double v0, double a0,
                                            double vf, double af)
{
  const int K = (int)P.size();      // points
  const int S = K - 1;              // segments
  const int N = 6 * S;              // unknowns

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(N);

  auto add_row_seg = [&](int r, int seg, const double row6[6], double rhs){
    int c0 = 6 * seg;
    for (int j=0;j<6;j++) A(r, c0+j) = row6[j];
    b(r) = rhs;
  };

  int r = 0;

  // (1) p_i(0)=P_i
  for(int i=0;i<S;i++){
    double row[6]; poly_row(0.0, row);
    add_row_seg(r++, i, row, P[i]);
  }

  // (2) p_i(T_i)=P_{i+1}
  for(int i=0;i<S;i++){
    double row[6]; poly_row(Tseg[i], row);
    add_row_seg(r++, i, row, P[i+1]);
  }

  // (3) C4 continuity at internal knots: v,a,jerk,snap
  for(int k=1;k<=K-2;k++){
    int L = k-1;
    int R = k;
    double TL = Tseg[L];

    auto add_cont = [&](void (*drow)(double,double*)){
      double rowL[6], rowR[6];
      drow(TL, rowL);
      drow(0.0, rowR);
      int cL = 6*L, cR = 6*R;
      for(int j=0;j<6;j++){
        A(r, cL+j) =  rowL[j];
        A(r, cR+j) = -rowR[j];
      }
      b(r) = 0.0;
      r++;
    };

    add_cont(d1_row);
    add_cont(d2_row);
    add_cont(d3_row);
    add_cont(d4_row);
  }

  // (4) start boundary: v(0)=v0, a(0)=a0
  {
    double row[6]; d1_row(0.0, row);
    add_row_seg(r++, 0, row, v0);
  }
  {
    double row[6]; d2_row(0.0, row);
    add_row_seg(r++, 0, row, a0);
  }

  // (5) end boundary: v(T)=vf, a(T)=af
  {
    double row[6]; d1_row(Tseg[S-1], row);
    add_row_seg(r++, S-1, row, vf);
  }
  {
    double row[6]; d2_row(Tseg[S-1], row);
    add_row_seg(r++, S-1, row, af);
  }

  Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

  QuinticSpline1D spl;
  spl.T = Tseg;
  spl.c.resize(S);
  for(int i=0;i<S;i++){
    spl.c[i] = x.segment<6>(6*i);
  }
  return spl;
}

void TrajectoryGenerator6_impl::EvalSpline1D(const QuinticSpline1D& spl, double t,
                                            double &p, double &v, double &a) const
{
  double accT = 0.0;
  int seg = (int)spl.T.size() - 1;
  for(int i=0;i<(int)spl.T.size();i++){
    if(t <= accT + spl.T[i]) { seg = i; break; }
    accT += spl.T[i];
  }
  double tau = t - accT;
  if(tau < 0) tau = 0;
  if(tau > spl.T[seg]) tau = spl.T[seg];

  const auto& c = spl.c[seg];
  p = eval_poly0(c, tau);
  v = eval_poly1(c, tau);
  a = eval_poly2(c, tau);
}

// ===============================
// Z polynomial 
// ===============================
void TrajectoryGenerator6_impl::CalculateCoefficientsZ(double zi, double zm, double zf,
                                                       double ti_traj, double tobj, double tf_traj) {
  // Z(t) = a*t^6 + b*t^5 + c*t^4 + d*t^3 + e*t^2 + f*t + h
  Eigen::Matrix<double,7,7> A{
    {pow(ti_traj,6), pow(ti_traj,5), pow(ti_traj,4), pow(ti_traj,3), pow(ti_traj,2), ti_traj, 1},
    {pow(tobj,6),    pow(tobj,5),    pow(tobj,4),    pow(tobj,3),    pow(tobj,2),    tobj,    1},
    {6*pow(tobj,5),  5*pow(tobj,4),  4*pow(tobj,3),  3*pow(tobj,2),  2*tobj,         1,       0},
    {pow(tf_traj,6), pow(tf_traj,5), pow(tf_traj,4), pow(tf_traj,3), pow(tf_traj,2), tf_traj, 1},
    {6*pow(ti_traj,5),5*pow(ti_traj,4),4*pow(ti_traj,3),3*pow(ti_traj,2),2*ti_traj,  1,       0},
    {6*pow(tf_traj,5),5*pow(tf_traj,4),4*pow(tf_traj,3),3*pow(tf_traj,2),2*tf_traj,  1,       0},
    {30*pow(ti_traj,4),20*pow(ti_traj,3),12*pow(ti_traj,2),6*ti_traj,2,0,0}
  };

  Eigen::Matrix<double,7,1> b{zi, zm, 0, zf, 0, 0, 0};

  coefficients = A.colPivHouseholderQr().solve(b);

  double error = (A * coefficients - b).norm() / (b.norm() + 1e-12);
  if (error > 1e-6)
      std::cerr << "Warning: Z solution error = " << error << std::endl;

  coefficients_z = coefficients;
}

// ===============================
// yaw quintic 
// ===============================
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

  double error = (A * coef - b).norm() / (b.norm() + 1e-12);
  if (error > 1e-6)
      std::cerr << "Warning: Yaw solution error = " << error << std::endl;

  return coef;
}

// polynomial eval helpers (used for yaw AND for Z poly)
double TrajectoryGenerator6_impl::EvaluatePosition(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double res = 0;
  for (int i = 0; i <= grado; i++) res += coef(i) * pow(t, grado - i);
  return res;
}
double TrajectoryGenerator6_impl::EvaluateVelocity(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double res = 0;
  for (int i = 0; i <= grado - 1; i++) res += (grado - i) * coef(i) * pow(t, grado - i - 1);
  return res;
}
double TrajectoryGenerator6_impl::EvaluateAcceleration(double t, const Eigen::VectorXd& coef) {
  int grado = (int)coef.size() - 1;
  double res = 0;
  for (int i = 0; i <= grado - 2; i++) res += (grado - i - 1) * (grado - i) * coef(i) * pow(t, grado - i - 2);
  return res;
}

// ===============================
// StartTraj
// ===============================
void TrajectoryGenerator6_impl::StartTraj(const Vector3Df &start,
                                          const Vector3Df &end,
                                          const Vector3Df &start_vel,
                                          float start_yaw) {
  is_running = true;
  first_update = true;
  is_finishing = false;
  yaw_frozen = false;

  start_pos = start;
  des_pos = start;
  CurrentTime = 0;
  yaw0 = start_yaw;

  // target with z-offset
  const double gripper = Gripper->Value();
  const double xobj = targetPosition.x;
  const double yobj = targetPosition.y;
  const double zobj = targetPosition.z - gripper;

  const double xi = start_pos.x;
  const double yi = start_pos.y;
  const double zi = start_pos.z;

  // determine facing/exit and yaw_final 
  const double dx_rel = xi - xobj;
  const double dy_rel = yi - yobj;
  const double projection = dx_rel * std::cos(targetYaw) + dy_rel * std::sin(targetYaw);
  const double dis = std::sqrt(dx_rel*dx_rel + dy_rel*dy_rel);

  double xbef, ybef, xf, yf;
  float yaw_final;

  if (projection > 0) {
    const double target_approach_x = xobj + (0.4 * dis) * std::cos(targetYaw);
    const double target_approach_y = yobj + (0.4 * dis) * std::sin(targetYaw);
    xbef = xi + 0.80 * (target_approach_x - xi);
    ybef = yi + 0.80 * (target_approach_y - yi);
    xf = xobj - 1.5 * std::cos(targetYaw);
    yf = yobj - 1.5 * std::sin(targetYaw);
    yaw_final = targetYaw + (float)M_PI;
  } else {
    const double target_approach_x = xobj - (0.4 * dis) * std::cos(targetYaw);
    const double target_approach_y = yobj - (0.4 * dis) * std::sin(targetYaw);
    xbef = xi + 0.80 * (target_approach_x - xi);
    ybef = yi + 0.80 * (target_approach_y - yi);
    xf = xobj + 1.5 * std::cos(targetYaw);
    yf = yobj + 1.5 * std::sin(targetYaw);
    yaw_final = targetYaw;
  }

  // 4 points (XY spline uses these, Z poly uses zi/zm/zf separately)
  P0 = start_pos;
  P1 = Vector3Df((float)xbef, (float)ybef, start_pos.z); // facing (z held)
  P2 = Vector3Df((float)xobj, (float)yobj, (float)zobj); // capture (z lowered)
  P3 = Vector3Df((float)xf,   (float)yf,   start_pos.z); // exit (z back)

  const double Ttot = 2.0 * tobj_ui->Value();

  const double L0 = dist3(P0, P1);
  const double L1 = dist3(P1, P2);
  const double L2 = dist3(P2, P3);
  const double Lsum = L0 + L1 + L2 + 1e-9;

  const double Tmin = 0.30;
  T0 = std::max(Tmin, Ttot * (L0 / Lsum));
  T1 = std::max(Tmin, Ttot * (L1 / Lsum));
  T2 = std::max(Tmin, Ttot * (L2 / Lsum));

  double Tsum = T0 + T1 + T2;
  double scale = (Tsum > 1e-9) ? (Ttot / Tsum) : 1.0;
  T0 *= scale; T1 *= scale; T2 *= scale;

  tf = T0 + T1 + T2;

  // build XY splines 
  std::vector<double> Tseg = {T0, T1, T2};
  std::vector<double> X = {P0.x, P1.x, P2.x, P3.x};
  std::vector<double> Y = {P0.y, P1.y, P2.y, P3.y};

  sx = BuildSplineC4_1D(X, Tseg, start_vel.x, 0.0, 0.0, 0.0);
  sy = BuildSplineC4_1D(Y, Tseg, start_vel.y, 0.0, 0.0, 0.0);


  const double tobj_local = tobj_ui->Value();
  const double tf_local   = 2.0 * tobj_local;
  const double ti_traj_local = tf_local / 5.0;
  const double tf_traj_local = tf_local - (tf_local / 5.0);

  tobj   = tobj_local;
  ti_traj = ti_traj_local;
  tf_traj = tf_traj_local;

  const double zm = zobj;
  const double zf = zi;
  CalculateCoefficientsZ(zi, zm, zf, ti_traj, tobj, tf_traj);

  // ---- yaw timing 
  tfac = T0;         // yaw finishes at facing point
  {
    auto wrapToPi = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
    double dyaw = wrapToPi((double)yaw_final - (double)start_yaw);
    double yaw_final_unwrapped = (double)start_yaw + dyaw;
    coefficients_yaw = CalculateCoefficientsYaw((double)start_yaw, yaw_final_unwrapped, 0.0, tfac);
  }
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

// ===============================
// Update
// ===============================
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

  float yaw_rate = 0.0f;
  float yaw_acc  = 0.0f;

  if (is_running) {
    // clamp for XY spline
    double txy = (double)CurrentTime;
    if (txy < 0) txy = 0;
    if (txy > tf) txy = tf;

    // ---- XY from spline
    double px,vx,ax;
    double py,vy,ay;
    EvalSpline1D(sx, txy, px, vx, ax);
    EvalSpline1D(sy, txy, py, vy, ay);

    des_pos.x = (float)px; vel.x = (float)vx; acc.x = (float)ax;
    des_pos.y = (float)py; vel.y = (float)vy; acc.y = (float)ay;

    // ---- Z from polynomial 
    const double t = (double)CurrentTime;

    if (t <= ti_traj) {
      des_pos.z = start_pos.z;
      vel.z = 0.0f;
      acc.z = 0.0f;
    } else if (t <= tf_traj) {
      des_pos.z = (float)EvaluatePosition(t, coefficients_z);
      vel.z     = (float)EvaluateVelocity(t, coefficients_z);
      acc.z     = (float)EvaluateAcceleration(t, coefficients_z);
    } else {
      des_pos.z = start_pos.z;
      vel.z = 0.0f;
      acc.z = 0.0f;
    }

    // ---- Yaw 
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

    if (CurrentTime >= tf) {
      is_running = false;

      // enforce final XY constraints
      des_pos.x = P3.x;
      des_pos.y = P3.y;
      vel.x = vel.y = 0.0f;
      acc.x = acc.y = 0.0f;

      // Z: keep what your original would give at the end
      des_pos.z = start_pos.z;
      vel.z = 0.0f;
      acc.z = 0.0f;

      yaw_rate = 0.0f;
      yaw_acc  = 0.0f;
    }
  } else {
    vel.x = vel.y = vel.z = 0.0f;
    acc.x = acc.y = acc.z = 0.0f;
    yaw_rate = 0.0f;
    yaw_acc  = 0.0f;
  }

  // output
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

void TrajectoryGenerator6_impl::setTargetPosition(Vector3Df posTarget, float yawTarget){
  this->targetPosition = posTarget;
  this->targetYaw = yawTarget;
}
