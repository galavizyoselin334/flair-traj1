// %flair:license{
// This file is part of the Flair framework distributed under the
// CECILL-C License, Version 1.0.
// %flair:license}
//  created:    2025/11/03
//  filename:   TrajectoryGenerator6.cpp
//
//  author:     Custom Implementation
//
//  purpose:    Trajectory generator wrapper (pimpl)
//
/*********************************************************************/

#include "TrajectoryGenerator6.h"
#include "TrajectoryGenerator6_impl.h"

#include <Matrix.h>
#include <Layout.h>
#include <LayoutPosition.h>
#include <Vector3D.h>
#include <DoubleSpinBox.h>

using std::string;
using namespace flair::core;
using namespace flair::gui;

namespace flair {
namespace filter {

TrajectoryGenerator6::TrajectoryGenerator6(
    const LayoutPosition *position, string name)
    : IODevice(position->getLayout(), name) {
  pimpl_ = new TrajectoryGenerator6_impl(this, position, name);
  AddDataToLog(pimpl_->output);
  SetIsReady(true);
}

TrajectoryGenerator6::~TrajectoryGenerator6() {
  delete pimpl_;
}

bool TrajectoryGenerator6::IsRunning(void) const {
  return pimpl_->is_running;
}

Matrix *TrajectoryGenerator6::GetMatrix(void) const {
  return pimpl_->output;
}

void TrajectoryGenerator6::StartTraj(const Vector3Df &start_pos,
                                     const Vector3Df &end_pos,
                                     const Vector3Df &start_vel,
                                     float start_yaw) {
  pimpl_->StartTraj(start_pos, end_pos, start_vel, start_yaw);
}

void TrajectoryGenerator6::FinishTraj(void) {
  pimpl_->FinishTraj();
}

void TrajectoryGenerator6::StopTraj(void) {
  pimpl_->is_running = false;
}

void TrajectoryGenerator6::SetMaxVelocity(float value) {
  (void)value;
}

void TrajectoryGenerator6::SetAcceleration(float value) {
  (void)value;
}

void TrajectoryGenerator6::GetPosition(Vector3Df &point) const {
  pimpl_->output->GetMutex();
  point.x = pimpl_->output->ValueNoMutex(0, 0);
  point.y = pimpl_->output->ValueNoMutex(0, 1);
  point.z = pimpl_->output->ValueNoMutex(0, 2);
  pimpl_->output->ReleaseMutex();
}

void TrajectoryGenerator6::GetSpeed(Vector3Df &point) const {
  pimpl_->output->GetMutex();
  point.x = pimpl_->output->ValueNoMutex(1, 0);
  point.y = pimpl_->output->ValueNoMutex(1, 1);
  point.z = pimpl_->output->ValueNoMutex(1, 2);
  pimpl_->output->ReleaseMutex();
}

void TrajectoryGenerator6::GetAcceleration(Vector3Df &point) const {
  pimpl_->output->GetMutex();
  point.x = pimpl_->output->ValueNoMutex(2, 0);
  point.y = pimpl_->output->ValueNoMutex(2, 1);
  point.z = pimpl_->output->ValueNoMutex(2, 2);
  pimpl_->output->ReleaseMutex();
}

void TrajectoryGenerator6::Update(Time time) {
  pimpl_->Update(time);
  ProcessUpdate(pimpl_->output);
}

float TrajectoryGenerator6::GetYaw(void) const {
  return pimpl_->GetYaw();
}

// Compatibilidad (viejo)
void TrajectoryGenerator6::updateTarget(Vector3Df posTarget, float yawTarget) {
  pimpl_->setTargetPosition(posTarget, yawTarget);
}

// NUEVO: target dinámico (pos + vel + yaw)
void TrajectoryGenerator6::updateTargetState(Vector3Df posTarget,
                                             Vector3Df velTarget,
                                             float yawTarget) {
  pimpl_->setTargetState(posTarget, velTarget, yawTarget);
}

} // end namespace filter
} // end namespace flair
