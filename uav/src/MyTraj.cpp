//  created:    2011/05/01
//  filename:   MyTraj.cpp
//
//  author:     Guillaume Sanahuja & Jossue Carino
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  version:    $Id: $
//
//  purpose:    demo trayectoria de 6to grado
//
//
/*********************************************************************/

#include "MyTraj.h"
#include <TargetController.h>
#include <Uav.h>
#include <GridLayout.h>
#include <PushButton.h>
#include <DoubleSpinBox.h>
#include <DataPlot1D.h>
#include <DataPlot2D.h>
#include <MetaDualShock3.h>
#include <FrameworkManager.h>
#include <VrpnClient.h>
#include <MetaVrpnObject.h>
#include <Tab.h>
#include <TabWidget.h>
#include <DataPlot1D.h>
#include <CheckBox.h>
#include <Matrix.h>
#include <cmath>
#include <Tab.h>
#include <Pid.h>
#include <PidThrust.h>
#include <Ahrs.h>
#include <AhrsData.h>
#include <NestedSat.h>

#include "TrajectoryGenerator6.h"

using namespace std;
using namespace flair::core;
using namespace flair::gui;
using namespace flair::sensor;
using namespace flair::filter;
using namespace flair::meta;

MyTraj::MyTraj(TargetController *controller): UavStateMachine(controller), 
    behaviourMode(BehaviourMode_t::Default), vrpnLost(false) {
    
    Uav* uav = GetUav();

    // Inicializar cliente VRPN
    VrpnClient* vrpnclient = new VrpnClient("vrpn", uav->GetDefaultVrpnAddress(), 80);
    
    if(vrpnclient->ConnectionType() == VrpnClient::Xbee) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(), '0');
        targetVrpn = new MetaVrpnObject("target", '0');
    } else if (vrpnclient->ConnectionType() == VrpnClient::Vrpn) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName());
        targetVrpn = new MetaVrpnObject("target");
    } else if (vrpnclient->ConnectionType() == VrpnClient::VrpnLite) {
        uavVrpn = new MetaVrpnObject(uav->ObjectName(), '0');
        targetVrpn = new MetaVrpnObject("target", '0');
    }
    
    getFrameworkManager()->AddDeviceToLog(uavVrpn);
    getFrameworkManager()->AddDeviceToLog(targetVrpn);
    vrpnclient->Start();
    
    uav->GetAhrs()->YawPlot()->AddCurve(uavVrpn->State()->Element(2), DataPlot::Green);

    // Botones de control
    positionHold = new PushButton(GetButtonsLayout()->NewRow(), "position hold");
    startSixthTraj = new PushButton(GetButtonsLayout()->NewRow(), "start sixth trajectory");
    stopSixthTraj = new PushButton(GetButtonsLayout()->LastRowLastCol(), "stop sixth trajectory");
 
    // Inicializar generador de trayectoria de 6to grado
    sixTrajectory = new TrajectoryGenerator6(vrpnclient->GetLayout()->NewRow(), "Sixth grade trajectory");
    uavVrpn->xPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(0, 0), DataPlot::Blue);
    uavVrpn->yPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(0, 1), DataPlot::Blue);
    uavVrpn->zPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(0, 2), DataPlot::Blue);
    uavVrpn->VxPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(1, 0), DataPlot::Blue);
    uavVrpn->VyPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(1, 1), DataPlot::Blue);
    uavVrpn->VzPlot()->AddCurve(sixTrajectory->GetMatrix()->Element(1, 2), DataPlot::Blue);
    
    getFrameworkManager()->AddDeviceToLog(sixTrajectory);
    
    // 
    uX = new Pid(setupLawTab->At(1,0), "u_x");
    uX->UseDefaultPlot(graphLawTab->NewRow());
    uY = new Pid(setupLawTab->At(1,1), "u_y");
    uY->UseDefaultPlot(graphLawTab->LastRowLastCol());
    getFrameworkManager()->AddDeviceToLog(uX); 
    //uX->AddDataToLog(uY);
    // Orientación de referencia
    customReferenceOrientation = new AhrsData(this, "reference");
    uav->GetAhrs()->AddPlot(customReferenceOrientation, DataPlot::Yellow);
    AddDataToControlLawLog(customReferenceOrientation);
    AddDeviceToControlLawLog(uX);
    AddDeviceToControlLawLog(uY);
    

    customOrientation = new AhrsData(this, "orientation");
}

MyTraj::~MyTraj() {
}

const AhrsData *MyTraj::GetOrientation(void) const {
    // Obtener yaw del VRPN
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);

    // Obtener roll, pitch y velocidades angulares del IMU
    Quaternion ahrsQuaternion;
    Vector3Df ahrsAngularSpeed;
    GetDefaultOrientation()->GetQuaternionAndAngularRates(ahrsQuaternion, ahrsAngularSpeed);

    // Mezclar: roll y pitch del IMU, yaw del VRPN
    Euler ahrsEuler = ahrsQuaternion.ToEuler();
    ahrsEuler.yaw = vrpnQuaternion.ToEuler().yaw;
    Quaternion mixQuaternion = ahrsEuler.ToQuaternion();

    customOrientation->SetQuaternionAndAngularRates(mixQuaternion, ahrsAngularSpeed);

    return customOrientation;
}

void MyTraj::AltitudeValues(float &z, float &dz) const {
    Vector3Df uav_pos, uav_vel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);
    
    // z y dz deben estar en el marco del UAV
    z = -uav_pos.z;
    dz = -uav_vel.z;
}

void MyTraj::GetReferenceAltitude(float &z_ref, float &dz_ref) {
    if (behaviourMode == BehaviourMode_t::SixthTrajectory) {
        Vector3Df des_pos, des_vel;
        sixTrajectory->GetPosition(des_pos);
        sixTrajectory->GetSpeed(des_vel);
            
        z_ref = -des_pos.z;
        dz_ref = -des_vel.z;
        // Actualizar las referencias durante la trayectoria
        frozenAltitudeRef = z_ref;
        frozenAltitudeVelRef = dz_ref;
        
    } else if (useFrozenAltitudeRef) {
        // Después de detener: usar referencias del último momento de la trayectoria
        z_ref = frozenAltitudeRef;
        dz_ref = 0.0f;  // Velocidad a cero
    } else {
        GetDefaultReferenceAltitude(z_ref, dz_ref);
    }
}

AhrsData *MyTraj::GetReferenceOrientation(void) {
    Vector2Df pos_err, vel_err;
    float yaw_ref;
    
    Euler refAngles;

    PositionValues(pos_err, vel_err, yaw_ref);

    refAngles.yaw = yaw_ref;

    uX->SetValues(pos_err.x, vel_err.x);
    uX->Update(GetTime());
    refAngles.pitch = uX->Output();

    uY->SetValues(pos_err.y, vel_err.y);
    uY->Update(GetTime());
    refAngles.roll = -uY->Output();

    customReferenceOrientation->SetQuaternionAndAngularRates(refAngles.ToQuaternion(), Vector3Df(0,0,0));

    return customReferenceOrientation;
}

void MyTraj::PositionValues(Vector2Df &pos_error, Vector2Df &vel_error, float &yaw_ref) {
    Vector3Df uav_pos, uav_vel;
    Vector2Df uav_2Dpos, uav_2Dvel;

    uavVrpn->GetPosition(uav_pos);
    uavVrpn->GetSpeed(uav_vel);

    uav_pos.To2Dxy(uav_2Dpos);
    uav_vel.To2Dxy(uav_2Dvel);

    if (behaviourMode == BehaviourMode_t::PositionHold) {
        pos_error = uav_2Dpos - posHold;
        vel_error = uav_2Dvel;
        yaw_ref = yawHold;
        
    } else if (behaviourMode == BehaviourMode_t::SixthTrajectory) {
        Vector3Df des_pos, des_vel, target_pos;
        Vector2Df des_2Dpos, des_2Dvel;
        
        targetVrpn->GetPosition(target_pos);
        
        // Actualizar y obtener los valores de la trayectoria
        sixTrajectory->updateTarget(target_pos);
        sixTrajectory->Update(GetTime());
        sixTrajectory->GetPosition(des_pos);
        sixTrajectory->GetSpeed(des_vel);
        
        des_pos.To2Dxy(des_2Dpos);
        des_vel.To2Dxy(des_2Dvel);
        
        pos_error = uav_2Dpos - des_2Dpos;
        vel_error = uav_2Dvel - des_2Dvel;
        
        // Mantener yaw inicial durante trayectoria , no queremos rotar
        yaw_ref = sixTrajectory->GetYaw();
        
    } else {
        // Default: sin movimiento, mantener yaw actual
        pos_error = Vector2Df(0, 0);
        vel_error = Vector2Df(0, 0);
        yaw_ref = yawHold;
    }
    
    // Rotar errores al frame del UAV 
    Quaternion currentQuaternion = GetCurrentQuaternion();
    Euler currentAngles;
    currentQuaternion.ToEuler(currentAngles);
    pos_error.Rotate(-currentAngles.yaw);
    vel_error.Rotate(-currentAngles.yaw);
}

void MyTraj::SignalEvent(Event_t event) {
    UavStateMachine::SignalEvent(event);
    switch(event) {
    case Event_t::TakingOff:
        behaviourMode = BehaviourMode_t::Default;
        vrpnLost = false;
        useFrozenAltitudeRef = false; //seteamos la flag a false 
        break;
    case Event_t::EnteringControlLoop:
        if ((behaviourMode == BehaviourMode_t::SixthTrajectory) && (!sixTrajectory->IsRunning())) {
            VrpnPositionHold();
        }
        break;
    case Event_t::EnteringFailSafeMode:
        behaviourMode = BehaviourMode_t::Default;
        break;
    }
}

void MyTraj::ExtraSecurityCheck(void) {
    if ((!vrpnLost) && ((behaviourMode == BehaviourMode_t::SixthTrajectory) ||
                        (behaviourMode == BehaviourMode_t::PositionHold))) {
        if (!targetVrpn->IsTracked(500)) {
            Thread::Err("VRPN, target lost\n");
            vrpnLost = true;
            EnterFailSafeMode();
            Land();
        }
        if (!uavVrpn->IsTracked(500)) {
            Thread::Err("VRPN, uav lost\n");
            vrpnLost = true;
            EnterFailSafeMode();
            Land();
        }
    }
}

void MyTraj::ExtraCheckPushButton(void) {
    if(positionHold->Clicked() && (behaviourMode == BehaviourMode_t::Default)) {
        VrpnPositionHold();
    }
    
    if(startSixthTraj->Clicked() && (behaviourMode != BehaviourMode_t::SixthTrajectory)) {
        StartSixthTrajectory();
    }
    
    if(stopSixthTraj->Clicked() && (behaviourMode == BehaviourMode_t::SixthTrajectory)) {
        StopSixthTrajectory();
    }
}

void MyTraj::ExtraCheckJoystick(void) {
    // Cross (detener trayectoria)
    if(GetTargetController()->IsButtonPressed(5)) {
        if(behaviourMode == BehaviourMode_t::SixthTrajectory) {
            printf("\n>>> CROSS BUTTON PRESSED - Stopping trajectory <<<\n");
            fflush(stdout);
            StopSixthTrajectory();
        }
    }
    
    // Square (position hold)
    if(GetTargetController()->IsButtonPressed(2) && 
       (behaviourMode == BehaviourMode_t::Default)) {
        printf("\n>>> SQUARE BUTTON PRESSED - Starting position hold <<<\n");
        fflush(stdout);
        VrpnPositionHold();
    }
    
    // Triangle (iniciar trayectoria)
    if(GetTargetController()->IsButtonPressed(3) && 
       (behaviourMode != BehaviourMode_t::SixthTrajectory)) {
        printf("\n>>> TRIANGLE BUTTON PRESSED - Starting trajectory <<<\n");
        fflush(stdout);
        StartSixthTrajectory();
    }
}

void MyTraj::VrpnPositionHold(void) {
    if(behaviourMode == BehaviourMode_t::PositionHold) {
        Thread::Warn("MyTraj: already in vrpn position hold mode\n");
        return;
    }
    
    Quaternion vrpnQuaternion;
    uavVrpn->GetQuaternion(vrpnQuaternion);
    yawHold = vrpnQuaternion.ToEuler().yaw;

    Vector3Df vrpnPosition;
    uavVrpn->GetPosition(vrpnPosition);
    vrpnPosition.To2Dxy(posHold);

    // Congelar altitud actual
    float z_current, dz_current;
    AltitudeValues(z_current, dz_current);
    frozenAltitudeRef = z_current;
    frozenAltitudeVelRef = 0.0f;
    useFrozenAltitudeRef = true;

    uX->Reset();
    uY->Reset();
    
    behaviourMode = BehaviourMode_t::PositionHold;
    SetOrientationMode(OrientationMode_t::Custom);
    SetAltitudeMode(AltitudeMode_t::Custom);
    Thread::Info("MyTraj: holding position\n");
}

void MyTraj::StartSixthTrajectory(void) {
    if(behaviourMode == BehaviourMode_t::SixthTrajectory) {
        Thread::Warn("MyTraj: already in trajectory mode\n");
        return;
    }
    
    if (!SetOrientationMode(OrientationMode_t::Custom)) {
        Thread::Warn("MyTraj: could not set custom orientation mode\n");
        return;
    }
    
    if (!SetAltitudeMode(AltitudeMode_t::Custom)) {
        Thread::Warn("MyTraj: could not set custom altitude mode\n");
        return;
    }
    
    Thread::Info("MyTraj: start trajectory\n");
    useFrozenAltitudeRef = false; 
    
    // Obtener posiciones inicial y final
    Vector3Df start_pos;
    uavVrpn->GetPosition(start_pos);

    Vector3Df end_pos;
    targetVrpn->GetPosition(end_pos);
   
    Vector3Df start_vel;
    uavVrpn->GetSpeed(start_vel);

    // Iniciar trayectoria
    sixTrajectory->updateTarget(end_pos);
    sixTrajectory->StartTraj(start_pos, end_pos, start_vel);

    // Reset PIDs
    uX->Reset();
    uY->Reset();

    // Guardar yaw actual para mantenerlo durante la trayectoria
    Quaternion uav_att;
    uavVrpn->GetQuaternion(uav_att);
    yawHold = uav_att.ToEuler().yaw;

    behaviourMode = BehaviourMode_t::SixthTrajectory;
}

void MyTraj::StopSixthTrajectory(void) {
    if(behaviourMode != BehaviourMode_t::SixthTrajectory) {
        Thread::Warn("MyTraj: not in trajectory mode\n");
        return;
    }
    useFrozenAltitudeRef = true;
    sixTrajectory->FinishTraj();
    Thread::Info("MyTraj: finishing trajectory\n");
    
    // Mantener la última posición como position hold
    VrpnPositionHold();
}