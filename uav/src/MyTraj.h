//  created:    2011/05/01
//  filename:   MyTraj.h
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

#ifndef MYTRAJ_H
#define MYTRAJ_H

#include <UavStateMachine.h>

namespace flair {
    namespace gui {
        class PushButton;
        class DoubleSpinBox;
        class Tab;
        class CheckBox;
    }
    namespace filter {
        class TrajectoryGenerator6;
        class Law;  // ← NUEVO: Tu control con cuaterniones
    }
    namespace meta {
        class MetaVrpnObject;
    }
    namespace sensor {
        class TargetController;
    }
}

class MyTraj : public flair::meta::UavStateMachine {
    public:
        MyTraj(flair::sensor::TargetController *controller);
        ~MyTraj();

    private:

        enum class BehaviourMode_t {
            Default,
            PositionHold,
            SixthTrajectory
        };

        BehaviourMode_t behaviourMode;
        bool vrpnLost;

        void VrpnPositionHold(void);
        void StartSixthTrajectory(void);
        void StopSixthTrajectory(void);

        void ExtraSecurityCheck(void) override;
        void ExtraCheckPushButton(void) override;
        void ExtraCheckJoystick(void) override;
        const flair::core::AhrsData *GetOrientation(void) const override;
        void AltitudeValues(float &z,float &dz) const override;
        void GetReferenceAltitude(float &z_ref, float &dz_ref) override;
        void ComputeCustomTorques(flair::core::Euler &torques) override;
        float ComputeCustomThrust(void) override; 
        
        flair::core::AhrsData *GetReferenceOrientation(void) override;
        void SignalEvent(Event_t event) override;
  
        // Control con cuaterniones
        flair::filter::Law *quaternionControl;

        flair::core::Vector2Df posHold;
        float yawHold;

        flair::gui::PushButton *positionHold;
        flair::gui::PushButton *startSixthTraj, *stopSixthTraj;
        
        flair::meta::MetaVrpnObject *uavVrpn, *targetVrpn;
        flair::filter::TrajectoryGenerator6 *sixTrajectory;
        
        flair::core::AhrsData *customReferenceOrientation, *customOrientation;

        float frozenAltitudeRef;       
        float frozenAltitudeVelRef;    
        bool useFrozenAltitudeRef;     
};

#endif // MYTRAJ_H