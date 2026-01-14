//  created:    2011/05/01
//  filename:   MyTraj.h
//
//  author:     Guillaume Sanahuja & Jossue Carino
//              Copyright Heudiasyc UMR UTC/CNRS 7253
//
//  purpose:    demo trayectoria de 6to grado
//
/*********************************************************************/

#ifndef MYTRAJ_H
#define MYTRAJ_H

#include <UavStateMachine.h>
#include <Vector2D.h>   

namespace flair {

namespace core {
    class AhrsData;
    class Euler;
}

namespace gui {
    class DataPlot1D;
    class PushButton;
    class DoubleSpinBox;
    class Tab;
    class CheckBox;
}

namespace filter {
    class TrajectoryGenerator6;
    class Law;
}

namespace meta {
    class MetaVrpnObject;
}

namespace sensor {
    class TargetController;
}

} // namespace flair

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

    // Overrides
    void ExtraSecurityCheck(void) override;
    void ExtraCheckPushButton(void) override;
    void ExtraCheckJoystick(void) override;

    const flair::core::AhrsData *GetOrientation(void) const override;
    flair::core::AhrsData *GetReferenceOrientation(void) override;

    void AltitudeValues(float &z, float &dz) const override;
    void GetReferenceAltitude(float &z_ref, float &dz_ref) override;

    void ComputeCustomTorques(flair::core::Euler &torques) override;
    float ComputeCustomThrust(void) override;

    void SignalEvent(Event_t event) override;

    void ExitPositionHold(void);

    // Control con cuaterniones
    flair::filter::Law *quaternionControl;

    // Holds
    flair::core::Vector2Df posHold;
    float yawHold;
    float initialYaw;

    // UI
    flair::gui::PushButton *positionHold;
    flair::gui::PushButton *startSixthTraj;
    flair::gui::PushButton *stopSixthTraj;

    // VRPN
    flair::meta::MetaVrpnObject *uavVrpn;
    flair::meta::MetaVrpnObject *targetVrpn;

    // Trajectory generator
    flair::filter::TrajectoryGenerator6 *sixTrajectory;

    // Orientation data
    flair::core::AhrsData *customReferenceOrientation;
    flair::core::AhrsData *customOrientation;

    float zHold;
};

#endif // MYTRAJ_H
