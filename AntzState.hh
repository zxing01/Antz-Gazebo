#ifndef _GAZEBO_ANTZ_STATE_HH_
#define _GAZEBO_ANTZ_STATE_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include "Antz.hh"
#include "AntzPlugin.hh"

namespace gazebo {
    class AntzState {
    public:
        friend class AntzPlugin;
        AntzState(AntzPlugin *plugin);
        virtual ~AntzState() {}
        void Update(const common::UpdateInfo &info); // template method
    protected:
        // hook operation
        virtual bool DetectTarget(const common::UpdateInfo &info) { return false; }
        
        // primitive operations
        virtual void DoInitialize(const common::UpdateInfo &info) = 0;
        virtual void DoGetInfo(int id, const common::UpdateInfo &info) = 0;
        virtual void DoAction(const common::UpdateInfo &info) = 0;
        
        // concrete operations
        void SetState(AntzState *nextState);
        void Move(double speedRatio = 1);
        void Stop();
        void TurnRandom();
        void Turn(double direction);
        double AngleFacing(double x, double y);

        AntzPlugin *_plugin;
        bool _obstacle;
    };
}

#endif