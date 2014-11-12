#ifndef _GAZEBO_EXPLORER_HH_
#define _GAZEBO_EXPLORER_HH_

#include "AntzState.hh"
#include <unordered_map>

namespace gazebo {
    class Explorer : public AntzState {
    public:
        static Explorer *Instance(AntzPlugin *plugin);
        void Reset(double direction, const common::UpdateInfo &info);
    protected:
        Explorer(AntzPlugin *plugin);
        virtual bool DetectTarget(const common::UpdateInfo &info);
        virtual void DoInitialize(const common::UpdateInfo &info);
        virtual void DoGetInfo(int id, const common::UpdateInfo &info);
        virtual void DoAction(const common::UpdateInfo &info);
    private:
        static std::unordered_map<AntzPlugin*, Explorer*> _instance;
        int _startTime;
    };
}

#endif