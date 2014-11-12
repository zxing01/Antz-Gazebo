#ifndef _GAZEBO_AVOIDER_HH_
#define _GAZEBO_AVOIDER_HH_

#include "AntzState.hh"
#include <unordered_map>

namespace gazebo {
    class Avoider : public AntzState {
    public:
        static Avoider *Instance(AntzPlugin *plugin);
        void Reset(AntzState *prev, const common::UpdateInfo &info);
    protected:
        Avoider(AntzPlugin *plugin);
        virtual void DoInitialize(const common::UpdateInfo &info);
        virtual void DoGetInfo(int id, const common::UpdateInfo &info);
        virtual void DoAction(const common::UpdateInfo &info);
    private:
        static std::unordered_map<AntzPlugin*, Avoider*> _instance;
        AntzState *_prev;
        int _startTime;
    };
}

#endif