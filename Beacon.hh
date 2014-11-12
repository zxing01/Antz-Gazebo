#ifndef _GAZEBO_BEACON_HH_
#define _GAZEBO_BEACON_HH_

#include "AntzState.hh"
#include <unordered_map>

namespace gazebo {
    class Beacon : public AntzState {
    public:
        static Beacon *Instance(AntzPlugin *plugin);
        void Reset(int frequency);
    protected:
        Beacon(AntzPlugin *plugin);
        virtual void DoInitialize(const common::UpdateInfo &info);
        virtual void DoGetInfo(int id, const common::UpdateInfo &info);
        virtual void DoAction(const common::UpdateInfo &info);
    private:
        static std::unordered_map<AntzPlugin*, Beacon*> _instance;
        std::unordered_map<int, int> _lastPresence;
        std::vector<int> _minClose;
        int _sameSignalCount;
    };
}

#endif