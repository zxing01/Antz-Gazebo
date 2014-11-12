#ifndef _GAZEBO_WALKER_HH_
#define _GAZEBO_WALKER_HH_

#include "AntzState.hh"
#include <unordered_map>
#include <map>
#include <vector>

namespace gazebo {
    class Walker : public AntzState {
    public:
        static Walker *Instance(AntzPlugin *plugin);
        void Reset();
    protected:
        Walker(AntzPlugin *plugin);
        virtual bool DetectTarget(const common::UpdateInfo &info);
        virtual void DoInitialize(const common::UpdateInfo &info);
        virtual void DoGetInfo(int id, const common::UpdateInfo &info);
        virtual void DoAction(const common::UpdateInfo &info);
    private:
        static std::unordered_map<AntzPlugin*, Walker*> _instance;
        int _signalCount;
        std::map<double, std::vector<int> > _attracts;
        std::map<int, std::vector<int> > _freqs;
    };
}

#endif