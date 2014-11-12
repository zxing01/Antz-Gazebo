#include "Avoider.hh"

using namespace gazebo;

std::unordered_map<AntzPlugin*, Avoider*> Avoider::_instance;

//////////////////////////////////////////////////////////////////////////////////////////
Avoider::Avoider(AntzPlugin *plugin): AntzState(plugin), _startTime(INT_MAX) {}

//////////////////////////////////////////////////////////////////////////////////////////
Avoider *Avoider::Instance(AntzPlugin *plugin) {
    if (!_instance.count(plugin))
        _instance[plugin] = new Avoider(plugin);
    return _instance[plugin];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Avoider::Reset(AntzState *prev, const common::UpdateInfo &info) {
    _prev = prev;
    _startTime = info.simTime.sec;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Avoider::DoInitialize(const common::UpdateInfo &info) {
}

//////////////////////////////////////////////////////////////////////////////////////////
void Avoider::DoGetInfo(int id, const common::UpdateInfo &info) {
}

//////////////////////////////////////////////////////////////////////////////////////////
void Avoider::DoAction(const common::UpdateInfo &info) {
    if (info.simTime.sec - _startTime <= AVOID_TIME) {
        if (!_obstacle)
            Move();
        else
            _startTime = info.simTime.sec;
    }
    else
        SetState(_prev); // return to previous state
}