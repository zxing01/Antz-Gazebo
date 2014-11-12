#include "Explorer.hh"
#include "Beacon.hh"
#include "Walker.hh"
#include "Avoider.hh"

using namespace gazebo;

std::unordered_map<AntzPlugin*, Explorer*> Explorer::_instance;

//////////////////////////////////////////////////////////////////////////////////////////
Explorer::Explorer(AntzPlugin *plugin): AntzState(plugin), _startTime(INT_MAX) {}

//////////////////////////////////////////////////////////////////////////////////////////
Explorer *Explorer::Instance(AntzPlugin *plugin) {
    if (!_instance.count(plugin))
        _instance[plugin] = new Explorer(plugin);
    return _instance[plugin];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Explorer::Reset(double direction, const common::UpdateInfo &info) {
    Turn(direction);
    _startTime = info.simTime.sec;
    ANTZ(_plugin->ID(), close).assign(TARGET_COUNT, -1);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Explorer::DetectTarget(const common::UpdateInfo &info) {
    double distance = ANTZ(_plugin->ID(), position)->Distance(AntzInfo::targetPos[_plugin->Target()]);
    if (distance <= DETECT_RANGE) {
        Beacon *beacon = Beacon::Instance(_plugin);
        SetState(beacon);
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Explorer::DoInitialize(const common::UpdateInfo &info) {
    std::cout << " #" << _plugin->ID() << " is explorer\n";
}

//////////////////////////////////////////////////////////////////////////////////////////
void Explorer::DoGetInfo(int id, const common::UpdateInfo &info) {
}

//////////////////////////////////////////////////////////////////////////////////////////
void Explorer::DoAction(const common::UpdateInfo &info) {
    if (info.simTime.sec - _startTime <= EXPLORE_TIME) {
        if (!_obstacle)
            Move();
        else {
            Avoider *avoider = Avoider::Instance(_plugin);
            avoider->Reset(this, info);
            SetState(avoider);
        }
    }
    else {
        Walker *walker = Walker::Instance(_plugin);
        walker->Reset();
        SetState(walker);
    }
}