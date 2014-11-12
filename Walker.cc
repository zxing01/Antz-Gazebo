#include "Walker.hh"
#include "Avoider.hh"
#include "Beacon.hh"
#include "Explorer.hh"

using namespace gazebo;

std::unordered_map<AntzPlugin*, Walker*> Walker::_instance;

//////////////////////////////////////////////////////////////////////////////////////////
Walker::Walker(AntzPlugin *plugin): AntzState(plugin) {}

//////////////////////////////////////////////////////////////////////////////////////////
Walker *Walker::Instance(AntzPlugin *plugin) {
    if (!_instance.count(plugin))
        _instance[plugin] = new Walker(plugin);
    return _instance[plugin];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Walker::Reset() {
    ANTZ(_id, close).assign(TARGET_COUNT, -1);
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Walker::DetectTarget(const common::UpdateInfo &info) {
    double distance = ANTZ(_id, position)->Distance(AntzInfo::targetPos[ANTZ(_id, target)]);
    if (distance <= DETECT_RANGE) {
        ANTZ(_id, target) = 1 - ANTZ(_id, target); // assume only one food source
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Walker::DoInitialize(const common::UpdateInfo &info) {
    _signalCount = 0;
    _attracts.clear();
    _freqs.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////
void Walker::DoGetInfo(int id, const common::UpdateInfo &info) {
    if (ANTZ(id, close)[ANTZ(_id, target)] == ANTZ_COUNT) { // target not found, follow frequency
        ++_signalCount;
        _freqs[ANTZ(id, frequency)].push_back(id);
    }
    else if (ANTZ(id, close)[ANTZ(_id, target)] >= 0) { // target found, follow attractiveness
        ++_signalCount;
        _attracts[ANTZ(id, attractive)[ANTZ(_id, target)]].push_back(id);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void Walker::DoAction(const common::UpdateInfo &info) {
    if (_attracts.size() > 0) { // go towards a beacon
        std::vector<int> &candidates = _attracts.begin()->second;
        int index = candidates[math::Rand::GetIntUniform(0, candidates.size() - 1)];
        Turn(AngleFacing(ANTZ(index, position)->x, ANTZ(index, position)->y));
        
        if (!_obstacle) { // become avoider
            double ratio = 1;
            //ratio = ANTZ(index, crowded)[ANTZ(_id, target)]; // speed control
            Move(ratio);
        }
        else {
            Avoider *avoider = Avoider::Instance(_plugin);
            avoider->Reset(this, info);
            SetState(avoider);
        }
    }
    else if (_signalCount < SIGN_THR) { // become beacon
        Beacon *beacon = Beacon::Instance(_plugin);
        int count = 0;
        int sum = 0;
        for (std::map<int, std::vector<int> >::iterator it = _freqs.begin(); it != _freqs.end(); ++it) {
            sum += it->first * it->second.size();
            count += it->second.size();
        }
        beacon->Reset(count > 1 ? sum/count : 0);
        SetState(beacon);
    }
    else if (_freqs.size() > 0) { // explore towards lowest frequency
        std::vector<int> &candidates = _freqs.begin()->second;
        int index = candidates[math::Rand::GetIntUniform(0, candidates.size() - 1)];
        double angle = AngleFacing(ANTZ(index, position)->x, ANTZ(index, position)->y);
        double direction = math::Rand::GetDblUniform(angle - M_PI/3, angle + M_PI/3);
        Explorer *explorer = Explorer::Instance(_plugin);
        explorer->Reset(direction, info);
        SetState(explorer);
    }
    else { // explore (might be redundant)
        double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
        Explorer *explorer = Explorer::Instance(_plugin);
        explorer->Reset(direction, info);
        SetState(explorer);
    }
}
