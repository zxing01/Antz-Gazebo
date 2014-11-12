#include "Beacon.hh"
#include "Explorer.hh"

using namespace gazebo;

std::unordered_map<AntzPlugin*, Beacon*> Beacon::_instance;

//////////////////////////////////////////////////////////////////////////////////////////
Beacon::Beacon(AntzPlugin *plugin): AntzState(plugin) {}

//////////////////////////////////////////////////////////////////////////////////////////
Beacon *Beacon::Instance(AntzPlugin *plugin) {
    if (!_instance.count(plugin))
        _instance[plugin] = new Beacon(plugin);
    return _instance[plugin];
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::Reset(int frequency) {
    ANTZ(_id, frequency) = frequency;
    _lastPresence.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::DoInitialize(const common::UpdateInfo &info) {
    _sameSignalCount = 0;
    _minClose.assign(TARGET_COUNT, ANTZ_COUNT);
    ANTZ(_id, count) = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::DoGetInfo(int id, const common::UpdateInfo &info) {
    if (ANTZ(id, close)[0] < 0) { // this is a walker, increase frequency and count
        if (_lastPresence.count(id) == 0 || info.simTime.sec - _lastPresence[id] > EXPLORE_TIME) {
            ++ANTZ(_id, frequency);
            _lastPresence[id] = info.simTime.sec;
        }
        ++ANTZ(_id, count);
    }
    else { // this is a beacon, gather info
        bool same = true;
        for (int i = 0; i < TARGET_COUNT; ++i) {
            if (_minClose[i] > ANTZ(id, close)[i])
                _minClose[i] = ANTZ(id, close)[i];
            
            if (ANTZ(id, close)[i] != ANTZ(_id, close)[i])
                same = false;
        }
        if (same)
            ++_sameSignalCount;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::DoAction(const common::UpdateInfo &info) {
    bool lost = true;
    bool source = false;
    for (int i = 0; i < TARGET_COUNT; ++i) {
        if (ANTZ(_id, position)->Distance(AntzInfo::targetPos[i]) <= DETECT_RANGE) {
            ANTZ(_id, close)[i] = 0;
            lost = false;
            source = true;
        }
        else if (_minClose[i] < ANTZ_COUNT) {
            ANTZ(_id, close)[i] = _minClose[i] + 1;
            lost = false;
        }
        else
            ANTZ(_id, close)[i] = ANTZ_COUNT;
        
        ANTZ(_id, attractive)[i] = ANTZ(_id, close)[i] + (double)ANTZ(_id, count)/10.0;
    }
    
    if (lost /*|| (!source && _sameSignalCount > IDENT_THR && CHANGE_PROB > math::Rand::GetDblUniform())*/) {
        double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
        Explorer *explorer = Explorer::Instance(_plugin);
        explorer->Reset(direction, info);
        SetState(explorer);
    }
}