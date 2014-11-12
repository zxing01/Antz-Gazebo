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
    ANTZ(_plugin->ID(), frequency) = frequency;
    _lastPresence.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::DoInitialize(const common::UpdateInfo &info) {
    std::cout << " #" << _plugin->ID() << " is beacon\n";
    _sameSignalCount = 0;
    _minClose.assign(TARGET_COUNT, ANTZ_COUNT);
    ANTZ(_plugin->ID(), count) = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Beacon::DoGetInfo(int id, const common::UpdateInfo &info) {
    if (ANTZ(id, close)[0] < 0) { // this is a walker, increase frequency and count
        if (_lastPresence.count(id) == 0 || info.simTime.sec - _lastPresence[id] > EXPLORE_TIME) {
            ++ANTZ(_plugin->ID(), frequency);
            _lastPresence[id] = info.simTime.sec;
        }
        ++ANTZ(_plugin->ID(), count);
    }
    else { // this is a beacon, gather info
        bool same = true;
        for (int i = 0; i < TARGET_COUNT; ++i) {
            if (_minClose[i] > ANTZ(id, close)[i])
                _minClose[i] = ANTZ(id, close)[i];
            
            if (ANTZ(id, close)[i] != ANTZ(_plugin->ID(), close)[i])
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
        if (ANTZ(_plugin->ID(), position)->Distance(AntzInfo::targetPos[i]) <= DETECT_RANGE) {
            ANTZ(_plugin->ID(), close)[i] = 0;
            lost = false;
            source = true;
        }
        else if (_minClose[i] < ANTZ_COUNT) {
            ANTZ(_plugin->ID(), close)[i] = _minClose[i] + 1;
            lost = false;
        }
        else
            ANTZ(_plugin->ID(), close)[i] = ANTZ_COUNT;
        
        ANTZ(_plugin->ID(), attractive)[i] = ANTZ(_plugin->ID(), close)[i] + ANTZ(_plugin->ID(), count)/10;
    }
    
    if (lost || (!source && _sameSignalCount > IDENT_THR && CHANGE_PROB > math::Rand::GetDblUniform())) {
        double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
        Explorer *explorer = Explorer::Instance(_plugin);
        explorer->Reset(direction, info);
        SetState(explorer);
    }
    //std::cout << " ####### id = " << _plugin->ID() << " close[0] = " << ANTZ(_plugin->ID(), close)[0] << " close[1] = " << ANTZ(_plugin->ID(), close)[1] << "\n";
}