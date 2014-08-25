#include "AntzPlugin.hh"
#include <climits>
#define ANTZ(x, y) boost::get<y>(_antz[x])

#define DEBUG

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Antz)

/////////////////////////////////////////////////
int Antz::_antzID = 0;
boost::tuple<int*, int*, const math::Vector3*> Antz::_antz[ANTZ_COUNT];
math::Vector3 Antz::_nestPos(NEST_X, NEST_Y, ANTZ_HGT / 2);
math::Vector3 Antz::_foodPos(FOOD_X, FOOD_Y, ANTZ_HGT / 2);
unsigned long Antz::_foodReturned = 0;

//////////////////////////////////////////////////////////////////////////////////////////
Antz::Antz():
_isBeacon(false),
_toNest(false),
_shouldExplore(false),
_shouldRetrieve(false),
_shouldAvoid(false),
_id(_antzID++),
_foodCardinal(-1),
_nestCardinal(-1),
_exploreStartTime(INT_MAX),
_retrieveStartTime(INT_MAX),
_avoidStartTime(INT_MAX),
_lastActiveTime(INT_MAX),
_lastTarget(ANTZ_COUNT),
_lastFoodCardinal(-1),
_lastNestCardinal(-1),
_orientation(0, 0, 0) {}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) {
    // Store the pointer to the model
    _model = parent;
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Antz::OnWorldUpdate, this, _1));
    
    // connect the dictionary with this antz's info
    _antz[_id] = boost::make_tuple(&_foodCardinal, &_nestCardinal, &_model->GetWorldPose().pos);
    
    // Create our node for communication
    _node = transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
    // Publish to a Gazebo topic
    _pub = _node->Advertise<msgs::Vector2d>("~/" + _model->GetName() + "/colorful");
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::PublishColorful() {
    msgs::Vector2d msg;
    if (_isBeacon) {
        msg.set_x(_foodCardinal);
        msg.set_y(_nestCardinal);
    }
    else if (!_toNest) {
        msg.set_x(-1);
        msg.set_y(-1);
    }
    else {
        msg.set_x(-2);
        msg.set_y(-2);
    }
    _pub->Publish(msg);
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Move() {
    math::Quaternion orientation = _model->GetWorldPose().rot;
    double yaw = orientation.GetYaw();
    math::Vector3 speed(SPEED * cos(yaw), SPEED * sin(yaw), 0);
    _model->SetLinearVel(speed);
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Stop() {
    math::Vector3 speed(0, 0, 0);
    _model->SetLinearVel(speed);
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Turn(double direction) {
    math::Pose pose = _model->GetWorldPose();
    pose.rot.SetFromEuler(0, 0, direction);
    _orientation.SetFromEuler(0, 0, direction); // for calibration
    _model->SetWorldPose(pose);
}

//////////////////////////////////////////////////////////////////////////////////////////
int Antz::RandomTarget(int minFoodCardinal, int minNestCardinal) {
    if (((!_toNest && _lastFoodCardinal == minFoodCardinal) || (_toNest && _lastNestCardinal == minNestCardinal))
        && _lastTarget < ANTZ_COUNT && ANTZ(_id, 2)->Distance(*ANTZ(_lastTarget, 2)) <= COMM_RANGE)
        return _lastTarget;
    
    _lastFoodCardinal = minFoodCardinal;
    _lastNestCardinal = minNestCardinal;
    
    if ((!_toNest && minFoodCardinal == ANTZ_COUNT) || (_toNest && minNestCardinal == ANTZ_COUNT))
        return _lastTarget = ANTZ_COUNT;
    
    std::vector<int> candidates;
    for (int i = 0; i < ANTZ_COUNT; ++i) {
        if (i == _id || !ANTZ(i, 0) || ANTZ(_id, 2)->Distance(*ANTZ(i, 2)) > COMM_RANGE)
            continue;
        
        if ((!_toNest && *ANTZ(i, 0) == minFoodCardinal) || (_toNest && *ANTZ(i, 1) == minNestCardinal))
            candidates.push_back(i);
    }
    return _lastTarget = candidates[math::Rand::GetIntUniform(0, candidates.size() - 1)];
}

//////////////////////////////////////////////////////////////////////////////////////////
double Antz::AngleFacing(double x, double y) {
    double deltaX = x - ANTZ(_id, 2)->x;
    double deltaY = y - ANTZ(_id, 2)->y;
    return atan2(deltaY, deltaX);
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Walker(const common::UpdateInfo &info, int signalCount, int target) {
    if (signalCount < SIGN_THR) {
        _isBeacon = true;
        _lastActiveTime = info.simTime.sec;
    }
    else if (target == ANTZ_COUNT || ANTZ(_id, 2)->Distance(*ANTZ(target, 2)) <= AVOID_RANGE) {
        double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
        Turn(direction);
        _exploreStartTime = info.simTime.sec;
        _shouldExplore = true;
    }
    else {
        Turn(AngleFacing(ANTZ(target, 2)->x, ANTZ(target, 2)->y));
        if (!DetectObstacle(info, target))
            Move();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Beacon(const common::UpdateInfo &info, int signalCount, int sameSignalCount, int minFoodCardinal, int minNestCardinal, bool isActive) {
    if (ANTZ(_id, 2)->Distance(_foodPos) <= TARGET_RANGE)
        _foodCardinal = 0;
    else
        _foodCardinal = minFoodCardinal == ANTZ_COUNT ? ANTZ_COUNT : minFoodCardinal + 1;
    
    if (ANTZ(_id, 2)->Distance(_nestPos) <= TARGET_RANGE)
        _nestCardinal = 0;
    else
        _nestCardinal = minNestCardinal == ANTZ_COUNT ? ANTZ_COUNT : minNestCardinal + 1;
    
    if (isActive)
        _lastActiveTime = info.simTime.sec;
    else if (_foodCardinal > 0 && _nestCardinal > 0 && info.simTime.sec - _lastActiveTime > WAIT_THR) { // inactive
        Revive();
        StartRetrieve(info);
    }
    
    if (_foodCardinal == ANTZ_COUNT && _nestCardinal == ANTZ_COUNT) { // lost beacons
        Revive();
        StartRetrieve(info);
    }
    else if (sameSignalCount > IDENT_THR && CHANGE_PROB > math::Rand::GetDblUniform()) { // possible redundancy
        Revive();
        StartExplore(info);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Explore(const common::UpdateInfo &info, int target) {
    double distance = ANTZ(_id, 2)->Distance(_toNest ? _nestPos : _foodPos);
    if (distance <= AVOID_RANGE) {
        StopExplore();
        StopRetrieve();
        if (_toNest)
            ++_foodReturned;
        _toNest = !_toNest;
    }
    else if (distance <= TARGET_RANGE) {
        math::Vector3 &pos = _toNest? _nestPos : _foodPos;
        Turn(AngleFacing(pos.x, pos.y));
        if (!DetectObstacle(info, target))
            Move();
    }
    else if (info.simTime.sec - _exploreStartTime <= EXPLORE_TIME) {
        if (!DetectObstacle(info, target))
            Move();
    }
    else
        StopExplore();
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Retrieve(const common::UpdateInfo &info, int target) {
    if (info.simTime.sec - _retrieveStartTime > RETRIEVE_TIME)
        StopRetrieve();
    else if (target == ANTZ_COUNT || ANTZ(_id, 2)->Distance(*ANTZ(target, 2)) <= AVOID_RANGE) {
        double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
        Turn(direction);
        _exploreStartTime = info.simTime.sec;
        _shouldExplore = true;
    }
    else {
        Turn(AngleFacing(ANTZ(target, 2)->x, ANTZ(target, 2)->y));
        if (!DetectObstacle(info, target))
            Move();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Avoid(const common::UpdateInfo &info, int target) {
    if (info.simTime.sec - _avoidStartTime <= AVOID_TIME) {
        if (!DetectObstacle(info, target))
            Move();
    }
    else
        _shouldAvoid = false;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::Revive() {
    _isBeacon = false;
    _foodCardinal = -1;
    _nestCardinal = -1;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::StartExplore(const common::UpdateInfo &info) {
    double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
    Turn(direction);
    _exploreStartTime = info.simTime.sec;
    _shouldExplore = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::StopExplore() {
    _lastFoodCardinal = -1;
    _lastNestCardinal = -1;
    _lastTarget = ANTZ_COUNT;
    _shouldExplore = false;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::StartRetrieve(const common::UpdateInfo &info) {
    _retrieveStartTime = info.simTime.sec;
    _shouldRetrieve = true;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::StopRetrieve() {
    _lastFoodCardinal = -1;
    _lastNestCardinal = -1;
    _lastTarget = ANTZ_COUNT;
    _shouldRetrieve = false;
}

//////////////////////////////////////////////////////////////////////////////////////////
bool Antz::DetectObstacle(const common::UpdateInfo &info, int target) {
    math::Quaternion orientation = _model->GetWorldPose().rot;
    double yaw = orientation.GetYaw();
    math::Vector3 pt1(*ANTZ(_id, 2));
    math::Vector3 pt2(pt1.x + AVOID_RANGE * cos(yaw), pt1.y + AVOID_RANGE * sin(yaw), pt1.z);
    double limit = WORLD_LEN / 2;
    
    if (pt2.x >= limit || pt2.x <= -limit || pt2.y >= limit || pt2.y <= -limit) {
        yaw += math::Rand::GetDblUniform(0, AVOID_ANGLE);
        Turn(yaw);
        _avoidStartTime = info.simTime.sec;
        _shouldAvoid = true;
        return true;
    }
    
    for (int i = 0; i < ANTZ_COUNT; ++i) {
        if (i == _id || !ANTZ(i, 0) || (!_shouldExplore && !_shouldAvoid && !_shouldRetrieve && i == target))
            continue;
        
        double distance = ANTZ(_id, 2)->Distance(*ANTZ(i, 2));
        
        if (distance <= AVOID_RANGE) {
            //math::Vector3 pt0(*ANTZ(i, 2));
            //double distToRoute = pt0.GetDistToLine(pt1, pt2);
            double angle = AngleFacing(ANTZ(i, 2)->x, ANTZ(i, 2)->y); // [-pi/2, pi/2]
            //std::cout << "  distToRoute(" << i << ") = " << distToRoute << " yaw = " << yaw << " angle = " << angle << "\n";
            double delta = angle - yaw;
            if ((delta > 0 && delta < M_PI/2) || (delta < 0 && delta > -M_PI/2) || (delta < 2*M_PI && delta > 3*M_PI/2) || (delta > -2*M_PI && delta < -3*M_PI/2)) {
                yaw += math::Rand::GetDblUniform(0, AVOID_ANGLE);
                Turn(yaw);
                _avoidStartTime = info.simTime.sec;
                _shouldAvoid = true;
                //std:: cout << "  --- avoiding!\n";
                return true;
            }
        }
    }
    //std:: cout << "  --- NOT avoiding!\n";
    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////
void Antz::OnWorldUpdate(const common::UpdateInfo &info) {
    PublishColorful();
    Stop();
    
    // for calibration
    math::Pose pose = _model->GetWorldPose();
    pose.rot = _orientation;
    _model->SetWorldPose(pose);
    
    int signalCount = 0;
    int sameSignalCount = 0;
    int minFoodCardinal = ANTZ_COUNT;
    int minNestCardinal = ANTZ_COUNT;
    int target = ANTZ_COUNT;
    bool isActive = false;
    
    for (int i = 0; i < ANTZ_COUNT; ++i) {
        if (i == _id || !ANTZ(i, 0))
            continue;
        
        double distance = ANTZ(_id, 2)->Distance(*ANTZ(i, 2));
        
        if (distance > COMM_RANGE)
            continue;
        else if (*ANTZ(i, 0) >= 0 || *ANTZ(i, 1) >= 0) {
            ++signalCount;
            if (*ANTZ(i, 0) == _foodCardinal && *ANTZ(i, 1) == _nestCardinal)
                ++sameSignalCount;
            minFoodCardinal = *ANTZ(i, 0) < minFoodCardinal ? *ANTZ(i, 0) : minFoodCardinal;
            minNestCardinal = *ANTZ(i, 1) < minNestCardinal ? *ANTZ(i, 1) : minNestCardinal;
        }
        else if (*ANTZ(i, 0) == -1)
            isActive = true;
    }
    
    if (signalCount > 0)
        target = RandomTarget(minFoodCardinal, minNestCardinal);
    
#ifdef DEBUG
    std::cout << _model->GetName() << " -- ";
#endif
    
    if (_isBeacon) {
#ifdef DEBUG
        std::cout << "Beacon\n";
#endif
        Beacon(info, signalCount, sameSignalCount, minFoodCardinal, minNestCardinal, isActive);
    }
    else if (_shouldAvoid) {
#ifdef DEBUG
        std::cout << "Avoid\n";
#endif
        Avoid(info, target);
    }
    else if (_shouldExplore) {
#ifdef DEBUG
        std::cout << "Explore\n";
#endif
        Explore(info, target);
    }
    else if (_shouldRetrieve) {
#ifdef DEBUG
        std::cout << "Retrieve\n";
#endif
        Retrieve(info, target);
    }
    else {
#ifdef DEBUG
        std::cout << "Walker\n";
#endif
        Walker(info, signalCount, target);
    }
    
#ifdef DEBUG
    std::cout << "  simTime = " << info.simTime.sec << "\n";
    std::cout << "  destination = " << (_toNest ? "Nest" : "Food") << "\n";
    std::cout << "  foodCardinal = " << _foodCardinal << "\n";
    std::cout << "  nestCardinal = " << _nestCardinal << "\n";
    std::cout << "  signalCount = " << signalCount << "\n";
    std::cout << "  sameSignalCount = " << sameSignalCount << "\n";
    std::cout << "  isActive = " << (isActive ? "Yes" : "No") << "\n";
    std::cout << "  target = " << target << "\n";
    std::cout << "  foodReturned = " << _foodReturned << "\n";
#endif
}