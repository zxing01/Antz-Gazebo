#include "AntzState.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////////////////
AntzState::AntzState(AntzPlugin *plugin):
_plugin(plugin),
_id(plugin->_id) {}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::Update(const common::UpdateInfo &info) {
    // calibration
    Stop();
    math::Pose pose = _plugin->_model->GetWorldPose();
    pose.rot = _plugin->_orientation;
    _plugin->_model->SetWorldPose(pose);
    
    DoInitialize(info); // primitive operation
    
    if (DetectTarget(info)) // hook operation
        return;
    
    // avoid world boundary
    _obstacle = false;
    double yaw = _plugin->_orientation.GetYaw();
    math::Vector3 pt1(*ANTZ(_id, position));
    math::Vector3 pt2(pt1.x + DETECT_RANGE * cos(yaw), pt1.y + DETECT_RANGE * sin(yaw), pt1.z);
    double limit = WORLD_LEN / 2;
    if (pt2.x >= limit || pt2.x <= -limit || pt2.y >= limit || pt2.y <= -limit) {
        yaw += math::Rand::GetDblUniform(0, AVOID_ANGLE);
        Turn(yaw);
        _obstacle = true;
    }
    
    for (int i = 0; i < AntzInfo::antz.size(); ++i) {
        if (i == _id)
            continue;
        double distance = ANTZ(_id, position)->Distance(*ANTZ(i, position));
        if (distance > COMM_RANGE)
            continue;
        
        // avoid other antz
        if (distance <= DETECT_RANGE) {
            double angle = AngleFacing(AntzInfo::antz[i].position->x, AntzInfo::antz[i].position->y); // [-pi/2, pi/2]
            double delta = angle - yaw;
            if ((delta >= 0 && delta <= M_PI/2) || (delta <= 0 && delta >= -M_PI/2)
                || (delta <= 2*M_PI && delta >= 3*M_PI/2) || (delta >= -2*M_PI && delta <= -3*M_PI/2)) {
                yaw += math::Rand::GetDblUniform(0, AVOID_ANGLE);
                Turn(yaw);
                _obstacle = true;
            }
        }
        
        DoGetInfo(i, info); // primitive operation
    }
    DoAction(info); // primitive operation
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::SetState(AntzState *nextState) {
    if (_plugin == nextState->_plugin) {
        _plugin->_state = nextState;
    }
    else
        std::cout << " Invalid state transition for antz#" << _id << "\n";
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::Move(double speedRatio) {
    math::Quaternion orientation = _plugin->_model->GetWorldPose().rot;
    double yaw = orientation.GetYaw();
    math::Vector3 speed(SPEED * speedRatio * cos(yaw), SPEED * speedRatio * sin(yaw), 0);
    _plugin->_model->SetLinearVel(speed);
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::Stop() {
    _plugin->_model->SetLinearVel(math::Vector3(0,0,0));
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::TurnRandom() {
    double direction = math::Rand::GetDblUniform(-M_PI, M_PI);
    math::Pose pose = _plugin->_model->GetWorldPose();
    pose.rot.SetFromEuler(0, 0, direction);
    _plugin->_orientation.SetFromEuler(0, 0, direction); // for calibration
    _plugin->_model->SetWorldPose(pose);
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzState::Turn(double direction) {
    math::Pose pose = _plugin->_model->GetWorldPose();
    pose.rot.SetFromEuler(0, 0, direction);
    _plugin->_orientation.SetFromEuler(0, 0, direction); // for calibration
    _plugin->_model->SetWorldPose(pose);
}

//////////////////////////////////////////////////////////////////////////////////////////
double AntzState::AngleFacing(double x, double y) {
    double deltaX = x - ANTZ(_id, position)->x;
    double deltaY = y - ANTZ(_id, position)->y;
    return atan2(deltaY, deltaX);
}
