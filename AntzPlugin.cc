#include "AntzPlugin.hh"
#include "AntzState.hh"
#include "Walker.hh"
#include <limits>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(AntzPlugin)

//////////////////////////////////////////////////////////////////////////////////////////
AntzPlugin::AntzPlugin():
_target(1),
_state(Walker::Instance(this)) {}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/) {
    // Store the pointer to the model
    _model = parent;
    
    // hook up the position of the model
    _id = AntzInfo::antz.size();
    AntzInfo::antz.push_back(AntzInfo(&_model->GetWorldPose().pos));
    
    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    _updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&AntzPlugin::OnWorldUpdate, this, _1));
    
    // Create our node for communication
    _node = transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
    // Publish to a Gazebo topic
    _pub = _node->Advertise<msgs::Vector2d>("~/" + _model->GetName() + "/colorful");
}

//////////////////////////////////////////////////////////////////////////////////////////
void AntzPlugin::PublishColorful() {
    msgs::Vector2d msg;
    if (AntzInfo::antz[_id].close[0] >= 0) {
        msg.set_x(AntzInfo::antz[_id].close[1]);
        msg.set_y(AntzInfo::antz[_id].close[0]);
    }
    else if (_target == 1) { // food
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
void AntzPlugin::OnWorldUpdate(const common::UpdateInfo &info) {
    PublishColorful();
    _state->Update(info);
}
