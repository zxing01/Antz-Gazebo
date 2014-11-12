#include "ColorfulPlugin.hh"
#include <math.h>

using namespace gazebo;
GZ_REGISTER_VISUAL_PLUGIN(Colorful)

void Colorful::Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    _visual = _parent;
    // Listen to the update event. This event is broadcast pre-render update???
    //this->updateConnection = event::Events::ConnectPreRender(boost::bind(&Colorful::OnUpdate, this));
    
    // Create our node for communication
    _node = transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
    
    std::string visualName = _visual->GetName();
    std::string topicName = "~/" + visualName.substr(0, visualName.find_first_of(':')) + "/colorful";
    // Listen to Gazebo world_stats topic
    _sub = _node->Subscribe(topicName, &Colorful::MsgCallback, this);
}

// Called by the world update start event
void Colorful::MsgCallback(ConstVector2dPtr &msg) {
    double x = msg->x();
    double y = msg->y();
    common::Color color;
    if (x == -1) {
        color.r = 1;
    }
    else if (x == -2) {
        color.r = 1;
        color.g = 1;
    }
    else if (x >= 0) {
        color.g = 1 - std::pow(0.75, y + 1);
        color.b = 1 - std::pow(0.75, x + 1);
    }
    
    _visual->SetAmbient(color);
    _visual->SetDiffuse(color);
}
/*
void Colorful::OnUpdate() {
    for (int i = 0; i < ANTZ_COUNT; ++i) {
        double x = ANTZ(i, close)[0];
        double y = ANTZ(i, close)[1];
        common::Color color;
        if (x == -1) {
            color.r = 1;
        }
        else if (x == -2) {
            color.r = 1;
            color.g = 1;
        }
        else if (x >= 0) {
            color.g = 1 - std::pow(0.75, x + 1);
            color.b = 1 - std::pow(0.75, y + 1);
        }
        
        _visual->SetAmbient(color);
        _visual->SetDiffuse(color);

    }
}
*/