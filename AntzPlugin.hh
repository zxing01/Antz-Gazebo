#ifndef _GAZEBO_ANTZ_PLUGIN_HH_
#define _GAZEBO_ANTZ_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "Antz.hh"

namespace gazebo {
    class AntzState;
    class AntzPlugin : public ModelPlugin {
    public:
        friend class AntzState;
        AntzPlugin();
        void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
        void OnWorldUpdate(const common::UpdateInfo &info);
        int ID() const { return _id; }
        int &Target() { return _target; }
    private:
        void PublishColorful();
        
        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;
        transport::NodePtr _node;
        transport::PublisherPtr _pub;

        int _id;
        int _target; // 0 - nest, 1 - food
        math::Quaternion _orientation; // store the set orientation for calibration etc. (in case of collision)
        AntzState *_state;
    };
}

#endif