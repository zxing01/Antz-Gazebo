#ifndef _GAZEBO_COLOR_PLUGIN_HH_
#define _GAZEBO_COLOR_PLUGIN_HH_

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "Antz.hh"

namespace gazebo
{
	class Colorful : public VisualPlugin
	{
	public:
    void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    void MsgCallback(ConstVector2dPtr &msg);

	private:
    // Pointer to the model
    rendering::VisualPtr _visual;
    // Pointer to the update event connection
    //event::ConnectionPtr updateConnection;
    transport::NodePtr _node;
    transport::SubscriberPtr _sub;
	};
}

#endif