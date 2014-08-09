#ifndef _GAZEBO_FACTORY_PLUGIN_HH_
#define _GAZEBO_FACTORY_PLUGIN_HH_

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "Antz.hh"

namespace gazebo
{
	class Factory : public WorldPlugin
	{
  public: 
  	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
    void OnUpdate(const common::UpdateInfo &_info);
  private:
  	physics::WorldPtr parent;
		event::ConnectionPtr updateConnection;
    transport::NodePtr node;
    transport::PublisherPtr pub;
    int lastTime;
    unsigned int count;
	};
}

#endif