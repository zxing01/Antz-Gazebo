#include "FactoryPlugin.hh"

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(Factory)

/////////////////////////////////////////////////
void Factory::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
{
    this->parent = _parent;
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Factory::OnUpdate, this, _1));
    
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(_parent->GetName());
    this->pub = node->Advertise<msgs::Factory>("~/factory");
    
    msgs::Factory msg;
    msg.set_sdf("<?xml version='1.0'?>\
                <sdf version='1.4'>\
                <model name='playground'>\
                <static>true</static>\
                <link name='link'>\
                <collision name='front_wall_collision'>\
                <pose>" + std::to_string(WORLD_LEN/2 + ANTZ_HGT/2) + " 0 " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(ANTZ_HGT) + " " + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                </collision>\
                <visual name='front_wall_visual'>\
                <pose>" + std::to_string(WORLD_LEN/2 + ANTZ_HGT/2) + " 0 " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(ANTZ_HGT) + " " + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 0 0 1</ambient>\
                <diffuse>0 0 0 1</diffuse>\
                </material>\
                </visual>\
                \
                <collision name='back_wall_collision'>\
                <pose>" + std::to_string(- WORLD_LEN/2 - ANTZ_HGT/2) + " 0 " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(ANTZ_HGT) + " " + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                </collision>\
                <visual name='back_wall_visual'>\
                <pose>" + std::to_string(- WORLD_LEN/2 - ANTZ_HGT/2) + " 0 " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(ANTZ_HGT) + " " + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 0 0 1</ambient>\
                <diffuse>0 0 0 1</diffuse>\
                </material>\
                </visual>\
                \
                <collision name='right_wall_collision'>\
                <pose>0 " + std::to_string(WORLD_LEN/2 + ANTZ_HGT/2) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                </collision>\
                <visual name='right_wall_visual'>\
                <pose>0 " + std::to_string(WORLD_LEN/2 + ANTZ_HGT/2) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 0 0 1</ambient>\
                <diffuse>0 0 0 1</diffuse>\
                </material>\
                </visual>\
                \
                <collision name='left_wall_collision'>\
                <pose>0 " + std::to_string(- WORLD_LEN/2 - ANTZ_HGT/2) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                </collision>\
                <visual name='left_wall_visual'>\
                <pose>0 " + std::to_string(- WORLD_LEN/2 - ANTZ_HGT/2) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <geometry>\
                <box>\
                <size>" + std::to_string(WORLD_LEN) + " " + std::to_string(ANTZ_HGT) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 0 0 1</ambient>\
                <diffuse>0 0 0 1</diffuse>\
                </material>\
                </visual>\
                \
                <visual name='food_visual'>\
                <pose>" + std::to_string(FOOD_X) + " " + std::to_string(FOOD_Y) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <transparency>0.05</transparency>\
                <geometry>\
                <box>\
                <size>" + std::to_string(2*ANTZ_LEN) + " " + std::to_string(2*ANTZ_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 1 0 1</ambient>\
                <diffuse>0 1 0 1</diffuse>\
                </material>\
                </visual>\
                \
                <visual name='nest_visual'>\
                <pose>" + std::to_string(NEST_X) + " " + std::to_string(NEST_Y) + " " + std::to_string(ANTZ_HGT/2) + " 0 0 0</pose>\
                <transparency>0.05</transparency>\
                <geometry>\
                <box>\
                <size>" + std::to_string(2*ANTZ_LEN) + " " + std::to_string(2*ANTZ_LEN) + " " + std::to_string(ANTZ_HGT) + "</size>\
                </box>\
                </geometry>\
                <material>\
                <ambient>0 0 1 1</ambient>\
                <diffuse>0 0 1 1</diffuse>\
                </material>\
                </visual>\
                </link>\
                </model>\
                </sdf>");
    this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void Factory::OnUpdate(const common::UpdateInfo &_info)
{
    if (count >= ANTZ_COUNT)
    {
        //this->updateConnection = NULL;
        return;
    }
    else if (this->count == 0 || _info.simTime.sec - this->lastTime >= SPAWN_INTERVAL)
    {
        this->lastTime = _info.simTime.sec;
        
        msgs::Factory msg;
        msg.set_sdf("<?xml version='1.0'?>\
                    <sdf version='1.4'>\
                    <model name='antz" + std::to_string(this->count++) + "'>\
                    <link name='link'>\
                    <collision name='collision'>\
                    <geometry>\
                    <cylinder>\
                    <radius>" + std::to_string(ANTZ_LEN / 2) + "</radius>\
                    <length>" + std::to_string(ANTZ_HGT) + "</length>\
                    </cylinder>\
                    </geometry>\
                    </collision>\
                    <visual name='visual'>\
                    <geometry>\
                    <cylinder>\
                    <radius>" + std::to_string(ANTZ_LEN / 2) + "</radius>\
                    <length>" + std::to_string(ANTZ_HGT) + "</length>\
                    </cylinder>\
                    </geometry>\
                    <material>\
                    <ambient>1 0 0 1</ambient>\
                    <diffuse>1 0 0 1</diffuse>\
                    </material>\
                    <plugin name='colorful' filename='/Users/zxing/Dropbox/Gazebo/build/libcolorful.dylib'/>\
                    </visual>\
                    </link>\
                    <plugin name='antz' filename='/Users/zxing/Dropbox/Gazebo/build/libantz.dylib'/>\
                    </model>\
                    </sdf>");
        double x, y;
        x = NEST_X + math::Rand::GetDblUniform(-COMM_RANGE/2, COMM_RANGE/2);
        y = NEST_Y + math::Rand::GetDblUniform(-COMM_RANGE/2, COMM_RANGE/2);		
        msgs::Set(msg.mutable_pose(), math::Pose(math::Vector3(x, y, ANTZ_HGT / 2), math::Quaternion(0, 0, 0)));
        this->pub->Publish(msg);
    }
}
