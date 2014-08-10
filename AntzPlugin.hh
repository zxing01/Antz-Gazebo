#ifndef _GAZEBO_ANTZ_PLUGIN_HH_
#define _GAZEBO_ANTZ_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "Antz.hh"

namespace gazebo
{
    class Antz : public ModelPlugin
    {
    public:
        Antz();
        void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
        void OnWorldUpdate(const common::UpdateInfo &info);
    private:
        void Move();
        void Stop();
        void Turn(double direction);
        void PublishColorful();
        int RandomTarget(int minFoodCardinal, int minNestCardinal);
        double AngleFacing(double x, double y);
        
        void Walker(const common::UpdateInfo &info, int signalCount, int target);
        void Beacon(const common::UpdateInfo &info, int signalCount, int sameSignalCount, int minFoodCardinal, int minNestCardinal, bool isActive);
        void Explore(const common::UpdateInfo &info, int target);
        void Retrieve(const common::UpdateInfo &info, int target);
        void Avoid(const common::UpdateInfo &info, int target);
        void Revive();
        void StartExplore(const common::UpdateInfo &info);
        void StopExplore();
        void StartRetrieve(const common::UpdateInfo &info);
        void StopRetrieve();
        bool DetectObstacle(const common::UpdateInfo &info, int target);
        
        bool _isBeacon;
        bool _toNest;
        bool _shouldExplore;
        bool _shouldRetrieve;
        bool _shouldAvoid;
        int _id;
        int _foodCardinal;
        int _nestCardinal;
        int _exploreStartTime;
        int _retrieveStartTime;
        int _avoidStartTime;
        int _lastActiveTime;
        int _lastTarget;
        int _lastFoodCardinal;
        int _lastNestCardinal;
        math::Quaternion _orientation;
        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;
        transport::NodePtr _node;
        transport::PublisherPtr _pub;
        
        static int _antzID;
        static boost::tuple<int*, int*, const math::Vector3*> _antz[ANTZ_COUNT];
        static math::Vector3 _nestPos;
        static math::Vector3 _foodPos;
        static unsigned long _foodReturned;
    };
}

#endif