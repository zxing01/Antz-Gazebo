#ifndef _GAZEBO_ANTZ_PLUGIN_HH_
#define _GAZEBO_ANTZ_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "Antz.hh"
#include <unordered_map>
#include <map>
#include <vector>

namespace gazebo {
    class Antz : public ModelPlugin {
    public:
        Antz();
        void Load(physics::ModelPtr parent, sdf::ElementPtr /*sdf*/);
        void OnWorldUpdate(const common::UpdateInfo &info);
    private:
        void Move(double speedRatio = 1);
        void Stop();
        void TurnRandom();
        void Turn(double direction);
        void PublishColorful();
        int RandomTarget(int minFoodCardinal, int minNestCardinal);
        double AngleFacing(double x, double y);
        
        void Walker(const common::UpdateInfo &info, int signalCount, int target);
        void Beacon(const common::UpdateInfo &info, int signalCount, int sameSignalCount, int minFoodCardinal, int minNestCardinal, int minFoodSource, int minNestSource, bool isActive);
        void Explore(const common::UpdateInfo &info);
        void Avoid(const common::UpdateInfo &info);
        void Revive();
        void StartExplore(const common::UpdateInfo &info);
        void StopExplore();
        bool DetectObstacle(const common::UpdateInfo &info);
        bool DetectTarget();
        
        bool _isBeacon;
        bool _toNest;
        bool _shouldExplore;
        bool _shouldAvoid;
        int _id;
        int _foodCardinal;
        int _nestCardinal;
        int _exploreStartTime;
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
        
        // coordinated search
        bool _foundFood;
        int _frequency;
        int _lastMinFreq;
        std::map<int, std::vector<int> > _freqValues;
        std::unordered_map<int, int> _lastPresenceTime;
        
        // congestion
        double _maxFoodAdjCrowdFactor;
        double _foodCrowdFactor;
        double _foodOverallFactor;
        double _maxNestAdjCrowdFactor;
        double _nestCrowdFactor;
        double _nestOverallFactor;
        
        static int _antzID;
        static boost::tuple<int*, int*, const math::Vector3*, int*, double*, double*, double*, double*, double*, double*> _antz[ANTZ_COUNT];
        static math::Vector3 _nestPos;
        static math::Vector3 _foodPos;
        static unsigned long _foodReturned;
    };
}

#endif