#include <actionlib/server/simple_action_server.h>
#include <multi_map_nav/NavToGoalAction.h>
#include "wormhole_manager.h"
#include "map_switcher.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <ros/package.h>

using namespace std;
using namespace ros;

class NavigationServer {
public:
  NavigationServer(const string& db_path);
  void execute(const multi_map_nav::NavToGoalGoalConstPtr& goal);

private:
  NodeHandle nh_;
  actionlib::SimpleActionServer<multi_map_nav::NavToGoalAction> as_;
  WormholeManager wormhole_manager_;
  MapSwitcher map_switcher_;
  string current_map_;
    
  bool move_base_to(double x, double y, double yaw);

};
