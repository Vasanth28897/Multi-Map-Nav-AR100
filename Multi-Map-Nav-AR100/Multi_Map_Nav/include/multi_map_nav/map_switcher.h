#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/Empty.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <fstream>

using namespace std;
using namespace ros;

class MapSwitcher {
public:
  MapSwitcher();
  void switchToMap(const string &map_name);

private:
  NodeHandle nh_;
  string map_folder_;
};