#include "multi_map_nav/map_switcher.h"


MapSwitcher::MapSwitcher() : nh_()
{
  nh_.param<string>("map_folder", map_folder_,
                          package::getPath("multi_map_nav") + "/maps");

  ROS_INFO("Map switcher initialized with map folder: %s", map_folder_.c_str());
  if (map_folder_.empty())
  {
    ROS_ERROR("Map folder is not set. Please set the 'map_folder' parameter.");
    return;
  }
}

void MapSwitcher::switchToMap(const string &map_name)
{
  string map_yaml_path = map_folder_ + "/" + map_name + ".yaml";

  ifstream map_file(map_yaml_path);
  if (!map_file.is_open())
  {
    ROS_ERROR("Failed to open map file: %s", map_yaml_path.c_str());
    return;
  }

  string line;
  bool valid = false;
  while (getline(map_file, line))
  {
    if (line.find("image:") != string::npos)
    {
      valid = true;
      break;
    }
  }

  if (!valid)
  {
    ROS_ERROR("Invalid map file format: %s", map_yaml_path.c_str());
    return;
  }

  map_file.close();

  string command = "rosrun map_server map_server " + map_yaml_path;
  
  int result = system((command + " &").c_str());
  Duration(1.0).sleep();

  if (result == 0)
  {
    ROS_INFO("Successfully switched to map: %s", map_name.c_str());
  }
  else
  {
    ROS_ERROR("Failed to switch to map: %s", map_name.c_str());
  }
}
