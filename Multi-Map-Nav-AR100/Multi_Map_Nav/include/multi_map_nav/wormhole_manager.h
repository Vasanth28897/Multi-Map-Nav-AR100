#include <string>
#include <utility>
#include <sqlite3.h>
#include <ros/ros.h>

using namespace std;

class WormholeManager {
public:
  WormholeManager(const string &db_path);
  ~WormholeManager();
  pair<double, double> getWormholeToMap(const string &current_map, const string &target_map);

private:
  sqlite3* db_;
};