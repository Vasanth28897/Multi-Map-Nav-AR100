# Multi-Map-Nav-AR100

## AR100 
The robot used here is the AR100 robot. 

### World and Map creation
A new world is created `simple_world.world` and the map also generated and saved under the `/maps`  directory in the AR100 package.

## Files to launch 

#### 1. Launch the robot in the simple_world

```sh
roslaunch start_anscer start_anscer.launch

```

#### 2. Launch the navigation 

```sh
roslaunch anscer_navigation anscer_navigation.launch

```

## Multi_Map_Nav
This package is for navigating the robot to separate mapped rooms. Each map is mapped in separate sessions, and saved in the `/maps` directory in the Multi_Map_Nav package. Wormhole mechanism allows the robot to switch between maps. 

#### **Key Features**

- **Switching maps based on Wormhole**
- **SQLite database** to store the map connection coordinates.
- **Dynamic Map Switching** using `map_server`
- **ROS Action Server** for receiving goals
- **move_base Integration** for local navigation

## Scenario Description
Imagine a robot in am office with multi_rooms:

- Each room is mapped separately (e.g., reception = `map1`, office = `map2`, meeting room = `map3`).
- A user requests the robot to go from reception (`map1`) to meeting room (`map3`).
- The robot:
  - Identifies the wormhole path (map1 → map2 → map3)
  - Switches maps as needed
  - Continues navigation across maps

## System Architecture

The system consists of three main components that work together to enable seamless navigation across multiple maps:

### 1. WormholeManager

- Connects to the SQLite database
- Retrieves wormhole coordinates between maps
- Handles queries to find paths between maps

### 2. MapSwitcher

- Loads map YAML files from the specified directory
- Launches map_server with the appropriate map
- Manages the transition between different environments

### 3. NavigationServer

- Implements ROS action server for handling navigation goals
- Orchestrates the entire navigation process
- Implements navigation logic for direct and indirect paths
- Utilizes move_base for actual robot movement

```sh
multi_map_nav
.
├── action
│   └── NavToGoal.action
├── database
│   └── wormholes.db
├── include
│   └── multi_map_nav
│       ├── map_switcher.h
│       ├── navigation_server.h
│       └── wormhole_manager.h
├── launch
│   └── multi_map_nav.launch
├── maps
│   ├── map1.pgm
│   ├── map1.yaml
│   ├── map2.pgm
│   ├── map2.yaml
│   ├── map3.pgm
│   └── map3.yaml
├── src
│    ├── map_switcher.cpp
│    ├── navigation_server.cpp
│    └── wormhole_manager.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## How It Works:

### Wormhole Concept

A **wormhole** is a defined position in one map that links to a corresponding position in another map—usually an overlapping area such as a doorway or hallway.

#### Wormhole Transition Steps:

1. Navigate to the wormhole in the current map.
2. Stop the map_server.
3. Launch map_server with the target map.
4. Teleport the robot to the corresponding wormhole in the new map.
5. Resume navigation toward the final goal.

### Navigation Process – Step by Step

#### 1. Goal Receive

The system receives a navigation request specifying:

- Target position (x, y)

- Target map (map_name)

#### 2. Current Map Evaluation

- If the target is within the current map:

- Directly forward the goal to move_base

- If the target is in another map:

- Initiate multi-map path planning

#### 3. Path Planning Logic

1. Direct Path

- Check for a direct wormhole from the current map to the target map.

- If found:

  - Navigate to the wormhole position

  - Switch to the destination map

  - Continue to the target position

2. Indirect Path (via Central Hub)

- If no direct path exists:

  - Navigate to a wormhole leading to the central map (map2)

  - Switch to map2

  - Navigate to a wormhole connecting map2 to the target map

  - Switch to the target map

  - Navigate to the final target position

#### 3. Move Base Integration

- For each segment:

  - Send a goal to move_base

  - Wait for the result (success/failure)

  - If successful, proceed to the next stage

  - If failed, abort with an error message

## Database Structure

The wormhole connections are stored in a SQLite database with the following schema:

```sql
CREATE TABLE wormholes_coord (
    from_map TEXT,  -- Source map name
    to_map TEXT,    -- Destination map name
    from_x REAL,    -- X-coordinate in source map
    from_y REAL     -- Y-coordinate in source map
);
```

Current wormhole connections:

```sql
INSERT INTO wormholes_coord VALUES ('map1', 'map2', 2.7, 4.9);
INSERT INTO wormholes_coord VALUES ('map2', 'map1', 2.7, 4.9);
INSERT INTO wormholes_coord VALUES ('map2', 'map3', -12.3, 6.9);
INSERT INTO wormholes_coord VALUES ('map3', 'map2', -12.3, 6.9);
```

####  Inspecting the Wormhole Database

You can view the stored wormhole connections using the sqlite3 command-line tool.

-  Steps to Open and Query the Database:

  ```sh
  sqlite3 wormholes.db
  ```

- Once inside the SQLite prompt, run:
  ```sh
  SELECT * FROM wormholes_coord;
  ```
-  Sample Output:
  ```sh
  map1|map2|2.72|4.97
  map2|map1|2.72|4.97
  map2|map3|12.38|6.9
  map3|map2|12.38|6.9

  ```
  > Each row represents a wormhole connection:

```sh
from_map | to_map | from_x | from_y
```

---

## Action Definition

NavToGoal.action:

```
# Request
float64 target_x
float64 target_y
string target_map
---
# Result
bool success
string message
---
# Feedback
string feedback_msg
```
---

# Setup and Usage

#### 1. Launch the multi_map_nav along with other launch files mentioned above:

```sh
roslaunch multi_map_nav multi_map_nav.launch

```

#### 2. Send a navigation goal: 

You can send navigation goals using rostopic:

```sh
rostopic pub /navigate_to_goal/goal multi_map_nav/NavToGoalActionGoal "goal:
target_x: 1.5
target_y: 9.0
target_map: 'map1'"
```





