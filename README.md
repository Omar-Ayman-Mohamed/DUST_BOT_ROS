# Dustbot - Autonomous Garbage Collection Robot

A ROS2-based simulation of an autonomous mobile robot navigating a 2D grid to collect garbage at specified locations.

##  Project Overview

Dustbot 2.0 is a mobile robot that navigates an NxN grid environment, autonomously collecting garbage at random locations. The system consists of two main ROS2 nodes communicating via topics and services:

- **World Node**: Simulates the environment, physics, and garbage spawning
- **Robot Node**: Implements the navigation logic and decision-making



### System Design

```
┌─────────────────┐         Topics          ┌─────────────────┐
│                 │ ──────────────────────> │                 │
│   World Node    │  /dustbot/global_pos    │   Robot Node    │
│  (Simulation)   │  /dustbot/garbage_pos   │   (Control)     │
│                 │ <────────────────────── │                 │
└─────────────────┘       Services          └─────────────────┘
                    /dustbot/set_direction
                    /dustbot/load_garbage
```

### Node Responsibilities

#### World Node (`world_node.py`)
- Maintains the NxN grid environment
- Simulates robot physics (1 cell/second movement)
- Publishes robot position at 1 Hz
- Spawns garbage at random locations
- Provides services for direction control and pickup validation
- Handles collision detection (walls)

#### Robot Node (`robot_node.py`)
- Subscribes to position and garbage location topics
- Implements path planning (greedy Manhattan distance)
- Calls services to change direction and pick up garbage
- Manages state machine (searching, moving, picking up)


### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- Ubuntu 22.04 (recommended)

### Package Structure

```
your_workspace/
├── src/
│   ├── dustbot/
│   │   ├── dustbot/
│   │   │   ├── __init__.py
│   │   │   ├── robot_node.py
│   │   │   └── world_node.py
│   │   ├── launch/
│   │   │   └── dustbot_launch.py
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── README.md (this file)
│   │
│   └── dustbot_interface/
│       ├── msg/
│       │   └── GarbagePosition.msg
│       ├── srv/
│       │   ├── SetDirection.srv
│       │   └── LoadGarbage.srv
│       ├── CMakeLists.txt
│       └── package.xml
```

### Installation

1. **Create ROS2 workspace** (if not already created):
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Place the packages** in the `src` directory:
   - `dustbot/` (main package)
   - `dustbot_interface/` (messages/services)

3. **Build the packages**:
```bash
cd ~/ros2_ws
colcon build --packages-select dustbot_interface dustbot
```

4. **Source the workspace**:
```bash
source install/setup.bash
```

##  Running the Simulation

### Basic Launch

Run with default parameters (10x10 grid, 5 pickups):

```bash
ros2 launch dustbot dustbot_launch.py
```

### Custom Parameters

**Method 1: Edit Launch File**

Edit `launch/dustbot_launch.py`:
```python
parameters=[
    {'n_grid': 15},      # Change grid size
    {'p_pickups': 8}     # Change number of pickups
]
```

**Method 2: Command Line Override**
```bash
ros2 launch dustbot dustbot_launch.py n_grid:=20 p_pickups:=10
```

##  Expected Output

"https://i.ibb.co/j2WgGYT/Screenshot-from-2026-02-14-20-31-45.png" 

##  Design Decisions & Implementation Details

### Navigation Algorithm

1. Calculate horizontal distance: `Δx = target_x - robot_x`
2. Calculate vertical distance: `Δy = target_y - robot_y`
3. Prioritize X-axis movement first, then Y-axis
4. Move one cell per second in the chosen direction

**Path Planning Priority:**
```python
if robot_x < target_x:  move East
elif robot_x > target_x:  move West
elif robot_y < target_y:  move North
elif robot_y > target_y:  move South
```


### Key Implementation Features

**Asynchronous Service Calls:**
- Uses `call_async()` with callbacks to avoid blocking
- Flags set immediately before service calls to prevent duplicates

**Clean Shutdown Protocol:**
- World sends kill signal (-100, -100) via garbage topic
- Robot acknowledges and shuts down gracefully
- 1-second delay before world node terminates

**Collision Handling:**
```python
# Wall boundaries
self.robot_x = max(0, min(self.robot_x, self.n - 1))
self.robot_y = max(0, min(self.robot_y, self.n - 1))
```


## Authors

- Omar Ismail



