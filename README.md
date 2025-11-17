# <u>Autonomous Navigation and Obstacle Avoidance System for Differential Drive Robots</u>

A ROS2-based reactive obstacle avoidance system featuring autonomous exploration with 360° LiDAR analysis, state-machine decision logic, and robust recovery behaviors for differential drive mobile robots.

---

## <u>Project Overview</u>

This project implements a **reactive obstacle avoidance and autonomous exploration** system for differential drive robots operating in dynamic office environments. The robot navigates unknown spaces using real-time LiDAR sensor data, employing a stop-analyze-act decision pattern to safely avoid obstacles without requiring maps or prior knowledge of the environment.

### Core Capabilities:

- **Reactive obstacle avoidance** with 360° LiDAR analysis
- **Autonomous exploration** using finite state machine control
- **Stop-analyze-act decision pattern** for safe navigation
- **Multi-stage recovery behaviors** (reverse, progressive rotation)
- **Real-time performance monitoring** with camera feed logging
- **Zero map dependency** - operates in completely unknown environments

### Assignment Context:

This project addresses the core robotics challenge: **"How can a mobile robot safely navigate and explore an unknown environment using only onboard sensors?"**

The solution demonstrates fundamental concepts in:
- Sensor-based reactive control
- Behavior-based robotics
- Real-time decision-making under uncertainty
- Robust recovery from failure states

---

## <u>Test Environment</u>

### Robot Platform: Custom Differential Drive Robot

**Physical Specifications:**
- **Drive Configuration:** 4-wheel differential drive (2 driven wheels, 2 caster wheels)
- **Dimensions:** 0.5m (length) × 0.3m (width) × 0.15m (height)
- **Wheel Specifications:**
  - Radius: 0.1m
  - Width: 0.05m
  - Mass: 0.5kg per wheel
- **Body Mass:** 10.0kg
- **Wheel Base:** 0.3m (distance between driven wheels)
- **Wheel Offset:** 0.15m (from center to front/rear wheels)

**Kinematic Model:**

Differential Drive Equations:
- v = (v_left + v_right) / 2 # Linear velocity
- ω = (v_right - v_left) / L # Angular velocity

Constraints:

- Max linear velocity: 0.5 m/s
- Max angular velocity: 1.0 rad/s


**Why Differential Drive?**
- ✅ **Omnidirectional turning:** Can rotate in place for tight spaces
- ✅ **Simple kinematics:** Easier to control than Ackermann steering
- ✅ **Indoor optimization:** Ideal for office/warehouse environments
- ✅ **Cost-effective:** Fewer actuators than omnidirectional robots

---

## <u> Sensor Suite </u>

#### 1. 2D LiDAR Scanner

**Model:** Generic 2D LiDAR (Gazebo `ray` sensor)

**Specifications:**
- **Field of View:** 360° (full coverage)
- **Angular Resolution:** 1° (360 samples per scan)
- **Range:** 0.55m (minimum) to 10m (maximum)
- **Update Rate:** 10 Hz
- **ROS Topic:** `/scan` (sensor_msgs/LaserScan)

**Technical Parameters:**

<ray> <scan> <horizontal> <samples>360</samples> <resolution>1</resolution> <min_angle>0</min_angle> <max_angle>6.28319</max_angle> <!-- 2π radians --> </horizontal> </scan> <range> <min>0.55</min> <max>10.0</max> <resolution>0.01</resolution> </range> </ray> ```

**Why 2D LiDAR?**

✅ 360° coverage: No blind spots for obstacle detection

✅ Accurate ranging: Better than ultrasonic sensors (±1cm vs ±5cm)

✅ Fast update rate: Real-time responsiveness at 10 Hz

✅ Obstacle discrimination: Differentiates individual objects vs cameras

✅ Lighting independent: Works in dark environments

### 2. RGB Camera 

**Model:** Generic RGB camera (Gazebo camera sensor)

**Specifications:**
- **Resolution:** 640×480 pixels
- **Field of View:** 60° horizontal
- **Frame Rate:** 30 Hz
- **ROS Topic:** /camera/image_raw (sensor_msgs/Image)

**Mounting Position:**

- **Location:** Front of robot (0.22m forward from center)
- **Height:** 0.12m above base_link
- **Orientation:** Forward-facing

**Purpose in This Project:**

- **Performance monitoring:** Visual logging of robot's perspective
- **Debugging:** Frame-by-frame analysis of navigation decisions
- **Future expansion:** Potential for vision-based obstacle classification

## <u>Simulation Environment: Creative Office World </u>

**Dimensions:** 15m × 12m × 3m (length × width × height)

**Environment Type:** Indoor office space with typical furniture and obstacles

**World Components:**

**Floor**
- Material: Light gray polished surface (high friction for traction)
Friction coefficients: μ₁=100, μ₂=50

**Walls (4 boundary walls)**
- North/South walls: 15m × 0.2m × 3m
- East/West walls: 12m × 0.2m × 3m
Material: Off-white painted surface

**Furniture and Obstacles:**
- Reception Desk: 2.0m × 0.8m × 0.8m (front entrance)
- Work Desks (3x): 1.6m × 0.8m × 0.75m (open workspace)
- Conference Table: 2.5m × 1.2m × 0.75m (meeting room)
- Office Chairs (2x): 0.3m radius cylinders (desk areas)
- Filing Cabinets (2x): 0.5m × 0.6m × 1.0m (storage area)
- Coffee Counter: 1.2m × 0.6m × 0.9m (break area)
- Coffee Machine: 0.2m radius cylinder (on counter)
- Plants (2x): 0.25m radius cylinders (decorative)
- Whiteboard: 1.5m × 0.05m × 1.2m (wall-mounted)
- Meeting Room Dividers (2x): 4m × 0.15m × 2.4m (partial walls)

**Lighting:**

- Sun: Directional overhead light (simulates skylight)
- Office Light: Directional light at (5, 5, 8) position
- Ceiling Light: Point light at (4, 4, 3.5) position

**Why This Environment?**

✅ Realistic office scenario: Represents common deployment environment (warehouses, hospitals, offices)

✅ Diverse obstacle types:

Large static obstacles: Desks, cabinets (test basic avoidance)

Narrow passages: Between furniture rows (test precision navigation)

Clustered obstacles: Meeting room area (test recovery behaviors)

Mixed geometries: Boxes, cylinders, walls (test sensor coverage)

✅ Challenging navigation scenarios:

U-shaped obstacles: Meeting room corner (test local minima handling)

Dead-end corridors: Behind desks (test wall-following prevention)

Tight clearances: Chair-desk gaps (test minimum safe distance)

✅ Scalable complexity:

Can add/remove obstacles for difficulty adjustment

Easily modifiable for different test scenarios

**Environment Characteristics:**

| Metric               | Value        | Significance                 |
| -------------------- | ------------ | ---------------------------- |
| Open Area            | ~120 m²      | Sufficient for exploration   |
| Obstacle Density     | 15% coverage | Moderately cluttered         |
| Min Passage Width    | 0.8m         | Requires precise navigation  |
| Obstacle Height      | 0.75-2.4m    | Within LiDAR detection plane |
| Max Exploration Time | 5-8 minutes  | Reasonable demo duration     |

---

## <u> System Architecture </u>

### Component Breakdown

### 1. Sensor Processing Module

- **Input:** Raw LaserScan messages (360 range values)
- **Processing:** Sector aggregation, invalid data filtering
- **Output:** 8-sector minimum distance dictionary
- **Update Rate:** 10 Hz (synchronized with LiDAR)

### 2. State Machine Controller

- **Type:** Finite State Machine (5 states)
- **Transitions:** Rule-based on sensor readings and timers
- **Persistence:** State tracking with timestamps
- **Output:** Current behavior mode

### 3. Decision Logic Engine

- **Inputs:** Sector distances, current state, history
- **Algorithms:** Priority-based selection, pattern detection
- **Output:** Target direction and velocity commands
- **Fallbacks:** Progressive recovery hierarchy

### 4. Velocity Command Publisher

- **Output Topic:** /cmd_vel (geometry_msgs/Twist)
- **Command Types:** Linear X, Angular Z
- **Safety Limits:** Velocity clamping, emergency stops
- **Update Rate:** 10 Hz

## <u>Algorithm Selection & Implementation Rationale</u>

### Development Methodology

This project follows a systematic evaluation approach where each algorithmic component was selected through:

- Literature review of established obstacle avoidance algorithms

- Comparative analysis of complexity vs performance tradeoffs

- Empirical testing in simulated scenarios

- Iterative refinement based on observed failure modes

### Phase 1: Why Reactive Approach?

Assignment Context
The assignment emphasizes real-time obstacle avoidance in dynamic environments where:

- Global maps are unavailable (unexplored spaces)

- Localization may be unreliable (odometry drift)

- Obstacles can move or change (dynamic environment)

- Computational resources are limited (embedded systems)

### Reactive vs Planning Approaches

| Criterion                 | Reactive Approach      | Planning Approach (e.g., RRT\*, A\*) |
| ------------------------- | ---------------------- | ------------------------------------ |
| Map Requirement           | None                   | Global map required                  |
| Computation Time          | <1ms per cycle         | 100-1000ms per plan                  |
| Response Latency          | Immediate (10 Hz)      | Delayed (replanning lag)             |
| Memory Usage              | Minimal (~1KB)         | Significant (map storage)            |
| Dynamic Obstacles         | Handles naturally      | Requires frequent replanning         |
| Optimality                | Suboptimal paths       | Globally optimal paths               |
| Implementation Complexity | Low (300 lines)        | High (2000+ lines)                   |
| Reliability               | Robust to sensor noise | Sensitive to map errors              |

**Decision** - Reactive approach selected for robustness, simplicity, and real-time performance

### Phase 2: Obstacle Avoidance Algorithm Comparison

We conducted a comprehensive survey of reactive obstacle avoidance algorithms:

| Algorithm        | Complexity | Reliability | Local Minima Risk | Implementation Effort | Exploration Capability | Selected? |
| ---------------- | ---------- | ----------- | ----------------- | --------------------- | ---------------------- | --------- |
| Bug Algorithms   | ⭐⭐         | ⭐⭐⭐         | Medium            | ⭐⭐⭐                   | ❌ (goal-required)      | ❌         |
| Potential Fields | ⭐⭐         | ⭐⭐          | High              | ⭐⭐⭐⭐                  | ❌ (goal-required)      | ❌         |
| DWA              | ⭐⭐⭐⭐⭐      | ⭐⭐⭐⭐        | Low               | ⭐⭐                    | ⚠️ (complex)           | ❌         |
| VFH              | ⭐⭐⭐⭐       | ⭐⭐⭐⭐        | Low               | ⭐⭐⭐                   | ✅                      | ✅Inspired |
| Simple Reactive  | ⭐          | ⭐⭐          | Medium            | ⭐⭐⭐⭐⭐                 | ✅                      | ✅Base     |

**Decision** - Developed a hybrid Simple Reactive + VFH-Inspired system that combines:

- Simplicity of reactive control (maintainability, debuggability)

- Robustness of VFH sector analysis (360° awareness, gap detection)

- Explicit state management (predictable behavior, recovery logic)

Result: A system that is simple enough to understand and debug yet robust enough to handle complex office environments. 

--- 

## <u> State Transition Logic Summary </u>

| Current State | Condition                         | Next State |
| ------------- | --------------------------------- | ---------- |
| EXPLORING     | front < 0.8m                      | STOPPED    |
| STOPPED       | wait 0.3s                         | ANALYZING  |
| ANALYZING     | clearance > 0.6m                  | TURNING    |
| ANALYZING     | no forward clearance, back > 0.5m | REVERSING  |
| ANALYZING     | all blocked                       | RECOVERY   |
| TURNING       | turn complete, front clear        | EXPLORING  |
| TURNING       | turn complete, front blocked      | ANALYZING  |
| REVERSING     | back blocked                      | RECOVERY   |
| REVERSING     | 2s elapsed                        | ANALYZING  |
| RECOVERY      | clearance found                   | TURNING    |
| RECOVERY      | 360° complete                     | EXPLORING  |

---

## <u>Setup and Execution</u>

**Disclaimer** This package requires ROS2 Humble and Gazebo Classic. Make sure these are installed on the system.

### Dependencies

**Python dependencies**
```
pip3 install numpy opencv-python
```
**ROS-CV bridge**
```
sudo apt install ros-humble-cv-bridge
```
### Setup

Create a ROS2 workspace and then clone the repository.

- Clone repository

```
git clone https://github.com/AK1902-a11y/Obstacle-Avoidance-and-Navigation-.git

```
- Build the workspace
```
cd ~/<your_ros2_ws>

colcon build 
```

- Source the workspace 

```
# Execute from the root of workspace

source install/setup.bash 

```

### Execution 

Launch the Simulation Environment and Obstacle Avoidance Node 

```

ros2 launch robot_obstacle_avoidance gazebo_sim.launch.py 

```
Launch the Simulation Environment and Obstacle Avoidance Node with tuneable parameters 

```

ros2 launch robot_obstacle_avoidance gazebo_sim.launch.py \
    obstacle_threshold:=0.7 \
    min_safe_distance:=0.5 \
    forward_velocity:=0.4 \
    turn_velocity:=0.6

``` 

**Available Parameters to modify**

| Parameter          | Default Value | Description                        | Valid Range |
| ------------------ | ------------- | ---------------------------------- | ----------- |
| use_sim_time       | true          | Use Gazebo simulation time         | true/false  |
| obstacle_threshold | 0.8           | Distance to stop for obstacles (m) | 0.5-2.0     |
| min_safe_distance  | 0.6           | Minimum safe clearance (m)         | 0.4-1.0     |
| forward_velocity   | 0.3           | Forward exploration speed (m/s)    | 0.1-0.8     |
| turn_velocity      | 0.5           | Angular turning speed (rad/s)      | 0.2-1.2     |

---

# BONUS POINTS

## <u>Future Scope and Improvements</u>

### Advanced Path Planning Pipeline (Partial Implementation)

To demonstrate the extensibility of this reactive system, implemented an advanced path planning pipeline consisting of three integrated modules:

### Implemented Components:
**1. Path Smoothing (Clothoid Curves)**

- Converts discrete waypoints into smooth, G²-continuous paths

- Uses clothoid (Euler spiral) curves optimized for differential drive kinematics

- Generates curvature-aware paths that minimize lateral acceleration

**2. Trajectory Generation (Forward-Backward Algorithm)**

- Time-parameterizes smoothed paths with velocity profiles

- Respects kinematic constraints (v_max, a_max, a_lat_max)

- Adapts velocity based on path curvature (slower on turns, faster on straights)

**3. Regulated Pure Pursuit (RPP) Controller**

- Adaptive lookahead distance proportional to velocity

- Curvature-based velocity regulation for smooth cornering

- Goal approach behavior with gradual deceleration

### Current Pipeline Flow:

Waypoints → Clothoid Smoothing → Trajectory Generation → RPP Controller → Robot Motion

**Status:** Demonstrated as standalone modules with successful trajectory tracking in simulation.

### Remaining Integration: Collision Checker & Obstacle Avoidance Layer

RPP Controller → /cmd_vel_desired → Obstacle Avoidance Layer (LiDAR /scan) → /cmd_vel
                                         
### Planned Implementation:

**1. Collision Checker Module**

- Forward projection of planned trajectory

- Collision prediction using robot footprint and LiDAR data

- Real-time safety validation before command execution

**2. Obstacle Avoidance Layer**

- Monitors RPP-generated commands via /cmd_vel_desired

- Overrides commands when obstacles detected within safety threshold

- Applies reactive avoidance (sector-based analysis) while maintaining trajectory awareness

- Returns control to RPP once path is clear

**3. Integration Benefits:**

- Combines efficiency of planned paths with safety of reactive avoidance

- Handles static environments (offline planning) and dynamic obstacles (online reaction)

- Maintains trajectory tracking accuracy while ensuring collision-free motion

**Due to time constraint this system was not fully implemented.** The assignment focus was on demonstrating robust reactive obstacle avoidance, which has been successfully achieved. The path planning pipeline was developed to showcase:

Understanding of trajectory generation algorithms

Controller implementation skills

Modular software architecture design

Future Work: Complete integration of the collision checker and obstacle avoidance layer would create a production-ready hybrid navigation system suitable for real-world deployment in warehouse and service robotics applications.

### Execution for partial implementation 

Start the Simulation Environment , Path Smoothing and Trajectory Generation Node

```
ros2 launch robot_obstacle_avoidance gazebo_motion_planner.launch.py

```

Start the regulated pure pursuit controller node

```
ros2 run robot_obstacle_avoidance rpp_controller

```