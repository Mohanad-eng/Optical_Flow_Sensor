# Optical_Flow_Sensor
Repo For Optical Flow Sensor

![Optical Flow](Optical_flow.jpg)

The ROS 2 Optical Flow Sensor package provides **real-time motion estimation** by analyzing the **displacement of visual features between consecutive camera frames**. Using computer vision techniques implemented with OpenCV, the package calculates optical flow vectors and publishes the resulting motion estimates as ROS 2 topics.

Optical flow is widely used in robotics to estimate the relative movement of a robot or camera without relying solely on GPS or wheel encoders. This package can be integrated into mobile robots, drones, autonomous vehicles, and research platforms requiring visual motion estimation.

***How to use it:*** 

The functionality of an optical flow sensor of course depends on being able to **find features to track**, a surface that is very uniform will be hard to track since all the frames will look the same. If you’ve ever tried using a mouse on a glass table or reflective surface you’ve probably seen that it doesn’t work.

## How it works : 

The magic behind these sensors lies in their ability to analyze changes in the visual field. By comparing consecutive frames captured by the sensor’s camera, the system identifies distinct features and tracks their movement. This tracking process allows the sensor to calculate the velocity and direction of these features, which are then used to estimate the overall motion of the device. 

The sensor’s output is typically a set of vectors representing the direction and magnitude of the motion detected. This data can be further processed to provide more advanced information, such as the device’s altitude, orientation, and even its proximity to obstacles. 

![Optical Flow](flow.png)



## Factors Affecting the Sensor : 

The sensor’s effectiveness depends on several factors, including : 

1- the quality of the camera.

2- the processing algorithms used, and the environmental conditions.

3- poor lighting or rapid changes in the scene can introduce noise and inaccuracies in the sensor’s output(We use a light rall to make good lighting).

## Installing the Packages : 

First Inside Any workspace you have go inside it : 

`cd ~/ros2_ws/src`

Then clone the Reposetriy : 

`git clone https://github.com/adityakamath/optical_flow_ros.git`

`cd ..`

Then Build : 

`colcon build`

Then Source : 

`source install/setup.bash`

## Messeage type : 


## Simulation : 

# PWM3901 Optical Flow — ROS 2 Jazzy

Simulated **and** real-hardware support for the PixArt **PMW3901** optical
flow sensor (commonly mislabeled "PWM3901"; same chip as on the Bitcraze
Flow Deck v2) on a differential-drive robot, targeting **ROS 2 Jazzy** with
**Gazebo Harmonic** and **Webots**.

```
.
├── ros2_ws/src/
│   ├── diffbot_description/      # robot model, Gazebo plugins, launch files
│   ├── diffbot_optical_flow/     # SIMULATED PWM3901 (math model, no SPI/camera)
│   └── pwm3901_driver/           # REAL HARDWARE PMW3901 SPI driver + node
├── webots/                       # Webots world + controller (same sensor model)
└── docs/
```

The simulated and real driver nodes publish on **identical topics with
identical message types**, so any consumer (state estimator, velocity
controller, logger) is sim/hardware-agnostic:

| Topic | Type | Meaning |
|---|---|---|
| `/pwm3901/flow_raw` | `geometry_msgs/Vector3Stamped` | uncompensated optical flow (x, y in rad/s), z = quality 0–1 |
| `/pwm3901/flow_comp` | `geometry_msgs/Vector3Stamped` | gyro-rate-compensated flow (translation only) |
| `/pwm3901/velocity_estimate` | `geometry_msgs/TwistStamped` | recovered body-frame linear velocity (m/s) |

---

## 1. What the PMW3901/PWM3901 actually is

It's a **20×20 monochrome image sensor + DSP** in one chip, originally
designed for optical computer mice, repurposed for downward-facing
drone/robot navigation. It does **not** give you velocity or distance — it
gives you raw pixel displacement (Δx, Δy in counts) between consecutive
internal frames at up to ~120 Hz.

Key facts:

- **Interface:** SPI only (mode 3), no I²C variant.
- **FOV:** ~42° (varies by lens), giving roughly **0.0098 rad/pixel** as a
  typical starting calibration constant — but this drifts per unit/lens and
  should be calibrated (see §6).
- **Output:** raw Δx/Δy pixel counts + a `SQUAL` (surface quality, 0–255)
  byte that tells you how trustworthy the reading is (textured carpet ≈
  good, glossy/uniform floor ≈ bad/garbage).
- **It cannot give you velocity by itself.** Pixel flow is proportional to
  `velocity / height`, so you must pair it with a **downward rangefinder**
  (VL53L1X, VL53L0X, ultrasonic, barometer-as-fallback) to recover actual
  speed. This is why the Bitcraze Flow Deck bundles a VL53L1X right next to
  the PMW3901 — it's not optional, it's structural to how the sensor works.
- **It also can't distinguish translation from rotation.** Spinning in
  place produces flow identical in form to translating sideways. Real
  systems subtract gyro rate to compensate (see math below) — this is
  exactly what PX4's `optical_flow` driver and ArduPilot's `AP_OpticalFlow`
  do internally.

## 2. The core math

```
flow_x_raw (rad/s) =  vy / h  -  wz      # raw, rotation-contaminated
flow_y_raw (rad/s) = -vx / h  -  wz

flow_x_comp = flow_x_raw + wz            # after subtracting gyro rate
flow_y_comp = flow_y_raw + wz

vx_estimate = -flow_y_comp * h
vy_estimate =  flow_x_comp * h
```

Where `h` is height above the surface (meters), `vx, vy` are body-frame
linear velocities, `wz` is body yaw rate (rad/s) from a gyro/IMU. Sign
conventions vary by mounting orientation — verify against your own
`flowdeck_link` axes before trusting an estimate.

This is exactly what both nodes in this repo implement: the simulator runs
it forward (ground-truth velocity → synthetic flow), the real driver runs
it in reverse (raw pixel counts → recovered velocity).

## 3. Repo packages

### `diffbot_optical_flow` (simulation)
Subscribes to Gazebo ground-truth odometry (`/diffbot/odom`) and a
downward altimeter (`/diffbot/height`), and synthesizes sensor-realistic
flow output: angular-rate noise, height noise, a saturation limit
(`max_speed_rad_s`), and a quality score that degrades out of valid range
or near saturation — so anything you build against this data has to
tolerate the same imperfections a real chip would produce.

### `pwm3901_driver` (real hardware)
- `pmw3901_spi.py` — low-level SPI register driver: power-up reset, product
  ID check (`0x49`), the PixArt-recommended init register sequence, and a
  `read_motion_burst()` call that pulls Δx/Δy/SQUAL/shutter in one
  transaction (avoids tearing data across two internal frames).
- `pmw3901_node.py` — ROS 2 node that polls the chip, converts raw counts
  to rad/s using a calibrated `rad_per_count`, fuses with `/imu/data` for
  gyro compensation and `/rangefinder` for height, and republishes on the
  same topic names as the simulator.

### `diffbot_description`
URDF/xacro differential-drive robot, `gz-sim` (Gazebo Harmonic) DiffDrive +
OdometryPublisher plugins, a `flowdeck_link` mount point, and a launch file
wiring robot_state_publisher → Gazebo → spawn → ros_gz_bridge → the
simulated flow node.

### `webots/`
Equivalent Webots `Robot` controller: reads wheel encoder deltas + a
`Gyro` device + a downward `DistanceSensor` for height, and runs the same
math model. Swap the `publish()` stub for an `Emitter` or `webots_ros2`
topic publisher to get it into ROS 2 proper.

---

## 4. Wiring the real chip (Raspberry Pi example)

| PMW3901 pin | Pi pin |
|---|---|
| VCC | 3V3 (pin 1) |
| GND | GND (pin 6) |
| MOSI | GPIO10 / pin 19 |
| MISO | GPIO9 / pin 21 |
| SCK | GPIO11 / pin 23 |
| CS | GPIO8 (CE0) / pin 24 |

Enable SPI (`sudo raspi-config` → Interface Options → SPI), then
`pip install spidev --break-system-packages` (or via your workspace venv).

Pair with a VL53L1X/VL53L0X on I²C for height, publishing a
`sensor_msgs/Range` on `/rangefinder` — there are existing ROS 2 drivers
for these (search for `vl53l1x_ros2` or similar) you can drop straight in
since `pmw3901_node.py` only expects the standard message type.

## 5. Build & run

```bash
# --- ROS2 Jazzy + Gazebo Harmonic simulation ---
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch diffbot_description diffbot_gazebo.launch.py

# in another terminal, drive it and watch flow output:
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" -r 10
ros2 topic echo /pwm3901/velocity_estimate

# --- real hardware (on the robot's onboard computer) ---
ros2 run pwm3901_driver pmw3901_node --ros-args -p rad_per_count:=0.0098
```

```bash
# --- Webots ---
# Open webots/worlds/*.wbt, set the controller field on your robot node to
# "diffbot_controller", and press play. Console prints flow output directly.
```

## 6. Calibrating `rad_per_count` on real hardware

The default `0.0098 rad/pixel` is a reasonable starting point but **drifts
with lens variant and mounting**. To calibrate:

1. Mount the sensor at a precisely known, fixed height `h`.
2. Move the robot a known straight-line distance `d` over `t` seconds (so
   `v = d/t` is known ground truth — a tape measure + stopwatch, or a
   motion-capture rig if you have one).
3. Log raw `delta_x`/`delta_y` from `read_motion_burst()` over the same
   interval and sum them.
4. Solve for `rad_per_count` from `flow_comp * h = v` using the summed
   counts and known `dt`, then set it as a node parameter.

Repeat at 2–3 different heights to confirm linearity before trusting it.

## 7. Fusing into actual state estimation

Optical flow velocity alone drifts (it's a velocity source, not position —
errors integrate). In practice you'd feed `/pwm3901/velocity_estimate`
(gated by the `quality`/SQUAL field — discard low-quality samples) into an
EKF alongside wheel odometry and an IMU, e.g. `robot_localization`'s `ekf_node`,
treating it as a `twist` input with quality-dependent covariance. This repo
intentionally stops at "raw + compensated + velocity" output and leaves
fusion to you, since covariance tuning is application-specific.

## 8. Known limitations / gotchas

- **Lighting matters.** PMW3901 needs enough ambient/IR light; in very dark
  environments SQUAL degrades hard. The simulator doesn't model this —
  add a lighting-quality term yourself if it matters for your use case.
- **Smooth/glossy/uniform surfaces** (polished tile, plain white paper)
  starve the chip of trackable texture — same caveat applies.
- **Height range:** PMW3901 is roughly reliable from ~8 cm to a few meters,
  worse the higher you go (less apparent pixel motion per unit velocity).
  Both nodes enforce `min_height_m`/`max_height_m` quality gating.
- **SPI mode 3, not mode 0** — the most common real-hardware bring-up bug.




