# Machine Learning for Robotics

### 1. `metal_detection_marker` — Treasure Detection

Detects buried metallic objects from the `/vrep/metalDetector` sensor signal and publishes their estimated world-frame positions as RViz sphere markers.

**Algorithm:**

1. **Cluster extraction** — accumulates readings while signal `s ≥ threshold`. Short drops below threshold are tolerated for `cluster_timeout` seconds to avoid splitting a single detection into multiple clusters.
2. **Position estimation** — computes a weighted barycenter of sensor positions within the cluster. Signal strength is used as weight, so stronger readings pull the estimate closer to the true location.
3. **Zone merging** — when a cluster finalizes, its centroid is compared against all previously detected treasures. If it falls within `zone_radius` of an existing treasure the detection refines that treasure's position; otherwise a new treasure is registered.

**Topics:**

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/vrep/metalDetector` | `std_msgs/Float32` |
| Publish | `~/detected_object` | `visualization_msgs/Marker` |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_frame` | `world` | Fixed reference frame |
| `sensor_frame` | `VSV/Kision_sensor` | Metal detector TF frame |
| `threshold` | `0.7` | Minimum signal to start a cluster |
| `min_weight_to_publish` | `5.0` | Minimum accumulated weight to register a treasure |
| `cluster_timeout` | `0.5` s | Grace period before finalizing a cluster |
| `zone_radius` | `5.0` m | Radius within which detections are merged |

**Launch:**
```bash
ros2 launch metal_detection_marker fpr.launch.py
```

---

### 2a. `shore_follower_observe` + `shore_follower_drive_base` — Horizontal Arm Motion

Controls the arm's left/right position to keep the metal detector centered on the terrain interface (shore edge).

**Approach:** Deep convolutional neural network trained on 32×32 images from the arm camera.

**Training:**

| Stage | Details |
|-------|---------|
| Classes | Left, Straight, Right |
| Images per class | 1500 |
| Training set | 4500 images |
| Validation set | 3000 images |
| Accuracy | 60–70% |

**Observer** (`shore_follower_observe`) — collects labeled images via joystick. Records arm twist command (`/arm_ik/twist` linear.x) and associates it with the camera image at that moment. Saves up to `max_image_per_type` images per class, skipping frames with insufficient displacement.

**Drive** (`shore_follower_drive_base`) — runs the trained model in real time. Maps predicted class to arm velocity on `/arm_ik/twist` linear.x:
- **Left** → positive speed
- **Straight** → 0
- **Right** → negative speed

**Topics:**

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `/vrep/kision/image` | `sensor_msgs/Image` |
| Subscribe | `/joy` | `sensor_msgs/Joy` |
| Publish | `/arm_ik/twist` | `geometry_msgs/Twist` (linear.x) |

**Observer launch:**
```bash
ros2 launch shore_follower_observe record.launch.py
```

**Train model:**
```bash
bash shore_follower_drive_base/scripts/train_shore_follower.sh
```

**Drive launch:**
```bash
ros2 launch shore_follower_drive_base start.launch.py
```

---

### 2b. `shore_follower_observe2` + `shore_follower_drive_base2` — Vertical Arm Motion

Controls the arm's up/down position to keep the metal detector at a consistent height above the terrain.

Same architecture as Objective 2a but trained on the vertical axis.

**Training:**

| Stage | Details |
|-------|---------|
| Classes | Up, Straight, Down |
| Images per class | 1500 |
| Training set | 4500 images |
| Validation set | 3000 images |
| Accuracy | 80–90% |
| Filter | Only frames with minimum XY or Z displacement are kept |

**Drive** maps predicted class to `/arm_ik/twist` linear.z:
- **Up** → positive speed
- **Straight** → 0
- **Down** → negative speed

**Observer launch:**
```bash
ros2 launch shore_follower_observe2 record.launch.py
```

**Drive launch:**
```bash
ros2 launch shore_follower_drive_base2 start.launch.py
```

---

### 3. `floor_follower` — Shore Following

Steers the truck to follow the shoreline using color segmentation on the arm camera image.

**Algorithm:**

1. Extract the bottom 30% of the image as the region of interest (ROI).
2. Convert ROI to HSV color space.
3. Generate masks:
   - **Grey** (floor/sand): HSV `[0,0,50]` – `[180,40,200]`
   - **Green** (vegetation/non-floor): HSV `[35,40,40]` – `[85,255,255]`
4. Count grey vs green pixels to determine which side has more floor.
5. Publish velocity:
   - Always move forward at `forward_speed`.
   - More grey pixels → turn left (`-turn_speed`).
   - More green pixels → turn right (`+turn_speed`).

**Topics:**

| Direction | Topic | Type |
|-----------|-------|------|
| Subscribe | `image` | `sensor_msgs/Image` |
| Publish | `twist` | `geometry_msgs/Twist` |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `forward_speed` | `0.3` | Linear forward velocity (m/s) |
| `turn_speed` | `0.1` | Angular turning rate (rad/s) |

**Launch:**
```bash
ros2 launch floor_follower start.launch.py
```

---

## Building

The packages in this directory are excluded from the workspace build by `COLCON_IGNORE`. To build them, remove that file first:

```bash
cd ~/ML4R/ros2_ws/src/Final\ Project
rm COLCON_IGNORE
cd ~/ML4R/ros2_ws
source /opt/ros/kilted/setup.bash
colcon build --symlink-install --packages-select \
    metal_detection_marker \
    shore_follower_observe shore_follower_drive_base \
    shore_follower_observe2 shore_follower_drive_base2 \
    floor_follower
source install/setup.bash
```

---

## Known Limitations & Future Work

| Objective | Limitation | Suggested Improvement |
|-----------|------------|-----------------------|
| 1 — Treasure Detection | Barycenter estimate drifts with sensor noise | Use inverse sensor model + circle-voting (Hough-style) for more accurate localization |
| 2a — Horizontal Arm | 60–70% accuracy | Improve data collection diversity; more balanced class distribution |
| 2b — Vertical Arm | ML model can be brittle near limits | Replace with point-cloud range check + PID controller for robust height keeping |
| 3 — Shore Following | Color segmentation fails under changing lighting | Train a CNN to follow the shore (like the arm model), or use a different sensor |
