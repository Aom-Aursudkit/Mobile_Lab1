# Mobile_Lab1

## Implementation Details

### EKF Node (ekf_node.py)
- Computes wheel odometry from `/joint_states` using differential drive kinematics
- Fuses wheel odometry with IMU measurements (`/imu`) using a 1D Extended Kalman Filter on heading angle
- Publishes both raw wheel odometry and EKF-fused odometry

**Key Parameters:**
- Wheel radius: 0.033m
- Wheel base: 0.160m
- Process noise (Q): 0.01
- Measurement noise (R): 0.05

### ICP Node (icp_node.py)
- Uses EKF odometry as initial guess for scan matching
- Implements point-to-point ICP with SVD-based transformation estimation
- Spatial hash for fast nearest neighbor search
- Quality-based fallback: uses odometry when ICP match quality is poor
- Builds and maintains a global map with voxel filtering

**Key Parameters:**
- Max ICP iterations: 15
- Correspondence distance: 0.5m
- Convergence threshold: 1e-4
- Map size limit: 50,000 points

### SLAM Toolbox
- Performs scan matching with pose graph optimization
- Outputs globally consistent map and pose estimates

## Results Comparison

## Sequence 00: Empty Hallway

**Environment:** A static indoor hallway environment with minimal obstacles and no dynamic objects.

| Method | Info |
| :--- | :--- |
| **Wheel Odom** | Significant drift accumulates over distance due to wheel slip and systematic errors. |
| **EKF** | IMU fusion corrects heading drift, but lateral drift remains due to wheel slip. |
| **ICP** | Despite sparse features, ICP successfully stabilizes the EKF guess and quality checks to ensure smooth transitions in feature-poor segments. |
| **SLAM** | Loop closure corrects accumulated drift when revisiting areas; produces a coherent occupancy grid. |

**Summary** In empty hallways, SLAM significantly outperforms local methods due to loop closure. While ICP provides good local stability, SLAM’s graph optimization is necessary to correct longitudinal drift when the robot returns to previously mapped areas.

---

## Sequence 01: Non-Empty Hallway with Sharp Turns

**Environment:** An indoor hallway environment containing obstacles and clutter, with sections of sharp turning motion.

| Method | Info |
| :--- | :--- |
| **Wheel Odom** | Wheel slip during sharp turns causes significant heading errors that propagate rapidly. |
| **EKF** | IMU effectively corrects rotational errors during high-yaw segments, though position drift remains moderate. |
| **ICP** | Due to sharp turn and rapid info that come in from lidar, we have to reduce the ros bag play rate to accomulate that. |
| **SLAM** | Pose graph optimization and loop closure produce a globally consistent trajectory with sharp obstacle boundaries. |

**Summary** This scenario highlights the strength and weakness of each method. While Wheel Odometry fails during sharp turns, ICP maintains high accuracy by locking onto environmental clutter but demand higher computing power. SLAM provides the final layer of global consistency, using rich features to trigger loop closures over long trajectories.

---

## Sequence 02: Non-Empty Hallway with Non-Aggressive Motion

**Environment:** An indoor hallway environment with obstacles, recorded with smoother and non-aggressive robot motion.

| Method | Info |
| :--- | :--- |
| **Wheel Odom** | Smooth motion reduces instantaneous wheel slip, but systematic errors cause slow, linear drift accumulation. |
| **EKF** | Stable IMU readings during smooth motion provide excellent heading estimates, keeping the robot aligned. |
| **ICP** | Kind of optimal conditions beacuse rich features and low-velocity motion result in a good odometry. |
| **SLAM** | Ideal SLAM conditions because of smooth motion prevents scan-smearing while loop closures maintain a clean occupancy grid. |

**Summary** Optimal conditions for all navigation methods. ICP approaches SLAM's local performance levels due to high-quality matches provided by smooth motion and distinct obstacles. The resulting SLAM map is exceptionally clean.

---

## Performance Summary

1. **SLAM** (all sequences) - Best overall, especially for long trajectories
2. **ICP** (Seq 02, 01) - Excellent locally when features available
3. **EKF** (all sequences) - Consistent baseline
4. **Wheel Odometry** - Accumulates drift quickly

### Method Pro & Con:
---

#### Wheel Odometry:

**Pro**
- Simple, always available
- No computational overhead

**Con**
- Drift accumulates unbounded
- Sensitive to wheel slip, especially during turns
- No correction mechanism

---

#### EKF Fusion:

**Pro**
- Reduces heading drift significantly
- Handles turns better than wheel-only
- Low computational cost
- Real-time capable

**Con**
- Cannot correct lateral position drift
- No loop closure
- No real ground truth

---

#### ICP:

**Pro**
- Significant drift reduction in feature-rich environments
- Significant improve in both position and heading

**Con**
- Struggles in feature-poor environments (empty hallways)
- Computationally expensive
- Unbounded drift over long trajectories (because no loop closure)
- Forward motion poorly constrained in hallways

---

#### SLAM

**Pro**
- Best long-term accuracy
- Loop closure eliminates accumulated drift
- Globally consistent maps
- Works in feature-poor environments with loop closure
- Strong to odometry errors

**Con**
- Highest computational cost
- Requires revisiting locations for loop closure
- Initial trajectory may drift before first loop closure