，# Robot Control System

This project implements an end-to-end ROS 2 solution for autonomous **search–pick–place** with decoupled microservices. The **Main Controller** governs a strict finite‑state machine (FSM), while **Vision**, **Navigation**, and **Manipulator** act as focused actuators. Semantic memory is maintained per color, enabling the robot to pair objects with their matching boxes and complete **3 full cycles** before automatic shutdown.


## 1. System Architecture

All components are decoupled and communicate via ROS 2 topics and the Nav2 action server. The FSM governs **control flow**, while actuators handle **data flow** (estimation, pathing, manipulation).

| Node                  | Runtime Name               | Responsibility                                                                                                                                                                                                                                 |
| :-------------------- | :------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **robot\_fsm**        | `robot_control_node`       | **Brain & FSM.** Maintains per‑color semantic memory. Persists `map` coordinates for navigation and always refreshes `camera_link` coordinates for manipulation **right at action time** to minimize motion error. Publishes boolean triggers. |
| **camera\_node**      | `vision_node`              | **Eyes.** RealSense frames → YOLOv8 detections (top‑k). Publishes `{color, type ∈ {object, box}, x, y, z}` in `camera_link`.                      |
| **nav\_node**         | `nav_controller_node`      | **Smart Legs.** Consumes `/nav/goal_point` (`PointStamped` in `map`). Computes yaw to face the target and stops at a safe **0.25 m standoff** for arm operation.                                                                               |
| **manipulator\_node** | `manipulator_control_node` | **Arm & Gripper.** Consumes `/arm/grasp_pose` and `/arm/grasp_status`. Drives MyCobot 280 with open‑loop moves.                                                                                                                                |

**TF Requirement:** A valid TF tree `map → odom → base_link → camera_link` is required. The controller transforms detections from `camera_link` to `map` for memory (navigation), while manipulation uses the latest `camera_link` pose at the moment of action.



## 2. Finite State Machine (FSM)

The robot repeats the full mission **three times** and then shuts down. **No wall‑clock timers are used** for GRASP/DROP verification. Instead, success is inferred when the **active color’s `object`** receives **no new updates** across **N consecutive vision messages** (miss thresholds; defaults below).

### State Flow

1.  **INIT**
    *   **Logic:** Probe the hardware graph. Checks that Vision publishes, and Nav/Arm subscribe as required.
    *   **Transition:** When all required endpoints are alive → **SEARCH**.

2.  **SEARCH**
    *   **Logic:** Toggle exploration (`/nav/cmd_explore = True`). Build **color → {object, box}** memory:
        *   **`cam_pose`** (in `camera_link`) is **always** stored per detection.
        *   **`map_pose`** is stored **opportunistically** when TF is available.
    *   **Transition:** When a color has **both** `object.map_pose` **and** `box.map_pose` → set `active_target_color` → **MOVE\_TO\_OBJECT**.

3.  **MOVE\_TO\_OBJECT**
    *   **Logic:** Publish the **object’s `map` point**. Nav computes yaw and standoff autonomously.
    *   **Transition:** On `/nav/goal_reached == True` → **GRASP**.

4.  **GRASP** *(event‑driven, no timers)*
    *   **Logic:** Immediately dispatch the **latest** `camera_link` pose of the **object** to the arm and **reset the miss baseline**.
    *   **Success Criterion:** If the active color’s `object` receives **no new vision updates** for **`grasp_miss_threshold`** consecutive messages → **success** → **MOVE\_TO\_BOX**.
    *   **Otherwise:** Keep waiting (no timeouts).

5.  **MOVE\_TO\_BOX**
    *   **Logic:** Publish the **box’s `map` point** for navigation.
    *   **Transition:** On `/nav/goal_reached == True` → **DROP**.

6.  **DROP** *(event‑driven, no timers)*
    *   **Logic:** Dispatch the **latest** `camera_link` pose of the **box** to the arm and **reset the miss baseline**.
    *   **Success Criterion:** If the active color’s `object` continues to receive **no new updates** for **`drop_miss_threshold`** consecutive messages → **success**:
        *   Increment cycle count.
        *   If `< 3`: clear memory of the completed color, reset `active_target_color = "NONE"` → **SEARCH**.
        *   Else: **Shutdown**.

### Output Control Matrix

The FSM drives the actuators with a compact 4‑variable boolean matrix:

| STATE                 | NAV\_EXPLORE | NAV\_GOTO | ARM\_GRASP | ARM\_DROP |
| :-------------------- | :----------- | :-------- | :--------- | :-------- |
| **\[0] INIT**         | False        | False     | False      | False     |
| **\[1] SEARCH**       | True         | False     | False      | False     |
| **\[2] MOVE\_OBJECT** | False        | True      | False      | False     |
| **\[3] GRASP**        | False        | False     | True       | False     |
| **\[4] MOVE\_BOX**    | False        | True      | False      | False     |
| **\[5] DROP**         | False        | False     | False      | True      |

> **Notes:**
>
> *   `NAV_EXPLORE` is realized by publishing `/nav/cmd_explore`.
> *   `NAV_GOTO` is realized by continuously publishing `/nav/goal_point` (timestamp refreshed each publish).
> *   `ARM_GRASP/ARM_DROP` are realized by publishing `/arm/grasp_pose` and `/arm/grasp_status` with appropriate semantics.

***

## 3. Communication API (Topics & Interfaces)

### Controller → Actuators (Outbound)

*   **`/nav/goal_point`** — `geometry_msgs/PointStamped`  
    Target point in **`map`**. The controller **refreshes the message timestamp** on each publish. Nav synthesizes yaw and applies a 0.25 m standoff.

*   **`/nav/cmd_explore`** — `std_msgs/Bool`  
    Exploration trigger (`True` = explore, `False` = stop).

*   **`/arm/grasp_pose`** — `my_robot_interfaces/CamArmPose`  
    Target **`camera_link`** pose for the manipulator (`x, y, z`). Used for both grasp and drop; the intent is indicated by `/arm/grasp_status`.

*   **`/arm/grasp_status`** — `my_robot_interfaces/GripperState`  
    Gripper command (`grip: True` to close/pick, `False` to open/place).

### Actuators/Vision → Controller (Inbound)

*   **`/detected_object`** — `my_robot_interfaces/ObjectTarget`  
    YOLO detection in **`camera_link`** with `{name ∈ {object, box}, color, x, y, z}`.

*   **`/nav/goal_reached`** — `std_msgs/Bool`  
    `True` when Nav reports successful arrival.



### Nav2 Action (used inside `nav_controller_node`)

*   **`NavigateToPose`** — `nav2_msgs/action/NavigateToPose`  
    Used by the nav node to execute random exploration and FSM targets.


## 4. Prerequisites

*   **ROS 2 Jazzy** (Ubuntu 24.04 recommended)
*   **Nav2** stack (bringup must be running)
*   **Python libraries**:
    *   `ultralytics` (YOLOv8)
    *   `numpy`, `opencv-python`
    *   `pyrealsense2` (Intel RealSense SDK)
    *   `pymycobot` (MyCobot 280)
*   **Custom Interfaces** (`my_robot_interfaces` must be built first):
    *   `ObjectTarget.msg`
    *   `GripperState.msg`
    *   `CamArmPose.msg`
*   **Model file:** Place `best.pt` in the installed share directory of `robot_control_system`.
