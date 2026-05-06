Here is the bilingual (English and Chinese) Markdown documentation containing all the ROS 2 terminal commands needed to debug and step through your FSM script. 

***

## ROS 2 FSM Debugging Commands / ROS 2 状态机调试命令

This guide provides the necessary command-line instructions to simulate the hardware environment, satisfy the `[0] INIT` conditions, and manually trigger each state transition in your `robot_fsm.py` node.
本指南提供了必要的命令行指令，用于模拟硬件环境、满足 `[0] INIT` 初始化条件，并手动触发 `robot_fsm.py` 节点中的每一个状态切换。

---

### 1. Environment Setup & Node Execution / 环境准备与节点运行

Before sending triggers, you must start the FSM node and a static TF publisher. The FSM relies on a coordinate transformation between `map` and `camera_link`.
在发送触发信号之前，必须启动状态机节点和静态 TF 广播。状态机依赖于 `map` 和 `camera_link` 之间的坐标系转换。

**Start the TF Publisher / 启动静态 TF 广播:**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_link
```

**Run the FSM Node / 运行状态机节点:**
*(Assuming your package is named `my_robot_controller` / 假设你的功能包名为 `my_robot_controller`)*
```bash
ros2 run my_robot_controller robot_fsm
```

---

### 2. Bypassing the INIT State / 通过 INIT 状态

The `[0] INIT` state checks the ROS 2 graph for active publishers and subscribers. You must open separate terminal tabs and run these commands to mock the hardware.
`[0] INIT` 状态会检查 ROS 2 计算图中活跃的发布者和订阅者。你必须打开多个独立的终端标签页并运行以下命令来模拟硬件就绪。

**Mock Arm Subscriber / 模拟机械臂订阅端:**
```bash
ros2 topic echo /arm/grasp_pose
```

**Mock Navigation Subscriber / 模拟导航订阅端:**
```bash
ros2 topic echo /nav/goal_point
```

**Mock Vision Publisher / 模拟视觉发布端:**
```bash
ros2 topic pub -r 10 /detection_state std_msgs/msg/Bool "{data: true}"
```
*Note: Once these three commands are running, the FSM will log `>>> ALL SYSTEMS GO! <<<` and transition to `[1] SEARCH`.*
*注：当这三个命令运行后，状态机将打印 `>>> ALL SYSTEMS GO! <<<` 并切换至 `[1] SEARCH` 状态。*

---

### 3. State Transition Triggers / 状态切换触发命令

Use the following commands to inject data and force the FSM to transition through its control loop.
使用以下命令注入数据，强制状态机在其控制循环中进行状态切换。

**Trigger [1] SEARCH -> [2] MOVE_TO_OBJECT (Publish Object Data / 发布目标物体):**
```bash
ros2 topic pub -1 /detected_object my_robot_interfaces/msg/ObjectTarget "{color: 'red', name: 'object', x: 1.5, y: -0.5, z: 0.2}"
```

**Trigger [1] SEARCH -> [2] MOVE_TO_OBJECT (Publish Box Data / 发布放置盒):**
*The FSM requires both an 'object' and a 'box' of the same color to create a "pair" and trigger the move.*
*状态机需要相同颜色的 'object' (物体) 和 'box' (放置盒) 才能成功“配对”并触发移动。*
```bash
ros2 topic pub -1 /detected_object my_robot_interfaces/msg/ObjectTarget "{color: 'red', name: 'box', x: 3.0, y: 2.0, z: 0.0}"
```

**Trigger [2] MOVE_TO_OBJECT -> [3] GRASP (Simulate Nav Success / 模拟导航到达物体):**
```bash
ros2 topic pub -1 /nav/goal_reached std_msgs/msg/Bool "{data: true}"
```

**Trigger [3] GRASP -> [4] MOVE_TO_BOX (Simulate Grasp Success / 模拟抓取成功):**
*Wait 10 seconds for the arm timer to finish. Then, the FSM checks if the object is absent. Stop publishing the 'red object' vision data, or publish an "unknown" object to advance the vision tick and simulate absence.*
*等待10秒让机械臂定时器结束。然后，状态机会检查物体是否消失。停止发布“红色物体”视觉数据，或者发布一个“未知”物体以推进视觉滴答声 (tick) 从而模拟物体已消失。*
```bash
ros2 topic pub -1 /detected_object my_robot_interfaces/msg/ObjectTarget "{color: 'unknown', name: 'none', x: 0.0, y: 0.0, z: 0.0}"
```

**Trigger [4] MOVE_TO_BOX -> [5] DROP (Simulate Nav Success / 模拟导航到达放置盒):**
```bash
ros2 topic pub -1 /nav/goal_reached std_msgs/msg/Bool "{data: true}"
```
*Note: The DROP state relies on an internal timer. After 13 seconds (10s drop + 3s cleanup), it will automatically transition back to `[1] SEARCH` and complete one cycle.*
*注：DROP 状态依赖内部定时器。13秒（10秒放置 + 3秒清理）后，它会自动切回 `[1] SEARCH` 并完成一个循环。*

---

### 4. Output Monitoring / 监控输出状态

To verify that your Control Matrix (Outputs) is behaving correctly, you can open additional terminals to monitor what the Master Logic is commanding.
为了验证控制矩阵（输出）的行为是否正确，你可以打开额外的终端来监控主逻辑发出的指令。

**Monitor Navigation Exploration State / 监控导航探索状态 (True during SEARCH):**
```bash
ros2 topic echo /nav/cmd_explore
```

**Monitor Target Task Info / 监控当前任务信息:**
```bash
ros2 topic echo /nav/target_info
```

**Monitor Gripper Control / 监控夹爪控制指令:**
```bash
ros2 topic echo /arm/grasp_status
```

**Monitor Arm Initialization / 监控机械臂复位指令:**
```bash
ros2 topic echo /arm/initial_position
```