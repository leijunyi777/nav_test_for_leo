# 🤖 ROS 2 集群网络操作手册：回滚与纯本地组网

---

## ⏪ 第一部分：旧配置回滚手册 (精简版)

> ⚠️ **操作提醒**：由于回滚物理网卡和路由配置会重置网络连接，如果你是通过 SSH 远程连接进行操作，执行网卡回滚命令时 SSH 可能会断开。建议直接在设备本地或确保有备用连接方式的情况下进行。

### 📦 模块一：NUC (核心路由与硬件中枢) 回滚

#### 1. 恢复 ROS 2 环境变量与停止服务
停止 Discovery Server，并清理 `.bashrc` 中的单播服务器配置（**注意：保留了 `ROS_DOMAIN_ID`**）：
```bash
# 终止后台运行的 fastdds discovery server
pkill -f "fastdds discovery"

# 仅从 ~/.bashrc 中删除带有 ROS_DISCOVERY_SERVER 的行，保留 ROS_DOMAIN_ID
sed -i '/ROS_DISCOVERY_SERVER/d' ~/.bashrc
source ~/.bashrc
```

#### 2. 回滚物理网卡与路由配置
将有线网卡恢复为自动获取（DHCP），清除静态 IP，并恢复默认路由规则：
```bash
# 恢复为自动获取 (DHCP)，并清除静态 IP 和网关配置
sudo nmcli con mod "Wired connection 1" ipv4.method auto ipv4.addresses "" ipv4.gateway ""

# 重新允许有线网卡作为默认路由
sudo nmcli con mod "Wired connection 1" ipv4.never-default no

# 重启网卡应用配置
sudo nmcli con up "Wired connection 1"
```

#### 3. 关闭网络转发与清理 NAT 规则
关闭内核级的 IP 转发，并删除之前设置的防火墙伪装规则（已移除 tailscale 相关规则）：
```bash
# 1. 关闭内核 IP 转发 (还原 sysctl.conf)
sudo sed -i 's/^net.ipv4.ip_forward=1/#net.ipv4.ip_forward=1/' /etc/sysctl.conf
sudo sysctl -p

# 2. 删除 iptables NAT 转发规则
sudo iptables -t nat -D POSTROUTING -o enp114s0 -j MASQUERADE
sudo iptables -t nat -D POSTROUTING -o wlo1 -j MASQUERADE

# 3. 保存清理后的防火墙状态，防止重启后之前的规则再次生效
sudo netfilter-persistent save

# 4. 删除 UFW 中放行的 NoMachine 端口规则 (如有配置)
sudo ufw delete allow 4000/tcp
```

#### 4. 恢复 WiFi 自动扫描
```bash
# 删除禁止后台扫描的配置文件
sudo rm -f /etc/NetworkManager/conf.d/99-disable-scanning.conf

# 重启网络管理器服务
sudo systemctl restart NetworkManager
```

### 🦾 模块二：Arm Pi (机械臂) 回滚

SSH 登录进入机械臂控制板进行操作：

#### 1. 恢复网关配置
解除强制指向 NUC (`10.0.0.100`) 的默认网关：
```bash
# 清空手动配置的网关（如果机械臂仍需要静态 IP 只是不走 NUC 路由）
sudo nmcli con mod "Wired connection 1" ipv4.gateway ""

# 如果你想彻底将机械臂网卡恢复为自动获取 IP (DHCP)，取消注释下面这行：
# sudo nmcli con mod "Wired connection 1" ipv4.method auto ipv4.addresses ""

# 重启网卡生效
sudo nmcli con up "Wired connection 1"
```

#### 2. 清理 ROS 2 环境变量
仅清理单播服务器配置，保留 `ROS_DOMAIN_ID`：
```bash
sed -i '/ROS_DISCOVERY_SERVER/d' ~/.bashrc
source ~/.bashrc
```

### 🚙 模块三：Leo Rover (底盘) 回滚

SSH 登录进入 Leo Rover 的树莓派进行操作：

#### 1. 恢复网关配置
如果 Leo Rover 使用的是 NetworkManager，执行以下命令：
```bash
# 清空强制网关
sudo nmcli con mod "Wired connection 1" ipv4.gateway ""
sudo nmcli con up "Wired connection 1"
```
*(注：如果 Leo Rover 使用的是传统的 `/etc/dhcpcd.conf` 或 `/etc/netplan/` 配置文件，请进入对应文件删除 `routers 10.0.0.100` 或 `gateway4: 10.0.0.100` 行，并执行 `sudo systemctl restart dhcpcd` 或 `sudo netplan apply`)*

#### 2. 清理 ROS 2 环境变量
仅清理单播服务器配置，保留 `ROS_DOMAIN_ID`：
```bash
sed -i '/ROS_DISCOVERY_SERVER/d' ~/.bashrc
source ~/.bashrc
```

---
---

## ⏩ 第二部分：定制版纯本地多播组网配置手册

### 🗺️ 第一阶段：网络 IP 规划 (10.0.0.x 网段)

将所有设备统一在 `10.0.0.x` 这个纯局域网内。
*   **ASUS NUC 13 Pro (主控节点)**: `10.0.0.100` (网卡: `enp114s0` 或按实际情况)
*   **大象机械臂 Pi**: `10.0.0.59` (网卡: `eth0` 或按实际情况)
*   **Leo Rover 底盘**: `10.0.0.1` (网卡: `eth0` 或按实际情况)
*   **子网掩码**: `255.255.255.0` (即 `/24`)

### 💻 第二阶段：配置静态 IP (具体到终端命令)

> **前提操作：** 确认 NUC 接交换机 1 口，机械臂和底盘接 2 口（Splitter 分出的两根线）。

#### 1. 配置 ASUS NUC 13 Pro
打开 NUC 的终端，将 NUC 强行绑定到 `10.0.0.x` 网段：
```bash
# 1. 查看你的有线网卡名称 (通常以 enp 或 eth 开头，例如 enp88s0 或 eth0)
ip -brief a

# 2. 创建名为 "robot_lan" 的静态网络配置，绑定网卡 (确保 enp114s0 是你上一步查到的真实网卡名)
sudo nmcli connection add type ethernet ifname enp114s0 con-name robot_lan ipv4.method manual ipv4.addresses 10.0.0.100/24

# 3. 激活这个网络配置
sudo nmcli connection up robot_lan

# 4. 验证IP是否生效 (应显示 10.0.0.100)
ip a show enp114s0
```

#### 2. 验证/配置大象机械臂 myCobot Pi 280
如果它之前已经是 `10.0.0.59`，原则上插上网线就能通。为了保险起见，如果需要重新配置，通过显示器或原来的方式进入终端执行：
```bash
# 1. 同样先确认网卡名称
ip -brief a

# 2. 覆盖或新建一个纯本地的静态配置 (假设网卡是 eth0)
sudo nmcli connection add type ethernet ifname eth0 con-name robot_lan ipv4.method manual ipv4.addresses 10.0.0.59/24

# 3. 激活网络
sudo nmcli connection up robot_lan
```

#### 3. 验证/配置 Leo Rover 1.8
同理，如果 Leo Rover 的 `eth0` 已经是 `10.0.0.1`，直接保持即可。如果需要重设，进入 Leo 的终端执行：
```bash
# 1. 确认网卡名称
ip -brief a

# 2. 配置静态 IP (假设网卡是 eth0)
sudo nmcli connection add type ethernet ifname eth0 con-name robot_lan ipv4.method manual ipv4.addresses 10.0.0.1/24

# 3. 激活网络
sudo nmcli connection up robot_lan
```

### 🔗 第三阶段：ROS 2 环境配置与多播打通

要让 NUC 看到 Leo Rover 自身的 Topic，且使用原生的局域网多播，必须确保三台设备的 `ROS_DOMAIN_ID` 绝对一致。

在 **NUC**、**机械臂Pi**、**Leo Rover** 三台设备的终端里，**分别**执行以下命令：
```bash
# 1. 打开 bashrc 文件
nano ~/.bashrc

# 2. 找到之前的 ROS_DOMAIN_ID 并修改，或者在文件最末尾添加：
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0

# 3. 保存并退出 (按 Ctrl+O 回车保存，Ctrl+X 退出)

# 4. 刷新环境变量生效
source ~/.bashrc
```

> 💡 **解决 "NUC 看不到 Leo Topic" 的必杀技：**
> 如果配置完后，NUC ping 得通 Leo，但 `ros2 topic list` 看不到 Leo 的话题，99% 是因为 NUC 连着 WiFi，ROS 2 的多播数据包（组播地址 `239.255.0.1`）默认从 WiFi 跑出去了，没走有线网卡。
> **终极解决方案（在 NUC 上执行一行命令，强制组播走有线）：**
> ```bash
> sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev enp114s0
> ```
> *(执行完这句，即使 NUC 开着 WiFi 上网，ROS 2 的通信也能精准通过有线网卡分发给 Leo 和机械臂。)*

### 🔌 第四阶段：NUC 传感器直连与权限配置

在 NUC 的终端执行，赋予硬件权限：

#### 1. Intel D435 相机 (Type-C to Type-C)
```bash
sudo apt-get install curl
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install librealsense2-udev-rules

sudo udevadm control --reload-rules && sudo udevadm trigger
```

#### 2. Slamtec A2M12 雷达 (USB-A)
```bash
sudo usermod -a -G dialout $USER
sudo chmod 777 /dev/ttyUSB*
```

### 🎯 第五阶段：全链路测试

1. **底层物理链路测试 (在 NUC 终端执行)：**
```bash
ping 10.0.0.59 -c 4  # 测试机械臂连接
ping 10.0.0.1 -c 4   # 测试 Leo Rover 连接
```

2. **ROS 2 节点互认测试：**
*   在 **Leo Rover** 终端随便发布一个数据或跑个节点：
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
*   在 **NUC** 终端查看：
    ```bash
    ros2 node list
    ros2 topic list
    ```
    如果你能在 NUC 上看到 `/talker` 节点和 Leo Rover 自身的里程计 (`/odom`) 或 cmd_vel 话题，说明整个集群已经处于完美的同一通信平面了。
```
