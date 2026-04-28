# 🤖 ROS 2 分布式集群网络与远程运维手册

### 🚀 方案简析
本方案的核心逻辑是让 **NUC 充当整个机器人系统的软路由与通信中枢**。
整体部署分为以下四个核心模块：
1.  **NUC (核心路由中枢) 配置：** 负责绑定静态 IP、开启双网卡 NAT 转发、建立 Tailscale 异地组网入口、部署 NoMachine 图形服务端，并启动 ROS 2 通信的 Discovery Server。
2.  **Arm Pi (机械臂) 配置：** 物理接入 NUC 有线网口，强制其网关指向 NUC，并配置单播环境变量。
3.  **Leo Rover (底盘) 配置：** 物理接入 NUC 有线网口，强制其网关指向 NUC，并配置单播环境变量。
4.  **控制端 PC (远程监控) 配置：** 接入 Tailscale 网络，配置智能自适应环境切换脚本，并验证全链路通信与远程桌面。
```bash
# ==============================================================================
#                      ROS 2 分布式集群网络拓扑架构图 (Jazzy)
# ==============================================================================
#
#                            [ 互联网 / 异地环境 ]
#                                     |
#                            (Tailscale 加密隧道)
#                                     |
#  +-------------------------+        v        +-------------------------------+
#  |    控制端 PC (异地)     | <-------------> |     NUC (核心路由与算力中枢)  |
#  | ----------------------- |                 |                               |
#  | 接口: Tailscale         |                 |  [虚拟入口 tailscale0]        |
#  | IP: 100.x.y.z           |                 |  IP: 100.x.y.z (NAT 转发)     |
#  | ROS_DS: 100.x.y.z:11811 |                 |                               |
#  +-------------------------+                 |  [物理无线 wlo1 - 动态切换]   |
#                                              |  模式A : 连实验室WiFi |
#  +-------------------------+                 |     IP: DHCP分配 (有外网接入) |
#  |    控制端 PC (本地)     | <-------------->  |  模式B : 开自带热点   |
#  | ----------------------- |                 |     IP: 10.42.0.1 (纯局域网)  |
#  | 接口: 实验室WiFi/热点   |                 |                               |
#  | IP: DHCP 或 10.42.0.x   |                 |  [本地硬件直连总线]           |
#  | ROS_DS: 自动切换        |                 |  USB-A 3.0: 激光雷达 (LiDAR)  |
#  +-------------------------+                 |  Type-C:    视觉相机          |
#                                              |                               |
#                                              |  [物理有线 enp114s0]          |
#                                              |  IP: 10.0.0.100 (下位机网关)  |
#                                              +-------------------------------+
#                                                  |                       |
#            +-------------------------------------+                       |
#            | (10.0.0.x 纯控制局域网，保障极低延迟)                       |
#            v                                                             v
#  +---------------------------------+                 +---------------------------------+
#  |       Arm Pi (机械臂控制板)     |                 |       Leo Rover (底盘树莓派)    |
#  | ------------------------------- |                 | ------------------------------- |
#  | 接口: 有线以太网                |                 | 接口: 有线以太网                |
#  | IP: 10.0.0.59                   |                 | IP: 10.0.0.1                    |
#  | 强制网关: 10.0.0.100            |                 | 强制网关: 10.0.0.100            |
#  | ROS_DS: 10.0.0.100:11811        |                 | ROS_DS: 10.0.0.100:11811        |
#  | 核心任务: 接收轨迹，执行运动    |                 | 核心任务: 接收 cmd_vel，驱动电机|
#  +---------------------------------+                 +---------------------------------+
# ==============================================================================
```

---

## 📦 模块一：NUC (核心路由与硬件中枢) 配置
NUC 承担着连接内外网、异地组网和 ROS 2 节点发现的核心任务。

### 1. 物理网络与路由配置
清理旧配置，设置内部有线网卡为静态 IP，并防止默认路由冲突：
```bash
# 1. 检查并删除可能存在的旧网桥 (如 br0)
nmcli con show
# 若存在废弃网桥，执行: sudo nmcli con delete br0

# 2. 配置有线网卡 enp114s0 的静态 IP
# 注意：若连接名不是 "Wired connection 1"，请根据上一条命令的输出替换
sudo nmcli con mod "Wired connection 1" \
ipv4.addresses 10.0.0.100/24 \
ipv4.method manual

# 3. 激活配置
sudo nmcli con up "Wired connection 1"

# 4. 禁用有线网卡的“默认路由” (防止流量错误导向无外网接口)
sudo nmcli con mod "Wired connection 1" ipv4.never-default yes
sudo nmcli con up "Wired connection 1"
```

开启本地 IP 转发与 NAT 伪装，打通内外网段：
```bash
# 永久开启内核 IP 转发
sudo sed -i 's/#net.ipv4.ip_forward=1/net.ipv4.ip_forward=1/' /etc/sysctl.conf
sudo sysctl -p

# 配置防火墙 NAT 转发规则
sudo iptables -t nat -A POSTROUTING -o enp114s0 -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o wlo1 -j MASQUERADE

# 保存规则使其开机自启
sudo apt install iptables-persistent
sudo netfilter-persistent save
```

禁用 WiFi 后台扫描，防止移动时高频丢包：
```bash
sudo bash -c "cat <<EOF > /etc/NetworkManager/conf.d/99-disable-scanning.conf
[device]
wifi.scan-rand-mac-address=no
wifi.backend=wpa_supplicant
EOF"

sudo systemctl restart NetworkManager
```

### 2. 异地穿透与远程桌面服务部署
安装 Tailscale 并将其作为子网路由节点：
```bash
# 安装与启动 Tailscale
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up

# 声明 NUC 负责转发无线和有线物理网段
sudo tailscale up --advertise-routes=10.0.0.0/24,10.42.0.0/24 --accept-dns=false

# 允许从 tailscale 虚拟网卡流向有线网卡的包被伪装 (补全异地访问链路)
sudo iptables -t nat -A POSTROUTING -i tailscale0 -o enp114s0 -j MASQUERADE
sudo netfilter-persistent save
```
> **后台操作：** 登录 Tailscale Admin Console，找到 NUC 设备，点击 Edit route settings，勾选确认刚才声明的两个子网路由。

部署 NoMachine 图形环境（解决 24.04 兼容问题）：
```bash
# 1. 安装 NoMachine (需提前下载 DEB 包)
sudo dpkg -i nomachine_*.deb

# 2. 禁用 Wayland 切换回 X11 (解决黑屏问题)
sudo nano /etc/gdm3/custom.conf
# 找到并取消注释：WaylandEnable=false
sudo systemctl restart gdm3

# 3. 安装虚拟显示驱动 (防止未接物理显示器时无法启动 GUI)
sudo apt install xserver-xorg-video-dummy
```

配置防火墙放行规则：
```bash
# 允许 Tailscale 接口所有流量
sudo ufw allow in on tailscale0

# 允许 NoMachine 端口 (默认 4000)
sudo ufw allow 4000/tcp
```

### 3. ROS 2 中间件服务
启动 Discovery Server 服务端，并配置本地环境变量：
```bash
# 监听 0.0.0.0 确保物理接口和 Tailscale 虚拟接口均可访问
fastdds discovery --server-id 0 --ip-address 0.0.0.0 --port 11811
```
在 NUC 的 `~/.bashrc` 末尾添加配置，并 `source ~/.bashrc`：
```bash
export ROS_DOMAIN_ID=5
# NUC 自身填 127.0.0.1
export ROS_DISCOVERY_SERVER="127.0.0.1:11811" 
```

---

## 💻 模块二：控制端 PC (远程开发与监控) 配置

### 1. 异地组网接入
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
```

### 2. 智能环境变量配置
将以下代码写入 PC 的 `~/.bashrc` 并 source。它会自动判断当前是在本地实验室还是异地远程，从而使用最快的链路。
```bash
export ROS_DOMAIN_ID=5

# 自动检测网络环境分配 Discovery Server IP
if ping -c 1 -W 1 10.42.0.1 > /dev/null 2>&1; then
    # 本地环境：能 ping 通无线物理网关
    export ROS_DISCOVERY_SERVER="10.42.0.1:11811"
else
    # 异地环境：走 Tailscale 虚拟 IP (请替换为 NUC 真实的 Tailscale IP)
    export ROS_DISCOVERY_SERVER="100.xx.yy.zz:11811" 
fi
```

### 3. 全系统验证测试
在 PC 上按顺序执行，快速定位网络及通信瓶颈：

* **网络穿透与路由测试：**
    * `ping 10.42.0.1` (验证 PC -> NUC 物理无线或 Tailscale 畅通)
    * `ping 10.0.0.59` (验证 PC -> NUC NAT 转发 -> 机械臂畅通)
    * `ping 10.0.0.1` (验证 PC -> NUC NAT 转发 -> 小车畅通)
    * *排障提示：若不通，重点排查 NUC 的 iptables 规则是否开机生效，或下位机网关是否指向 10.0.0.100。*
* **链路质量测试：**
    * 执行 `ping 10.0.0.59 -i 0.2`
    * *预期指标：本地延迟稳定在 5ms 左右；异地根据网络情况波动，但不应有高频丢包。*
* **跨网段与异地 ROS 2 通信测试：**
    * 在 Arm Pi 上运行：`ros2 run demo_nodes_cpp talker`
    * 在 PC 上运行：`ros2 node list` (应能看到 `/talker`)
    * 在 PC 上接收：`ros2 topic echo /chatter`

### 4. 远程桌面 (NoMachine) 使用指南
1.  **连接目标：** 打开 PC 端 NoMachine 客户端，在地址栏输入 NUC 的 Tailscale 虚拟 IP (`100.x.y.z`) 或本地无线 IP (`10.42.0.1`)。
2.  **账号登录：** 输入 NUC 的系统用户名和密码。
3.  **画面调优：** 进入桌面后，点击右上角翻页图标 (或按 `Ctrl+Alt+0`)：
    * 进入 **Display** 选项，勾选 `Match the client window size` (自适应分辨率)。
    * 若网络存在波动导致卡顿，建议勾选 `Disable network adaptive quality` 强制稳定画质。
    * 进入 **Audio** 选项，勾选 `Mute audio on server` 节省音频传输带宽。
4.  **最终验证：** 在桌面上打开终端，输入 `rviz2`，验证是否能流畅打开可视化界面并正常显示传感器点云/图像。

---

## 🦾 模块三：Arm Pi (机械臂) 配置
下位机的核心要求是网关必须指向 NUC。

### 1. 网关配置
SSH 登录机械臂控制板 (假设 IP 已分配为 `10.0.0.59`)，确认或修改网关：
```bash
sudo nmcli con mod "Wired connection 1" ipv4.gateway 10.0.0.100
sudo nmcli con up "Wired connection 1"
```

### 2. ROS 2 环境变量
在机械臂的 `~/.bashrc` 末尾添加配置，并 `source ~/.bashrc`：
```bash
export ROS_DOMAIN_ID=5
export ROS_DISCOVERY_SERVER="10.0.0.100:11811" 
```

---

## 🚙 模块四：Leo Rover (底盘) 配置

### 1. 网关配置
SSH 登录底盘树莓派 (IP: `10.0.0.1`)。同理，确保其网络配置文件中的 **Default Gateway** 设为 `10.0.0.100`。

### 2. ROS 2 环境变量
在小车的 `~/.bashrc` 末尾添加配置，并 `source ~/.bashrc`：
```bash
export ROS_DOMAIN_ID=5
export ROS_DISCOVERY_SERVER="10.0.0.100:11811" 
```
