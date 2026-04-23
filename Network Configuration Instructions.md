# 🤖 ROS 2 分布式集群网络配置手册 (Jazzy)

**核心架构：** 双网段物理隔离 + 逻辑路由转发 (NAT) + Discovery Server 单播通信
**系统环境：** Ubuntu 24.04 LTS | ROS 2 Jazzy Jalisco

---

## 📍 第一阶段：网络拓扑与 IP 规划

为确保高带宽传感器数据与无线控制信令互不干扰，系统采用双网段隔离设计。

| 设备角色 | 物理接口 | 分配 IP 地址 | 默认网关 |
| :--- | :--- | :--- | :--- |
| **NUC (核心路由)** | `wlo1` (无线热点) | `10.42.0.1` | - |
| **NUC (硬件中枢)** | `enp114s0` (有线) | **`10.0.0.100`** | - |
| **控制端 PC** | WiFi 连接 | `10.42.0.x` | `10.42.0.1` |
| **LeoRover (底盘)**| 有线连接 | `10.0.0.1` | **`10.0.0.100`** |
| **Arm Pi (机械臂)**| 有线连接 | `10.0.0.59` | **`10.0.0.100`** |

> 💡 **架构说明：** LeoRover 出厂默认占用 `10.0.0.1`（常规网关地址）。为规避冲突，NUC 有线网口被指定为 `10.0.0.100`，并作为有线网段的统一网关。

---

## ⚙️ 第二阶段：NUC 核心网络配置

在 NUC 上执行以下操作，构建逻辑路由中枢。

### 1. 清理旧配置并设置静态 IP
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
```

### 2. 禁用有线网卡的“默认路由” 
由于 `enp114s0` 仅连接内部局部网络（无外网访问），需防止系统错误地将流量导向该接口。
```bash
sudo nmcli con mod "Wired connection 1" ipv4.never-default yes
sudo nmcli con up "Wired connection 1"
```

### 3. 开启 IP 转发与 NAT 伪装 (打通跨网段)
此步骤允许 `10.42.0.x` 网段的 PC 访问 `10.0.0.x` 网段的下位机。
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

### 4. 禁用 WiFi 后台扫描 (保障控制低延迟)
防止 NUC 在移动过程中因搜索周围 WiFi 导致瞬间的高延迟掉包。
```bash
sudo bash -c "cat <<EOF > /etc/NetworkManager/conf.d/99-disable-scanning.conf
[device]
wifi.scan-rand-mac-address=no
wifi.backend=wpa_supplicant
EOF"

sudo systemctl restart NetworkManager
```

---

## 🚙 第三阶段：执行端配置 (LeoRover & Arm Pi)

**核心要求：** 有线网段下属设备的网关必须指向 NUC。

### 1. 机械臂配置 (10.0.0.59)
SSH 登录后确认或修改网关：
```bash
sudo nmcli con mod "Wired connection 1" ipv4.gateway 10.0.0.100
sudo nmcli con up "Wired connection 1"
```

### 2. LeoRover 配置 (10.0.0.1)
同理，需进入小车系统，确保其网络配置的默认路由 (Default Gateway) 指向 `10.0.0.100`。

---

## 🌐 第四阶段：ROS 2 Discovery Server 部署

应对跨网段多播失效的最佳实践，强制节点通过单播向 Server 注册。

### 1. 启动服务端 (NUC)
```bash
# 监听 0.0.0.0 确保无线和有线网段均可访问
fastdds discovery --server-id 0 --ip-address 0.0.0.0 --port 11811
```

### 2. 客户端环境变量下发 (`~/.bashrc`)
根据设备所处的网络位置，在各自的 `~/.bashrc` 末尾添加对应配置，并执行 `source ~/.bashrc`。

| 设备 | 添加的环境变量指令 |
| :--- | :--- |
| **NUC** | `export ROS_DOMAIN_ID=5`<br>`export ROS_DISCOVERY_SERVER="127.0.0.1:11811"` |
| **PC (控制台)** | `export ROS_DOMAIN_ID=5`<br>`export ROS_DISCOVERY_SERVER="10.42.0.1:11811"` |
| **Arm Pi (机械臂)**| `export ROS_DOMAIN_ID=5`<br>`export ROS_DISCOVERY_SERVER="10.0.0.100:11811"` |
| **LeoRover (底盘)**| `export ROS_DOMAIN_ID=5`<br>`export ROS_DISCOVERY_SERVER="10.0.0.100:11811"` |

---

## 📷 第五阶段：传感器物理与数据链路检查

传感器产生海量数据，必须在 NUC 本地做好预检，避免拖垮 WiFi 链路。

### 1. 物理总线平衡检查
```bash
lsusb -t
```
> ⚠️ **带宽分配原则**：若相机工作在 1080p/30fps，将极大消耗 USB 根集线器 (Root Hub) 带宽。务必确保雷达与相机插在不同的物理总线端口上。

### 2. 相机底层取流稳定性
```bash
sudo apt install v4l-utils
# 列出支持格式，优先选用 MJPG (低带宽)
v4l2-ctl -d /dev/video0 --list-formats-ext
# 压力测试：连续抓取 100 帧查看是否报错
v4l2-ctl -d /dev/video0 --stream-mmap --stream-count=100
```

### 3. 雷达串口权限与设备名绑定 (Udev)
防止雷达重启后设备节点在 `ttyUSB0` 和 `ttyUSB1` 间漂移。
```bash
# 1. 通过 lsusb 查看雷达的 Vendor ID 和 Product ID
lsusb 

# 2. 创建规则文件 (此处以 ID 04b4:8613 为例，请替换为实际 ID)
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="8613", MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/rplidar.rules

# 3. 重新加载并触发规则
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## ✅ 第六阶段：全系统最终验证流程

请按顺序执行，快速定位网络及通信瓶颈：

**1. 物理穿透测试 (在 PC 上执行)**
* `ping 10.42.0.1` (验证 PC -> NUC 无线畅通)
* `ping 10.0.0.59` (验证 PC -> NUC NAT 转发 -> 机械臂畅通)
* `ping 10.0.0.1` (验证 PC -> NUC NAT 转发 -> 小车畅通)
> *如果有线网段设备不通，请重点排查 NUC 的 NAT 伪装规则或执行端的网关设置。*

**2. 链路稳定性测试**
* 在 PC 上执行：`ping 10.0.0.59 -i 0.2`
> *预期指标：延迟稳定在 5ms 左右，无突发跳跃，无丢包。*

**3. ROS 2 跨网段通信测试**
* 在 **Arm Pi** 上运行发送端：`ros2 run demo_nodes_cpp talker`
* 在 **PC** 上运行节点发现：`ros2 node list` (应能看到 `/talker`)
* 在 **PC** 上接收数据：`ros2 topic echo /chatter`
