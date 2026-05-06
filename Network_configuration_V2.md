Here’s a **clean, copy-ready Markdown (English version)** you can use directly:

---

````markdown
# 📡 ROS2 Ethernet Communication Setup (NUC ↔ Leo Rover / PC)

## 🎯 Objective

- Use **Ethernet (wired connection)** for ROS2 communication  
- Assign static IPs (`10.0.0.x`)  
- Avoid WiFi interference  
- Achieve **stable multi-machine ROS2 communication**

---

# 🧩 1. Network Configuration

## 1.1 IP Assignment

| Device | IP Address |
|--------|-----------|
| NUC (Host) | `10.0.0.2` |
| Leo Rover / PC | `10.0.0.1` |

---

## 1.2 Set Static IP (using `nmtui`)

```bash
nmtui
````

Navigate to:

```
Edit a connection → Wired connection
```

Set:

```
IPv4 Configuration: Manual
Address: 10.0.0.2/24 (NUC) or 10.0.0.1/24 (peer)
Gateway: (leave empty)
DNS: (leave empty)
✔ Never use this network for default route
```

---

## 1.3 Activate Connection

```bash
nmcli connection up <connection_name>
```

---

## 1.4 Verify Connectivity

```bash
ip addr
ping 10.0.0.1
```

---

# ⚙️ 2. ROS2 Environment Setup

## 2.1 Temporary Setup (on BOTH machines)

```bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
```

---

## 2.2 Permanent Setup

Edit:

```bash
nano ~/.bashrc
```

Add:

```bash
# ROS2 network config
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0

# Do not enable FastDDS XML by default.
# Enable it only when using wired ROS2 multi-machine communication.
# export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml

alias ros_local='unset FASTRTPS_DEFAULT_PROFILES_FILE; export ROS_DOMAIN_ID=5; export ROS_LOCALHOST_ONLY=0; echo "ROS2 local mode: FastDDS XML disabled"'

alias ros_wired='export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml; export ROS_DOMAIN_ID=5; export ROS_LOCALHOST_ONLY=0; echo "ROS2 wired mode: FastDDS XML enabled"'


```

Apply:

```bash
source ~/.bashrc
```

---

# 🌐 3. Force ROS2 to Use Ethernet (Critical)

## ❗ Problem

In multi-interface systems (WiFi + Ethernet):

```
ROS2 (DDS) may choose WiFi by default ❌
```

Result:

```
Ethernet communication fails
```

---

## ✅ Solution: Bind DDS to Ethernet Interface

### 3.1 Create FastDDS Configuration File

```bash
nano ~/fastdds.xml
```

---

### 3.2 Configuration (Example for NUC)

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.0.0.2</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
```

👉 On the other machine, change to:

```
10.0.0.1
```

---

### 3.3 Enable Configuration

```bash
echo "export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds.xml" >> ~/.bashrc
source ~/.bashrc
```

---

# 🧪 4. Communication Test

## Publisher (Machine A)

```bash
ros2 run demo_nodes_cpp talker
```

---

## Subscriber (Machine B)

```bash
ros2 run demo_nodes_cpp listener
```

---

## Verify Topics

```bash
ros2 topic list
```

Expected output:

```
/chatter
```

---

# 🚨 5. Troubleshooting

## ❌ 1. Cannot ping

```bash
ip addr
```

Check:

```
inet 10.0.0.x must exist
```

---

## ❌ 2. Topics not visible

```bash
echo $ROS_LOCALHOST_ONLY
```

Must be:

```
0
```

---

## ❌ 3. WiFi interference

Disable temporarily:

```bash
nmcli radio wifi off
```

---

## ❌ 4. Wrong network interface (most common)

Symptoms:

```
WiFi works ✔
Ethernet fails ❌
```

👉 Solution:

```
Bind DDS to Ethernet (FastDDS config)
```

---

# 🧠 6. Key Concepts

## ROS2 Communication Model

```
ROS2 = DDS (peer-to-peer, no master)
```

Requires:

```
✔ Same subnet
✔ Same ROS_DOMAIN_ID
✔ ROS_LOCALHOST_ONLY=0
✔ Correct network interface
```

---

## Multi-Network Issue

```
DDS selects default network interface automatically
```

NOT:

```
The interface you expect
```

---

# 🎯 7. Recommended Architecture

```
        WiFi (192.168.x.x)
             ↑
         Internet / SSH

NUC ---------------- Leo Rover
  |                    |
  └── Ethernet (10.0.0.x) → ROS2 Communication
```

---

# 🚀 8. One-Line Summary

```
ROS2 communication issues are usually NOT code problems,
but network + DDS interface selection problems.
```

```

---

如果你后面要写 report / GitHub README，这一版已经是**工程级标准文档**，可以直接用 👍
```
