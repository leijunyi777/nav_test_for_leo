````markdown
# 🖥️ Remote Desktop Setup Guide: Tailscale + NoMachine on Ubuntu 24.04

## 🎯 Objective

Set up a stable remote desktop connection between:

- **NUC**: remote machine to be controlled
- **Laptop / PC**: machine used to access the NUC remotely

Recommended architecture:

```text
Laptop / PC  ── Tailscale VPN ── NUC
                    ↓
              NoMachine Remote Desktop
````

---

# 1. Network Concept

## 1.1 What Tailscale Does

Tailscale creates a virtual private network between your devices.

Each device gets a virtual IP address like:

```text
100.x.x.x
```

This IP is independent from:

```text
WiFi IP:      192.168.x.x
Ethernet IP: 10.0.0.x
```

So your NUC can still use Ethernet for ROS2 while Tailscale is used only for remote desktop.

---

## 1.2 Recommended Network Separation

```text
Computer 1 / Leo Simulator  ── Ethernet ── NUC
        ROS2 communication: 10.0.0.x

Computer 2 / Remote PC      ── Tailscale ── NUC
        Remote desktop: 100.x.x.x
```

Recommended use:

| Function                 | Network                |
| ------------------------ | ---------------------- |
| ROS2 topic communication | Ethernet `10.0.0.x`    |
| Remote desktop           | Tailscale `100.x.x.x`  |
| Internet access          | WiFi or normal network |

---

# 2. Install Tailscale

Do this on both:

* NUC
* Remote PC

## 2.1 Install Tailscale

```bash
curl -fsSL https://tailscale.com/install.sh | sh
```

---

## 2.2 Start Tailscale

```bash
sudo tailscale up
```

A browser login page will appear.

Log in using the same Tailscale account on both devices.

---

## 2.3 Check Tailscale IP

```bash
tailscale ip
```

Expected output:

```text
100.x.x.x
```

This is the Tailscale virtual IP of the current machine.

---

## 2.4 Check Tailscale Connection

On the remote PC, ping the NUC Tailscale IP:

```bash
ping <NUC_TAILSCALE_IP>
```

Example:

```bash
ping 100.101.102.103
```

If the ping works, the Tailscale network is connected.

---

# 3. Install NoMachine

Do this on both:

* NUC
* Remote PC

## 3.1 Download NoMachine

Go to the NoMachine download page:

```text
https://www.nomachine.com/download
```

Choose:

```text
Linux → Ubuntu → DEB package
```

---

## 3.2 Install NoMachine

Go to the folder where the `.deb` file was downloaded.

Example:

```bash
cd ~/Downloads
```

Install it:

```bash
sudo dpkg -i nomachine_*.deb
sudo apt-get install -f
```

---

## 3.3 Check NoMachine Service

```bash
sudo systemctl status nxserver
```

Expected status:

```text
active (running)
```

If it is not running, start it:

```bash
sudo systemctl start nxserver
```

Enable auto-start:

```bash
sudo systemctl enable nxserver
```

---

# 4. Configure Ubuntu Desktop for Stable Remote Access

## 4.1 Disable Wayland

Ubuntu 24.04 uses Wayland by default, but remote desktop software may work more reliably with X11.

Edit the GDM configuration file:

```bash
sudo nano /etc/gdm3/custom.conf
```

Find this line:

```text
#WaylandEnable=false
```

Change it to:

```text
WaylandEnable=false
```

Save and exit:

```text
Ctrl + O
Enter
Ctrl + X
```

Reboot:

```bash
sudo reboot
```

---

## 4.2 Disable Screen Lock

On the NUC:

```bash
gsettings set org.gnome.desktop.screensaver lock-enabled false
```

Optional: disable idle delay:

```bash
gsettings set org.gnome.desktop.session idle-delay 0
```

---

# 5. Connect to the NUC Using NoMachine

Open NoMachine on the remote PC.

Create a new connection:

```text
Protocol: NX
Host: <NUC_TAILSCALE_IP>
Port: 4000
```

Example:

```text
Host: 100.101.102.103
Port: 4000
```

Then log in using the NUC Ubuntu username and password.

---

# 6. Local Network Alternative

If the remote PC and NUC are on the same WiFi or same LAN, Tailscale is not required.

You can connect directly using the NUC LAN IP.

Check NUC IP:

```bash
ip addr
```

Example:

```text
192.168.1.20
```

In NoMachine, use:

```text
Host: 192.168.1.20
Port: 4000
```

This method may work without internet.

---

# 7. Important Notes for ROS2 Users

## 7.1 Keep ROS2 and Remote Desktop Separated

Recommended:

```text
ROS2 communication:
Computer 1 ↔ NUC over Ethernet 10.0.0.x

Remote desktop:
Computer 2 ↔ NUC over Tailscale 100.x.x.x
```

Do not force ROS2 to use the Tailscale interface unless you intentionally want ROS2 communication over VPN.

---

## 7.2 FastDDS Configuration

If ROS2 is configured to use Ethernet only, the FastDDS XML file should contain the local Ethernet IP of each machine.

On the NUC:

```xml
<interfaceWhiteList>
  <address>10.0.0.2</address>
</interfaceWhiteList>
```

On Computer 1 / Leo Simulator:

```xml
<interfaceWhiteList>
  <address>10.0.0.1</address>
</interfaceWhiteList>
```

Do not put the Tailscale IP in the FastDDS XML unless ROS2 needs to communicate through Tailscale.

---

# 8. Useful Commands

## 8.1 Check Tailscale Status

```bash
tailscale status
```

---

## 8.2 Check Tailscale IP

```bash
tailscale ip
```

---

## 8.3 Temporarily Disconnect Tailscale

```bash
sudo tailscale down
```

Reconnect:

```bash
sudo tailscale up
```

---

## 8.4 Check NoMachine Service

```bash
sudo systemctl status nxserver
```

Restart NoMachine:

```bash
sudo systemctl restart nxserver
```

---

## 8.5 Check IP Addresses

```bash
ip addr
```

---

# 9. Troubleshooting

## 9.1 Cannot Connect to NUC

Check Tailscale:

```bash
tailscale status
```

Check ping:

```bash
ping <NUC_TAILSCALE_IP>
```

Check NoMachine service:

```bash
sudo systemctl status nxserver
```

---

## 9.2 Connected but Black Screen

Possible cause:

```text
Wayland is still enabled
```

Fix:

```bash
sudo nano /etc/gdm3/custom.conf
```

Make sure this line exists:

```text
WaylandEnable=false
```

Then reboot:

```bash
sudo reboot
```

---

## 9.3 NUC Screen Locks During Remote Access

Disable screen lock:

```bash
gsettings set org.gnome.desktop.screensaver lock-enabled false
gsettings set org.gnome.desktop.session idle-delay 0
```

---

## 9.4 Tailscale Works but NoMachine Cannot Connect

Check if port 4000 is reachable.

On the NUC, check the service:

```bash
sudo systemctl status nxserver
```

If firewall is enabled, temporarily disable it for testing:

```bash
sudo ufw status
sudo ufw disable
```

After testing, you can re-enable it:

```bash
sudo ufw enable
```

---

## 9.5 ROS2 Topics Stop Working After Remote Desktop Setup

Do not mix the ROS2 communication interface with the remote desktop interface.

Recommended:

```text
FastDDS XML: Ethernet IP only
Tailscale: remote desktop only
```

NUC FastDDS:

```xml
<address>10.0.0.2</address>
```

Computer 1 FastDDS:

```xml
<address>10.0.0.1</address>
```

Restart terminals after changing environment variables:

```bash
source ~/.bashrc
```

Or open a new terminal.

---

# 10. Recommended Final Architecture

```text
                 Internet / Tailscale
                         │
                         │ Remote Desktop
                         │ 100.x.x.x
                         │
Remote PC ────────────── NUC
                         │
                         │ ROS2 Ethernet
                         │ 10.0.0.2
                         │
                  Computer 1 / Leo Simulator
                         10.0.0.1
```

---

# 11. One-Line Summary

```text
Use Tailscale + NoMachine for remote desktop,
but keep ROS2 communication on the dedicated Ethernet network.
```

```
```
