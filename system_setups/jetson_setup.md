# Jetson Setup Guide

## Platform

- **Hardware:** NVIDIA Jetson
- **JetPack Version:** 6.2

---

# Auto-connect to Wi-Fi and Set Hostname

Connect to the Wi-Fi network and configure the hostname.

```bash
nmcli device wifi list
nmcli device wifi connect mavlab password mavlab24
nmcli connection modify mavlab connection.autoconnect yes
sudo hostnamectl set-hostname timi
```

---

# SSH Installation

Install and enable the SSH server.

```bash
sudo apt update
sudo apt install -y openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh
```

---

# Git Installation

Install Git.

```bash
sudo apt-get update
sudo apt-get install git -y
```

---

# Docker Installation

Install Docker and add the current user to the Docker group.

```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin -y
sudo systemctl enable docker
sudo systemctl start docker
sudo usermod -aG docker $USER
```

---

# USB Wi-Fi Adapter Driver Installation

Install the driver for the USB Wi-Fi adapter.

```bash
sudo apt install dkms git build-essential
git clone https://github.com/morrownr/88x2bu-20210702.git
cd 88x2bu-20210702
sudo ./install-driver.sh
sudo reboot
```

---

# Verify Wi-Fi Interfaces

Check whether both Wi-Fi interfaces are detected.

```bash
iw dev
```

You should see both interfaces.

If the USB Wi-Fi adapter does not obtain an IP address, connect it manually.

```bash
sudo nmcli dev wifi connect "mavlab" password "mavlab24" ifname wlx8c902d14c273
```

---

# Check Wi-Fi Signal Strength

Display signal strength and link quality.

> Lower (more negative) signal strength in dBm indicates a weaker signal.

```bash
iwconfig wlP1p1s0
iwconfig wlx8c902d14c273
```

---

# Configure Static IP for Ethernet Switch

Configure a persistent static IP for the Ethernet interface (`enP8p1s0`).

```bash
sudo nmcli con add type ethernet con-name eth_switch ifname enP8p1s0 ip4 192.168.194.10/24
sudo nmcli con mod eth_switch connection.autoconnect yes
sudo nmcli con up eth_switch
```

Verify the assigned IP address.

```bash
ifconfig
```
If you are unable to ping the sonar or any other device connected to the Ethernet switch, the Ethernet interface on the Raspberry Pi may not have initialized correctly.

Try reconnecting the enP8p1s0 interface using NetworkManager:
```bash
sudo nmcli device disconnect enP8p1s0
sudo nmcli device connect enP8p1s0
```
This forces the Ethernet interface to reinitialize and often restores connectivity to devices on the Ethernet network. After reconnecting, verify communication by pinging the target device again.

# SonarView AppImage Dependencies

Install FUSE and grant serial port permissions.

```bash
sudo apt update
sudo apt install libfuse2
sudo usermod -aG dialout $USER
```

---

# Cyclone DDS Network Tuning

Large ROS 2 messages (images, point clouds, etc.) require larger kernel network buffers.

## Permanent Configuration

Create the configuration file.

```bash
sudo nano /etc/sysctl.d/60-auv-ros2-buffers.conf
```

Add the following:

```text
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
net.core.rmem_max=2147483647
```

Apply the configuration.

```bash
sudo sysctl -p /etc/sysctl.d/60-auv-ros2-buffers.conf
```

---

## Temporary Configuration

Apply until the next reboot.

```bash
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo sysctl -w net.core.rmem_max=2147483647
```