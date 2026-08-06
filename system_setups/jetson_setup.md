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

# Configure Static IP for Ethernet Port of Jetson

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

# Router Setup for WiFi/LAN Connection for AUV

This setup allows a user to connect to the Jetson and other onboard Ethernet devices over WiFi while maintaining a wired LAN connection between the router, Jetson, and Ethernet sensors.

## Router Configuration

Open the router configuration page and verify the following settings.

### LAN Configuration

![LAN Configuration](./1.png)

### Internet Configuration

![Internet Configuration](./2.png)

### DHCP Server Configuration

![DHCP Server Configuration](./3.png)

## Network Setup

After configuring the router, connect one of the router's **LAN ports** to the **Ethernet switch**. Connect the Jetson and other Ethernet-based sensors to the same switch.

All devices should be configured on the same LAN subnet. For this configuration:

- **Router LAN IP:** `192.168.194.1`
- **LAN Subnet:** `192.168.194.0/24`
- **Subnet Mask:** `255.255.255.0`
- **Jetson:** `192.168.194.x`
- **Ethernet Sensors:** `192.168.194.x`

Ensure that each device is assigned a **unique IP address** within the subnet.

The laptop/ground station can then be connected to the router over WiFi. Since the WiFi and Ethernet devices are part of the same LAN, the laptop can communicate with the Jetson and other Ethernet devices using their respective IP addresses.

For example, if the Jetson has the IP address `192.168.194.10`, its connectivity can be checked using:

```bash
ping 192.168.194.10
```

The Jetson can then be accessed over the network using SSH, a GUI, ROS, or any other network service running on it.

## Network Architecture

```text
                              INTERNET
                                  │
                                  │
                         Upstream Network
                                  │
                                  ▼
                     ┌─────────────────────────┐
                     │         Router          │
                     │                         │
                     │  WAN IP: 10.21.138.100  │
                     │                         │
                     │          NAT            │
                     │       WAN ↔ LAN         │
                     │                         │
                     │  LAN IP: 192.168.194.1  │
                     └───────────┬─────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
                  WiFi                      LAN
                    │                         │
                    ▼                         ▼
           ┌─────────────────┐       ┌─────────────────┐
           │ Laptop / Ground │       │ Ethernet Switch │
           │     Station     │       └────────┬────────┘
           │                 │                │
           │ 192.168.194.x   │       ┌────────┴─────────┐
           └─────────────────┘       │                  │
                                    ▼                  ▼
                            ┌───────────────┐   ┌────────────────┐
                            │    Jetson     │   │    Ethernet    │
                            │               │   │     Sensors    │
                            │192.168.194.x  │   │ 192.168.194.x │
                            └───────────────┘   └────────────────┘


                    LAN: 192.168.194.0/24
                    Subnet Mask: 255.255.255.0
                    Default Gateway: 192.168.194.1
```

## WAN and NAT

The router connects the local network to an external/upstream network.

- **WAN IP:** `10.21.138.100`
- **LAN IP:** `192.168.194.1`
- **LAN Subnet:** `192.168.194.0/24`

The WAN IP is the address assigned to the router on the upstream network, while `192.168.194.1` is the router's address on the local network.

The router performs **NAT (Network Address Translation)** between the LAN and WAN. NAT allows devices with private `192.168.194.x` addresses, such as the Jetson, to access the Internet through the router's WAN connection.

## Internet Access on the Jetson

The Jetson can access the Internet through this connection setup.

For Internet access to work, the Jetson should have:

- An IP address in the `192.168.194.0/24` subnet
- `192.168.194.1` configured as the default gateway
- A valid DNS server
- A working WAN/Internet connection on the router

The current routing configuration can be checked using:

```bash
ip route
```

The routing table should contain a default route similar to:

```text
default via 192.168.194.1 dev <ethernet-interface>
```

If the default route is missing, configure the gateway and DNS settings for the Ethernet connection using:

```bash
sudo nmcli connection modify eth_switch \
  ipv4.gateway 192.168.194.1 \
  ipv4.never-default no \
  ipv4.dns "8.8.8.8 1.1.1.1" \
  ipv4.ignore-auto-dns no
```
After modifying the connection, restart it for the changes to take effect:

```bash
sudo nmcli connection down eth_switch
sudo nmcli connection up eth_switch
```

Verify the routing table again:

```bash
ip route
```

You should now see a default route similar to:

```text
default via 192.168.194.1 dev <ethernet-interface>
```

You can also verify the configured DNS servers using:

```bash
resolvectl status
```

Once the default route is configured, test Internet connectivity:

```bash
ping 8.8.8.8
```

Then verify DNS resolution:

```bash
ping google.com
```

If `8.8.8.8` works but `google.com` does not, check the DNS configuration.

If `192.168.194.1` is reachable but `8.8.8.8` is not, check the router's WAN connection, NAT configuration, and upstream network connectivity.

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