# Raspberry Pi 5 Ethernet Configuration for Sensors Connected via an Ethernet Switch

## Platform

- **Hardware:** Raspberry Pi 5
- **Operating System:** Ubuntu 24.04.3 LTS (Noble Numbat)

# Step 1: Configure Netplan to use NetworkManager

Edit the Netplan configuration.

```bash
sudo nano /etc/netplan/*.yaml
```

Add the **renderer** field near the top.

Example:

```yaml
network:
  version: 2
  renderer: NetworkManager

  wifis:
    wlan0:
      optional: true
      dhcp4: true
      access-points:
        "mavlab":
          auth:
            key-management: psk
            password: "<PASSWORD>"

    wlxdc6279486c49:
      optional: true
      dhcp4: true
      dhcp6: false
      access-points:
        "mavlab":
          auth:
            key-management: psk
            password: "<PASSWORD>"
```

> **Note:** Keep your Wi-Fi configuration unchanged. Only add `renderer: NetworkManager`.

---

# Step 2: Allow NetworkManager to Manage Interfaces

Edit:

```bash
sudo nano /etc/NetworkManager/NetworkManager.conf
```

Change

```ini
[main]
plugins=ifupdown,keyfile

[ifupdown]
managed=false

[device]
wifi.scan-rand-mac-address=no
```

to

```ini
[main]
plugins=ifupdown,keyfile

[ifupdown]
managed=true

[device]
wifi.scan-rand-mac-address=no
```

---

# Step 3: Apply the Changes

```bash
sudo netplan apply
```

> **Important:**  
> SSH over Wi-Fi may disconnect for a few seconds while the networking backend switches to NetworkManager.
>
> This is expected.
>
> Simply reconnect via SSH after a few seconds.

Restart NetworkManager:

```bash
sudo systemctl restart NetworkManager
```

---

# Step 4: Verify NetworkManager

```bash
nmcli device status
```

`eth0` should **not** be listed as **unmanaged**.

---

# Step 5: Create the Static Ethernet Connection

Create a static Ethernet connection for the sonar network.

```bash
sudo nmcli connection add \
    type ethernet \
    con-name eth_switch \
    ifname eth0 \
    ipv4.method manual \
    ipv4.addresses 192.168.2.10/24
```
> **Important:**  
> Set the ip to whatever static ip your sonar devices are set to.

---

# Step 6: Enable Auto-connect

```bash
sudo nmcli connection modify eth_switch connection.autoconnect yes
```

---

# Step 7: Bring the Connection Up

```bash
sudo nmcli connection up eth_switch
```

---

# Step 8: Verify Configuration

Verify IP address:

```bash
ip addr show eth0
```

Expected:

```
inet 192.168.2.10/24
```

Verify routing:

```bash
ip route
```

Expected route:

```
192.168.2.0/24 dev eth0
```

Verify NetworkManager:

```bash
nmcli device status
```

Expected:

```
DEVICE   TYPE      STATE      CONNECTION

wlan0    wifi      connected  netplan-wlan0-mavlab
eth0     ethernet  connected  eth_switch
```

---

# Step 9: Test Sensor Connectivity

Ping Sonar 1:

```bash
ping -I eth0 192.168.2.90
```

Ping Sonar 2:

```bash
ping -I eth0 192.168.2.91
```

Both should respond successfully.

---

# Tethered Ethernet Connection for Raspberry Pi

## Overview

For tethered ROV operation, the Raspberry Pi can be accessed through the onboard Ethernet switch using the Ethernet tether. 

```text
                    Laptop
               (Ethernet Port)
                      │
                      │
               Ethernet Tether
                      │
                      ▼
        ┌─────────────────────────────┐
        │  Unmanaged Ethernet Switch  │
        └─────────────────────────────┘
          │          │         │        │
          │          │         │        │
          ▼          ▼         ▼        ▼
    Raspberry Pi   Sonar 1   Sonar 2   DVL
      (eth0)
```

All devices connected to the Ethernet switch should use **static IP addresses** in the same subnet.

| Device | IP Address |
|--------|------------|
| Raspberry Pi (`eth0`) | `192.168.2.10/24` |
| Laptop Ethernet | `192.168.2.20/24` |
| Sonar 1 | `192.168.2.90/24` |
| Sonar 2 | `192.168.2.91/24` |

---

## Configuring the Laptop

Open:

**Settings → Network → Wired → IPv4**

Configure the Ethernet interface as follows:

- **IPv4 Method:** Manual
- **Address:** `192.168.2.20`
- **Netmask:** `255.255.255.0`
- **Gateway:** Leave blank.
- **DNS:** Disable **Automatic DNS** and leave blank.
- **Routes:** Disable **Automatic Routes**.
- Enable **Use this connection only for resources on its network**.


---

## Verifying Connectivity

Verify that the Raspberry Pi is reachable:

```bash
ping 192.168.2.10
```

If the ping is successful, connect to the Raspberry Pi using SSH:

```bash
ssh <username>@192.168.2.10
```

Replace `<username>` with the Raspberry Pi username (e.g., `ubuntu`).

---

## Troubleshooting

If the Raspberry Pi or any Ethernet device connected to the switch is not reachable, reconnect the Ethernet interface on the Raspberry Pi:

```bash
sudo nmcli device disconnect eth0
sudo nmcli device connect eth0
```

This forces the Ethernet interface to reinitialize and typically restores communication with devices connected to the Ethernet switch.

After reconnecting, verify connectivity again:

```bash
ping 192.168.2.10
```