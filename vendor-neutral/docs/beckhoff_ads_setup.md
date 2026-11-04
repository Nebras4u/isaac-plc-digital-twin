# TwinCAT 3 (ADS) — Setup and Troubleshooting Guide

This document describes how to configure the **vendor-neutral bridge** to
communicate with a **Beckhoff TwinCAT 3** PLC using the **ADS** protocol
(via `pyads`).

---

## Status

Connection, read, and write have been verified. Bridge driver, PLC program
(heartbeat, E-stop), and latency measurements are in progress.

---

## Requirements

- **TwinCAT 3.1** (Build 4024.x) installed on the Windows PLC machine.
- **pyads** >= 3.3.0 installed on the Linux client (inside Docker or native).
- A **direct Ethernet cable** between the Linux host and the Windows machine,
  or an isolated private network switch. **Never expose ADS (port 48898)**
  to a public or shared network.

---

## Network prerequisites

Two settings must be in place before ADS will work reliably. Both caused
real failures during bring-up (see Troubleshooting).

1. **Static IP addresses on both machines** (example: Windows
   `192.168.50.1`, Linux `192.168.50.2`, subnet `/24`). On the Linux side,
   make the address permanent with NetworkManager, otherwise it can fall
   back to a different IP after a reboot:

   ```bash
   nmcli con show    # find the connection that uses your Ethernet interface
   sudo nmcli con mod "Wired connection 1" ipv4.method manual ipv4.addresses 192.168.50.2/24 ipv4.never-default yes
   sudo nmcli con up "Wired connection 1"
   ```

   `ipv4.never-default yes` stops this interface from becoming the default
   route, so Wi-Fi keeps providing internet access.

2. **Receive Side Scaling (RSS) disabled on the Windows Ethernet adapter.**
   With RSS enabled, new inbound TCP connections were intermittently left
   unanswered. Disable it once, and re-check after reboots or driver
   updates:

   ```powershell
   Disable-NetAdapterRss -Name "Ethernet"
   Get-NetAdapterRss -Name "Ethernet"    # should report Enabled : False
   ```

   (Adapter name may differ; list adapters with `Get-NetAdapter`.)

---

## Configuration (`plc_ros_bridge.yaml`)

Add the following block to your bridge configuration:

```yaml
plc:
  driver: "ads"
  host: "192.168.50.1"                # IP of the Windows TwinCAT machine
  ams_net_id: "111.111.111.3.1.1"     # AMS NetID of the TwinCAT system (NOT the IP)
  ams_port: 851                        # 851 is the default PLC port
  local_ams_net_id: "192.168.50.2.1.1" # AMS NetID of the Linux client
```

> **Important:** The AMS NetID is **not derived from the IP address**. You
> must read it from the TwinCAT system and use the exact value.

---

## How to find the AMS NetID on Windows

1. On the Windows machine, right-click the **TwinCAT icon** in the system tray.
2. Select **Router** -> **Change AMS NetId**.
3. Copy the value displayed (for example `111.111.111.3.1.1`).
4. Use this exact value as `ams_net_id` in your YAML configuration.

---

## Route configuration on Windows

TwinCAT will **reject** ADS packets from a client that is not in its routing
table. You must register the Linux client as a route.

### Method 1: Edit `StaticRoutes.xml` (most reliable)

1. Open Notepad **as Administrator**.
2. Open the file:

```
C:\TwinCAT\3.1\Target\StaticRoutes.xml
```

3. Add the following `<Route>` block inside `<RemoteConnections>`:

```xml
<?xml version="1.0"?>
<TcConfig xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <RemoteConnections>
    <Route>
      <Name>Linux_PC</Name>
      <Address>192.168.50.2</Address>
      <NetId>192.168.50.2.1.1</NetId>
      <Type>TCP_IP</Type>
      <Flags>32</Flags>
    </Route>
  </RemoteConnections>
</TcConfig>
```

4. Save the file.
5. **Restart the TwinCAT System Service** (`TcSysSrv`) for the change to take
   effect. Open `services.msc`, find `TcSysSrv`, right-click, and choose
   **Restart**.

> **Note:** The `<Flags>32</Flags>` field is required in some TwinCAT versions
> to activate the route.

### Method 2: TwinCAT Router UI

1. Right-click the TwinCAT tray icon -> **Router** -> **Edit Routes**.
2. Click **Add**.
3. Fill in:
   - **AmsNetID**: `192.168.50.2.1.1`
   - **Transport**: `TCP_IP`
   - **Address**: `192.168.50.2`
4. Enter your **Windows username and password** when prompted.
   **This is the Windows account**, not a PLC password.
5. Uncheck **Secure ADS** before clicking OK.

---

## Verify the connection from Linux

Before running the bridge, verify that ADS is reachable:

```bash
nc -zv -w 3 192.168.50.1 48898
```

Expected output:

```
Connection to 192.168.50.1 48898 port [tcp/*] succeeded!
```

If this fails, the problem is at the network or firewall level, not in ADS.

A single success does not prove the link is stable, because some faults
are intermittent (see the RSS entry below). Run a loop and confirm that
**every** attempt succeeds:

```bash
for i in $(seq 1 30); do nc -zv -w 3 192.168.50.1 48898; sleep 1; done
```

---

## Minimal Python test

Run this script from the Linux host to confirm the connection:

```python
import pyads

TARGET_AMS = "111.111.111.3.1.1"
TARGET_IP = "192.168.50.1"
LOCAL_AMS = "192.168.50.2.1.1"

pyads.open_port()
pyads.set_local_address(LOCAL_AMS)

plc = pyads.Connection(TARGET_AMS, 851, TARGET_IP)
plc.open()

print("is_open:", plc.is_open)

if plc.is_open:
    print("Connected successfully!")
    plc.close()
```

If this prints `is_open: True`, the ADS layer is fully working.

---

## Troubleshooting

| Error | Meaning | Solution |
| :--- | :--- | :--- |
| `Socket(): connect failed` / host unreachable | Basic network problem | Check IP, subnet, and cable. Use `nc -zv 192.168.50.1 48898`. A clean `ping` does not prove TCP works, so always test the port directly. |
| `Target port not found (6)` | PLC is not in Run mode, port 851 is not active, or the AMS NetID is wrong | Start the PLC in Run mode and verify the AMS NetID. |
| `Port not connected (13)` | The client's ADS connection is not open or was already closed | Call `plc.open()` before any read/write and reopen after a disconnect. |
| `Timeout elapsed (1861)` | The request was sent but no reply arrived in time | Check that the PLC machine is reachable, that its firewall allows TCP 48898, and that the client's route is registered on the PLC machine. If timeouts are **intermittent** and `ping` is clean, see *Intermittent failures (RSS)* below. |
| Connection opens then closes immediately | The local AMS NetID set in the client does not match the route registered on the PLC machine | Verify `pyads.set_local_address()` matches the `NetId` in `StaticRoutes.xml`. |
| Intermittent connection failures (`nc` alternates between success and timeout, `ping` shows 0% loss) | Receive Side Scaling (RSS) enabled on the Windows Ethernet adapter | See *Intermittent failures (RSS)* below. |
| After a reboot, `ping` shows 100% loss and `nc` times out | The Linux Ethernet interface lost its static address | See *Interface lost its static IP after reboot* below. |
| `Device is busy (1800)` | TwinCAT router is stuck or the boot files are corrupt | Stop the PLC (`Set-AdsState -State Stop`) or switch to Config mode, then retry. |
| `Port is disabled (0x12)` | TwinCAT is in a Stop state | Switch it to Config mode using `Set-AdsState -State Config -Force` in PowerShell. |
| `Target machine not found - Missing ADS routes (7)` | The client is not registered in the TwinCAT routing table | Edit `StaticRoutes.xml` manually and restart `TcSysSrv`. |
| Route cannot be added via PowerShell | Permissions or version mismatch in `TcXaeMgmt` | Edit `StaticRoutes.xml` manually. This is the most reliable method. |

### Intermittent failures (RSS)

**Symptoms**

- `nc -zv -w 3 192.168.50.1 48898` alternates between success and timeout
  within seconds, and `pyads` fails to connect or raises `1861` at random.
- `ping` shows 0% packet loss with stable latency (around 0.5 ms).
- The connection may work one day and fail the next.

**How to confirm it is not TwinCAT or `pyads`**

- The same intermittent failures happen on an unrelated port. Start
  `python -m http.server 5000` on the Windows machine and run the same
  `nc` loop against port 5000.
- `tcpdump` on the Linux client shows failed attempts as a SYN
  retransmitted at 1 s and 2 s with no reply, while successful attempts
  get a SYN-ACK in under 1 ms:

  ```bash
  sudo tcpdump -ni <interface> host 192.168.50.1 and tcp port 48898
  ```

- Connections started **from** the Windows machine to Linux are stable,
  so only inbound connections to Windows are affected.

**Cause**

With RSS enabled on the Windows Ethernet adapter, new inbound TCP
connection attempts (SYN packets) were sometimes left unanswered, while
ICMP and Windows-initiated connections kept working. Observed on a
direct-cable setup between two laptops; disabling RSS alone resolved it.

**Fix (on the Windows machine)**

Device Manager > Network adapters > Ethernet adapter > Properties >
Advanced > **Receive Side Scaling** > **Disabled**. Or from an elevated
PowerShell:

```powershell
Disable-NetAdapterRss -Name "Ethernet"
```

Verify from the client that all attempts succeed:

```bash
for i in $(seq 1 30); do nc -zv -w 3 192.168.50.1 48898; sleep 1; done
```

### Interface lost its static IP after reboot

**Symptoms**

- Everything worked before, then after rebooting `ping 192.168.50.1`
  shows 100% loss and `nc` times out.
- `ip addr show <interface>` shows a different address than expected
  (observed: `111.111.111.1/24` instead of `192.168.50.2/24`).

**Temporary fix (lost on reboot)**

```bash
sudo ip addr flush dev <interface>
sudo ip addr add 192.168.50.2/24 dev <interface>
ping -c 5 192.168.50.1
```

**Permanent fix**

Use the NetworkManager commands from *Network prerequisites* above, then
confirm the address with `ip addr show <interface> | grep inet`.

After a reboot, also check that the Windows machine still has its own
static address (`ipconfig`) and that RSS is still disabled
(`Get-NetAdapterRss -Name "Ethernet"`), because a reboot can change
either one.

---

## Notes

- ADS is **not encrypted** and has no authentication beyond the route entry.
  The network must be private (isolated, or a direct cable between the two
  machines). Never expose ADS (port 48898) to a public or shared network.
- Do **not** use `Add-AdsRoute` from PowerShell if your `TcXaeMgmt` version is
  3.2.17 or older. Use the manual `StaticRoutes.xml` method instead.
- The AMS NetID of TwinCAT is independent of its IP address. Always read it
  from the Router UI; never guess it.

---

*Last updated: 2026-11-04*
