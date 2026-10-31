# Deployment on WSL 2 (Windows 11 lab laptop)

> **UNTESTED ON HARDWARE.** No step of this guide has been run on the lab laptop, with the KR C4 or with the ELP
> camera. Only the latency test's loopback run (both sides on one Linux host) runs, in CI. The table in
> [Untested steps](#untested-steps) lists what has not been checked.

The planned edge PC is the Windows 11 lab laptop. The ROS 2 stack runs in WSL 2. In mirrored networking mode WSL
shares the laptop's network interfaces, so the controller's RSI configuration needs no change: the PC address in
`RSI_EthernetConfig*.xml` stays the laptop's address. The general edge/client layout is in `docs/DEPLOY.md`.

Placeholders: `<LAPTOP_IP>` is the laptop's address on the KR C4 network, `<RSI_PORT>` is the port in
`RSI_EthernetConfig*.xml` (`59152` in the copies in `legacy/krc_config/rsi/`), `<BUSID>` is the camera's USB bus ID.
Real values go in local files only.

## Sources

The Microsoft pages below are the sources of this guide. `learn.microsoft.com` could not be opened from the session
that wrote it; the text was read from the pages' source files in the `MicrosoftDocs/WSL` repository on GitHub, which
the pages are built from. The quotes are from those files.

| Page | Source file |
|---|---|
| [Accessing network applications with WSL](https://learn.microsoft.com/windows/wsl/networking) | `MicrosoftDocs/WSL`, `WSL/networking.md` (ms.date 07/16/2024) |
| [Advanced settings configuration in WSL](https://learn.microsoft.com/windows/wsl/wsl-config) | `MicrosoftDocs/WSL`, `WSL/wsl-config.md` |
| [Connect USB devices](https://learn.microsoft.com/windows/wsl/connect-usb) | `MicrosoftDocs/WSL`, `WSL/connect-usb.md` (ms.date 04/16/2024) |
| [Basic commands for WSL](https://learn.microsoft.com/windows/wsl/basic-commands) | `MicrosoftDocs/WSL`, `WSL/basic-commands.md` |
| [Configure Hyper-V firewall](https://learn.microsoft.com/windows/security/operating-system-security/network-security/windows-firewall/hyper-v-firewall) | not read; the commands below are the ones quoted on the networking page |
| [Netsh interface portproxy](https://learn.microsoft.com/windows-server/networking/technologies/netsh/netsh-interface-portproxy) | not read (see [Why NAT mode cannot work](#why-the-default-nat-mode-cannot-work)) |
| [usbipd-win](https://github.com/dorssel/usbipd-win) | `README.md` and the wiki page `WSL-support` |
| [WSL2-Linux-Kernel](https://github.com/microsoft/WSL2-Linux-Kernel) | `arch/x86/configs/config-wsl` on `linux-msft-wsl-6.6.y` and `linux-msft-wsl-6.1.y` |

## 1. Distribution

Two ways. Both give ROS 2 Jazzy on Ubuntu 24.04, as in CI.

**A. Ubuntu 24.04 in WSL 2 with ROS 2 Jazzy.** In PowerShell:

```powershell
wsl --list --online
```

```powershell
wsl --install -d Ubuntu-24.04
```

`basic-commands.md`: "For a valid list of distribution names, run `wsl --list --online`." Use the name that list shows
for Ubuntu 24.04; `Ubuntu-24.04` is what it is expected to be (unconfirmed). Check that the distribution runs as WSL 2:

```powershell
wsl -l -v
```

Expected: the Ubuntu 24.04 line shows `VERSION 2`. Then install ROS 2 Jazzy (`ros-jazzy-ros-base`) in it by the ROS 2
documentation's Ubuntu (deb packages) instructions, and build this repository as in the root `README.md`:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build
```

**B. Docker inside WSL with `ros:jazzy-ros-base`.** Install Docker Engine in the Ubuntu distribution by Docker's
instructions for Ubuntu. Docker's service needs systemd; `wsl-config.md` gives the `/etc/wsl.conf` setting:

```ini
[boot]
systemd=true
```

and: "You will then need to close your WSL distribution using `wsl.exe --shutdown` from PowerShell to restart your
WSL instances." Then build and test the repository in the CI image, from the repository root:

```bash
docker run --rm -it --network host -v "$PWD":/ws -w /ws ros:jazzy-ros-base bash
```

Inside the container (the same commands as `.github/workflows/ci.yml`):

```bash
apt-get update && apt-get install -y python3-colcon-common-extensions python3-pytest python3-pandas
rosdep update && rosdep install --from-paths src --ignore-src -y
source /opt/ros/jazzy/setup.bash && colcon build && colcon test && colcon test-result --verbose
```

`--network host` puts the container on WSL's network, which in mirrored mode is the laptop's. Expected: `colcon
test-result` reports 0 errors and 0 failures. The bridge's reply process wants real-time limits; in Docker they come
from `cap_add: SYS_NICE` and `ulimits` as in `deploy/compose.yaml` (`docs/DEPLOY.md` step 3).

## 2. Networking

### 2.1 Mirrored mode

`networking.md`: "On machines running Windows 11 22H2 and higher you can set `networkingMode=mirrored` under `[wsl2]`
in the `.wslconfig` file to enable mirrored mode networking. Enabling this changes WSL to an entirely new networking
architecture which has the goal of 'mirroring' the network interfaces that you have on Windows into Linux". One of the
listed benefits: "Connect to WSL directly from your local area network (LAN)".

`wsl-config.md`: `.wslconfig` lives in `%UserProfile%\.wslconfig` ("The .wslconfig file does not exist by default.
It must be created and stored in your `%UserProfile%` directory"). The `networkingMode` and `firewall` settings
"Require Windows 11 version 22H2 or higher." Create `%UserProfile%\.wslconfig` with:

```ini
[wsl2]
networkingMode=mirrored
```

Restart WSL ("you may need to run `wsl --shutdown` to shut down the WSL 2 VM and then restart your WSL instance for
these changes to take effect"):

```powershell
wsl --shutdown
```

**Check.** Compare the laptop's address on the KR C4 network in Windows and in WSL:

```powershell
ipconfig
```

```bash
ip -4 addr
```

Expected: `ip -4 addr` in WSL lists the same IPv4 address (`<LAPTOP_IP>`) on an interface as `ipconfig` shows for
the KR C4 network adapter. (What the mirrored interface is called in Linux is not stated on the page: unconfirmed.)

### 2.2 Hyper-V firewall rule for the RSI port

`networking.md`, in the mirrored mode section: "Run the following command in PowerShell window with admin privileges
to Configure Hyper-V firewall settings to allow inbound connections", with two commands. The first allows all
inbound traffic to WSL:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

The second adds one rule; Microsoft's example is TCP port 80:

```powershell
New-NetFirewallHyperVRule -Name "MyWebServer" -DisplayName "My Web Server" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol TCP -LocalPorts 80
```

`{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}` is the WSL VM creator ID. Use one rule for the RSI port only, not the
allow-all setting. The UDP rule is the same command with `-Protocol UDP` (administrator PowerShell):

```powershell
New-NetFirewallHyperVRule -Name "RSI-UDP" -DisplayName "KUKA RSI (UDP)" -Direction Inbound -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts <RSI_PORT>
```

`networking.md` also says: "On machines running Windows 11 22H2 and higher, with WSL 2.0.9 and higher, the Hyper-V
firewall feature will be turned on by default." Check the WSL version:

```powershell
wsl --version
```

`wsl-config.md` on `firewall` (default `true`): "Setting this to true allows the Windows Firewall rules, as well as
rules specific to Hyper-V traffic, to filter WSL network traffic." Whether the Windows Defender Firewall also needs
an inbound rule for the port, besides the Hyper-V rule, is not stated on the pages read: **unconfirmed**. The latency
test (section 4) shows whether packets get through: it reports `no answer` when they do not.

### 2.3 Why the default NAT mode cannot work

In NAT mode, `networking.md` says: "WSL 2 has a virtualized ethernet adapter with its own unique IP address", and to
reach it from the LAN "you will need to go through the same steps as you would for a regular virtual machine". Its
example is a `netsh interface portproxy add v4tov4` rule on the Windows host. Two reasons rule this out for RSI:

- `netsh interface portproxy` forwards TCP only, and RSI is UDP. This is the statement of the Netsh interface
  portproxy page, which could not be opened when this guide was written: **unconfirmed from the source**. The
  `networking.md` example has no protocol option.
- The KR C4 sends to the address in `RSI_EthernetConfig*.xml` and expects the reply from it. In NAT mode the bridge
  would sit at the VM's own address behind the host, so the controller configuration or a forwarder would have to
  change. Mirrored mode avoids both.

## 3. ELP camera

### 3.1 USB passthrough (usbipd-win)

`connect-usb.md` prerequisites: "Running Windows 11 (Build 22000 or later)"; a Linux kernel "of 5.10.60.1 or higher".
Check the kernel in WSL:

```bash
uname -r
```

To update it, in PowerShell:

```powershell
wsl --shutdown
```

```powershell
wsl --update
```

Install usbipd-win from its release page, or:

```powershell
winget install --interactive --exact dorssel.usbipd-win
```

In an administrator PowerShell, with a WSL command line open ("This will keep the WSL 2 lightweight VM active"):

```powershell
usbipd list
```

```powershell
usbipd bind --busid <BUSID>
```

Then (no administrator prompt needed):

```powershell
usbipd attach --wsl --busid <BUSID>
```

usbipd-win's README: "Sharing a device is persistent; it survives reboots." and "Attaching devices to a client is
non-persistent. You will have to re-attach after a reboot, or when the device resets or is physically
unplugged/replugged." Detach with:

```powershell
usbipd detach --busid <BUSID>
```

**Check** in WSL:

```bash
lsusb
```

Expected: the ELP camera is listed.

### 3.2 Video driver

```bash
sudo modprobe uvcvideo
```

```bash
v4l2-ctl --list-devices
```

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

(`v4l2-ctl` is in the `v4l-utils` package.) Expected: a device node for the ELP (`/dev/video0` or another number;
put it in `video_device`), and an `MJPG` format with `1920x1200` at the frame rate set in `elp_usb_cam.yaml` (60 fps).

Kernel support is the main risk:
- `config-wsl` on `linux-msft-wsl-6.6.y` has `CONFIG_USB_VIDEO_CLASS=m` (uvcvideo as a module) and
  `CONFIG_USBIP_VHCI_HCD=m`. On `linux-msft-wsl-6.1.y` it has `# CONFIG_MEDIA_SUPPORT is not set`: no video driver.
  So a 6.6 kernel or later is needed (`uname -r`, section 3.1).
- Whether the WSL kernel package ships the `uvcvideo` module so that `modprobe` finds it, and whether the camera's
  MJPG stream at 1920×1200 runs over USB/IP without dropped frames, is **unconfirmed**. If `modprobe` fails, the
  usbipd-win wiki (`WSL-support`, "Building your own WSL 2 kernel with additional drivers") describes building a
  kernel with the driver.

### 3.3 usb_cam

```bash
sudo apt install ros-jazzy-usb-cam
```

The example parameters are `src/print_session/config/elp_usb_cam.yaml` (installed to
`share/print_session/config/elp_usb_cam.yaml`). Set `video_device` from `v4l2-ctl --list-devices`, then start the
camera with the bringup (`start_camera:=true camera_params:=<that file>`), or alone for the checks below:

```bash
ros2 run usb_cam usb_cam_node_exe --ros-args -r __node:=elp_camera -r image_raw:=/elp/image_raw --params-file src/print_session/config/elp_usb_cam.yaml
```

**Checks:**

| Check | Command | Expected |
|---|---|---|
| Frame rate | `ros2 topic hz /elp/image_raw` | average rate at the configured `framerate` (60) |
| Dropped frames | `ros2 topic hz /elp/image_raw` (its `max` interval) | `max` below 2 / framerate (33 ms at 60 fps); a longer interval is at least one dropped frame |
| Stamp delay | `ros2 topic delay /elp/image_raw` | now − `header.stamp`, steady over minutes; it is the capture-to-ROS delay. A drift means the clock stepped (see `print_session/README.md`, Camera) |

```bash
ros2 topic hz /elp/image_raw
```

```bash
ros2 topic delay /elp/image_raw
```

No expected value for the stamp delay is known yet; record it in the first session's notes.

## 4. Latency test

It measures the RSI link before the robot is involved: `krc_sim` stands in for the KR C4 and sends one packet every
4 ms to the bridge on the laptop, over the cable and NIC the KR C4 will use.

**Bridge side** (the laptop, in WSL): `krc_rsi_bridge` alone, closed mode, the limits from `config/e1.yaml`. The
bridge binds the host and port of the file's `krc_rsi_bridge` section (`0.0.0.0`, `59152`), or the ones given:

```bash
ros2 launch krc_replay latency_bridge.launch.py params_file:=config/e1.yaml port:=<RSI_PORT>
```

Expected log line: `[latency_bridge] krc_rsi_bridge closed mode on 0.0.0.0:<RSI_PORT>, window 12..100 (speed floor 12), ov_default
40, no controller`.

**Simulator side** (a second Linux machine with this repository built, on the laptop's KR C4 network port in place of
the controller):

```bash
ros2 run krc_replay latency_test --host <LAPTOP_IP> --port <RSI_PORT> --duration 600 --params-file config/e1.yaml
```

Options: `--wait` (seconds to wait for the bridge's first answer, default 10), `--json <file>` (also write the
report). The simulator moves along the pattern at the override the bridge replies with, so the print window opens as
on the robot. With no controller the bridge answers `ov_default` (40) inside the window and `ov_outside` (100)
outside it.

The report:

```
RSI latency test: krc_sim -> bridge <LAPTOP_IP>:<RSI_PORT>, 600 s at 4 ms
  packets sent            ...
  replies on time         ...
  late replies            ... (longest run ...; Timeout at 100)
  Timeout                 no
  round trip (on time)    p50 ... ms, p99 ... ms, max ... ms
  nonzero RKorr           0
  OvCmd outside 12..100   0
  replies without OvCmd   0
  OvCmd values seen       40, 100
  stale replies           ... (old IPOC); simulator overruns ...
  result                  PASS
```

| Reading | Expected |
|---|---|
| Timeout | `no` |
| nonzero RKorr, OvCmd outside the window, replies without OvCmd | 0 (safety invariant 1 and 2; `PASS` needs all three and no Timeout) |
| late replies | at most 0.1 % of packets sent and a longest run below 10: the legacy lab criterion (`legacy/realtime_cl_261023/rtt_shadow_test.py`). CI on a shared runner asserts 1 % (`src/krc_rsi_bridge/test/loop_helpers.py`) |
| round trip p99 | well below 4 ms. CI on loopback (one host, no cable): p50 0.08 ms, p99 0.15 ms, max 0.22 ms; no laptop figure yet |
| simulator overruns | 0. If not, the simulator machine missed its own 4 ms schedule, and late replies are not the laptop's fault |

Exit code: 0 = PASS, 1 = FAIL, 2 = no answer within `--wait` (bridge not running, wrong port, or the firewall drops
UDP: section 2.2).

On one host (as CI does), give the bridge `host:=127.0.0.1` and the simulator `--host 127.0.0.1`.

## 5. What cannot run at the same time

- **The Windows pipeline and the ROS stack.** In mirrored mode WSL and Windows share the laptop's ports.
  `wsl-config.md` describes `ignoredPorts` (under `[experimental]`, mirrored mode only) as the ports "Linux
  applications can bind to, even if that port is used in Windows", so without it a port that Windows uses cannot be
  bound in Linux. The legacy pipeline (`legacy/realtime_cl_261023/rsi_proc.py`) and the bridge both bind the RSI
  port. Stop the Windows pipeline before starting the bridge, and do not set `ignoredPorts` for the RSI port: the
  KR C4's packets must reach exactly one program.
- **The camera on Windows and in WSL.** `connect-usb.md`: "as long as the USB device is attached to WSL, it cannot be
  used by Windows." Detach it (`usbipd detach --busid <BUSID>`) before the Windows pipeline uses it.

## 6. Shadow mode needs a program on RSI_MIN

In shadow mode the bridge's reply carries no `<OvCmd>`. The print program loads `RSI_OV4`
(`RSI_CREATE("RSI_OV4.rsi")`), where OvCmd goes through `LIMIT1` (1..100) to `$OV_PRO`. With no OvCmd the input is
0, `LIMIT1` raises it to 1, and **the robot runs at 1 % override** (`legacy/README.md`;
`legacy/realtime_cl_261023/run_cl.py` lines 4 and 7). `print_session` logs this warning at start-up in shadow mode.
Run shadow mode only with a print program on `RSI_MIN`.

## Untested steps

| Step | Status |
|---|---|
| 1 A / B: WSL 2 install, ROS 2 Jazzy or Docker in WSL | untested on the laptop; the Docker commands are CI's |
| 2.1 mirrored mode, same address in WSL | untested |
| 2.2 Hyper-V UDP rule; whether a Windows Defender Firewall rule is also needed | untested; the second point unconfirmed |
| 2.3 portproxy is TCP only | unconfirmed from the Microsoft page |
| 3.1 usbipd bind / attach of the ELP | untested |
| 3.2 `uvcvideo` in the WSL kernel, MJPG 1920×1200 over USB/IP | untested, unconfirmed |
| 3.3 usb_cam frame rate, dropped frames, stamp delay | untested; no expected stamp delay yet |
| 4 latency test over the cable to the laptop | untested; only the loopback run (one Linux host) runs, in CI |
| 4 real-time limits of the reply process in WSL (`rt_priority`, `cpu`) | untested |
| 5 port and camera conflicts | taken from the Microsoft pages, not observed |
