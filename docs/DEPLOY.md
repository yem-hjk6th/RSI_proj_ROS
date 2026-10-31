# Deployment: edge PC and client

The design is in `docs/DESIGN.md` §3. Every loop with a deadline closes on the **edge PC**. The **client** handles
visualisation and operation only, and it is never in the control path.

```
 KR C4 (KLI) ──wired── edge PC ──wired── cameras (USB)
                         │
                   LAN or Wi-Fi
                         │
                       client
```

## Which links must stay wired

| Link | Medium | Why |
|---|---|---|
| KR C4 ↔ edge, **RSI** (UDP, 4 ms) | **wired only**, ideally a dedicated NIC with no switch or a small unmanaged switch | Every packet needs a reply within the 4 ms cycle. 100 late replies stop RSI. Wi-Fi jitter and retransmissions make that certain. **RSI never runs over Wi-Fi.** |
| KR C4 ↔ edge, **EKI** (TCP) | wired | There is no 4 ms deadline, and `sps.sub` falls back to `EKI_OV_SAFE` when the link is lost. A wireless link would still cause needless fallbacks during a print. |
| Cameras ↔ edge | USB, wired | The width readout feeds the control loop. |
| Edge ↔ client | LAN or **Wi-Fi allowed** | Visualisation, parameters, start/stop and bag playback carry no deadline. A lost client never stops the print, because the safety logic lives in the bridge on the edge. |

Keep `/krc/override_cmd` publishers on the edge. A controller node on the client would put Wi-Fi into the control path.
The bridge's heartbeat watchdog would catch it, by ramping to `ov_default`, but that is a fallback, not a design.

## Edge PC setup (Ubuntu 24.04)

1. **OS:** Ubuntu 24.04 LTS Server or Desktop.
2. **Low-latency kernel**, or PREEMPT_RT:
   ```bash
   sudo apt install linux-lowlatency
   ```
   Reboot and check that `uname -v` shows `PREEMPT_DYNAMIC`, with the lowlatency flavour in `uname -r`. Ubuntu Pro also
   ships a real-time kernel (`sudo pro enable realtime-kernel`) if the low-latency kernel is not enough.
3. **Real-time limits** for the user that runs the bridge, outside Docker:
   ```bash
   sudo groupadd -f realtime
   sudo usermod -aG realtime "$USER"
   ```
   Then add this to `/etc/security/limits.d/99-realtime.conf`:
   ```
   @realtime - rtprio 99
   @realtime - memlock unlimited
   ```
   The `edge` compose service sets the same limits with `cap_add: SYS_NICE` and `ulimits` (`rtprio: 99`,
   `memlock: -1`).
4. **CPU and power:**
   - set the performance governor: `sudo apt install linux-tools-common`, then
     `sudo cpupower frequency-set -g performance`;
   - disable deep C-states in the BIOS where possible;
   - optionally isolate one core for the RSI reply process (`isolcpus=` or a `cset` shield).
5. **Network:**
   - put the KR C4 on its own NIC with a static address in the KLI subnet;
   - disable power management on that NIC;
   - keep the client on a different NIC or VLAN;
   - put addresses only in the site parameter files (`deploy/params/*.yaml`, ignored by git), never in the repository.
6. **Docker:** install Docker Engine and the compose plugin from Docker's apt repository.
7. **Build and start:**
   ```bash
   cp src/krc_eki_bridge/config/example.yaml deploy/params/krc_eki_bridge.yaml
   ```
   Fill in the placeholders, then start the edge service:
   ```bash
   docker compose -f deploy/compose.yaml up --build -d edge
   ```
   - By default `edge` runs the EKI bridge (route C). Set `EDGE_CMD` to launch another stack, e.g. the RSI bridge once
     it exists.
   - `edge` also starts the Fast DDS Discovery Server on UDP 11811 (`START_DISCOVERY_SERVER=1`).
8. **Firewall:** allow UDP 11811 (discovery) and the DDS ports from the client subnet, plus the RSI/EKI port from the
   controller NIC.

## Client setup

The client can be any Ubuntu 24.04 machine with Docker, or with a native ROS 2 Jazzy install.

```bash
EDGE_IP=<EDGE_IP> docker compose -f deploy/compose.yaml run --rm client
```

Then, inside the container:

```bash
ros2 topic hz /krc/state
```

### Client on Wi-Fi: Fast DDS Discovery Server

Plain DDS discovery multicasts to every participant, and Wi-Fi handles multicast poorly. With a Discovery Server,
every participant talks only to the server on the edge.

- **Edge**: `ROS_DISCOVERY_SERVER=127.0.0.1:11811`, already set in `compose.yaml`. The XML equivalent is
  `deploy/fastdds/discovery_server.xml`.
- **Client**: `ROS_DISCOVERY_SERVER=<EDGE_IP>:11811` and `ROS_SUPER_CLIENT=true`, so `ros2 topic list` and rqt see the
  whole graph. The XML equivalent is `deploy/fastdds/discovery_client.xml`; replace `<EDGE_IP>` and set
  `FASTRTPS_DEFAULT_PROFILES_FILE`.
- **Every** edge process must use the Discovery Server. A process left on multicast discovery is invisible to the
  client.
- Send images as compressed transport: `image_transport` `compressed` plugin, `ros-jazzy-image-transport-plugins`.
  Raw images over Wi-Fi saturate the link.
- Prefer the default best-effort QoS for viewers of `/krc/state` and images on the client.

### Alternative: `rmw_zenoh`

`rmw_zenoh` replaces DDS discovery with a Zenoh router, which tolerates Wi-Fi and NAT better.

- Install `ros-jazzy-rmw-zenoh-cpp` on both machines. It is not in the image yet; add it to `deploy/Dockerfile`
  if you choose this route.
- On the edge, run the router (`ros2 run rmw_zenoh_cpp rmw_zenohd`) and start every node with
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp`.
- On the client, set `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, and point the session config at the edge router:
  `ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<EDGE_IP>:7447"]'`.
- All processes in one graph must use the same RMW, so switch the whole edge at once.
- The Zenoh router is a separate process on the edge. It sits in no deadline loop: the bridge's reply loop never
  touches the RMW.

## Checks before a print

1. On the edge, `ros2 topic hz /krc/state` shows the expected rate: 250 Hz for RSI, or the submit rate for EKI.
2. The bridge log shows `connected=True` and a `last_out` source of `control` while the controller node runs.
3. Unplug the client: the print goes on, with no change in `/krc/state` timing.
4. Stop the controller node: the bridge log shows `watchdog`, and the override ramps to `ov_default` over
   `fallback_ramp_s`.
