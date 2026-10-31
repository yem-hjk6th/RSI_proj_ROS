# krc_eki_bridge controller templates: NOT RUN

Nothing in this folder has run on a controller. Each file is headed `NOT RUN`, and the header stays until the file
has been tested on hardware (safety invariant 6 in `CLAUDE.md`).

| File | Role |
|---|---|
| `KrcEki.xml` | EthernetKRL channel `KrcEki`. The controller is the TCP server on `<EKI_PORT>`; it receives `Ext/Ov` and `Ext/Seq` and sends `State/...`. |
| `sps_eki_override.sub` | `sps.sub` user-section snippet. It reads the override without blocking, writes `$OV_PRO` only in AUT/EXT, falls back to `EKI_OV_SAFE` when the link is lost, and re-opens the channel. |
| `eki_send_state.src` | `EkiSendState()`: sends `$POS_ACT`, `$OV_PRO`, `$VEL_ACT`, the extruder rpm, a timer and the mode code. |

`krc_eki_bridge.fake_eki.SpsOverrideModel` is a Python model of the snippet's logic. The tests check it, not
the KRL.

## Wire format

- PC → controller: `<Ext><Ov>75</Ov><Seq>12</Seq></Ext>`. The bridge clamps the value before it sends it, and
  `Seq` goes up by one with every message.
- Controller → PC: `<State><Pos X=".." Y=".." Z=".." A=".." B=".." C=".." /><Ov>..</Ov><Vel>..</Vel><Rpm>..</Rpm><T>..</T><Mode>..</Mode></State>`.
- Neither direction carries a motion target.

## KUKA rules these templates rely on

I found no public, official KUKA source for EthernetKRL or for the system-variable rules below; the
KUKA manuals sit behind a login. So the table marks a rule **unverified** unless a file in this
repository ran with it on the lab's KR C4 (KSS 8.3). Third-party pages that describe a rule are listed
as leads only. They are not authoritative, and some of them contradict each other.

| # | Rule | Status | Source |
|---|---|---|---|
| 1 | `sps.sub` layout: `USER DECL`, `USER INIT` and `USER PLC` folds inside `DEF SPS()`; the `LOOP` runs cyclically in the submit interpreter | used on the lab controller | [`legacy/krc_config/sps.sub`](../../../legacy/krc_config/sps.sub) |
| 2 | `$MODE_OP` and the value `#EX` (External Automatic) | used on the lab controller | [`legacy/krc_config/sps.sub`](../../../legacy/krc_config/sps.sub) (`IF $MODE_OP==#EX`) |
| 3 | `#T1`, `#T2` and `#AUT` are the other `$MODE_OP` values | unverified | lead: [industrialmonitordirect: override bypass in sps.sub](https://industrialmonitordirect.com/blogs/knowledgebase/kuka-krc2-ov-pro-speed-override-bypass-fix-in-spssub) |
| 4 | The submit interpreter may write `$OV_PRO` (INT, 0..100) | unverified | lead: same page as #3 |
| 5 | Writing `$OV_PRO` takes effect only in AUT and EXT. The snippet writes it only then and holds the last value in T1/T2. | unverified (project design assumption, `docs/DESIGN.md` §2) | – |
| 6 | `$VEL_ACT` is the current path velocity in m/s | unverified; the legacy file reads `$VEL_ACT` but its unit is not checked | [`legacy/krc_config/sps.sub`](../../../legacy/krc_config/sps.sub) (`$SEN_PREA[1] = $VEL_ACT`) |
| 7 | `$POS_ACT` is the current Cartesian pose (X, Y, Z mm; A, B, C deg) | unverified | – |
| 8 | `$ROB_TIMER` is a free-running ms timer | unverified. If it is wrong, drop `State/T`: the bridge then uses PC time and sends `ipoc = 0`. | – |
| 9 | EKI XML layout: `CONFIGURATION/EXTERNAL/TYPE`, `INTERNAL` (`ENVIRONMENT`, `BUFFERING`, `BUFFSIZE`, `ALIVE Set_Flag`, `IP`, `PORT`, `PROTOCOL`, `MESSAGES`), and `RECEIVE/XML` and `SEND/XML` with `ELEMENT Tag` paths, `@` for attributes | unverified | lead: [industrialmonitordirect: connection state detection](https://industrialmonitordirect.com/blogs/knowledgebase/kukaethernetkrl-connection-state-detection-without-errors) |
| 10 | `EXTERNAL/TYPE = Client` makes the controller the TCP server | unverified | – |
| 11 | `ENVIRONMENT = Submit` lets `sps.sub` use the channel | unverified (third-party sources disagree) | lead: [industrialmonitordirect: EKI functions in sps.sub](https://industrialmonitordirect.com/blogs/knowledgebase/kuka-eki-functions-in-spssub-compiler-error-fix) |
| 12 | `EKI_CheckBuffer` returns at once, with the number of buffered messages in `.Buff` | unverified | lead: [industrialmonitordirect: empty element memory](https://industrialmonitordirect.com/blogs/knowledgebase/eki-access-to-empty-elementmemory-error-resolution-kuka-krc4) |
| 13 | `EKI_GetInt` takes the oldest buffered value of one element and does not block | unverified | lead: same page as #12 |
| 14 | `EKI_Open` in server mode does not block the submit interpreter while it waits for the client | **unverified, critical**: if it blocks, `sps.sub` stalls. Check this first, on a bench controller. | – |
| 15 | `ALIVE Set_Flag` keeps the `$FLAG` TRUE while the connection is up | unverified | lead: same page as #9 |
| 16 | After a lost connection, `EKI_Clear`, `EKI_Init` and `EKI_Open` make the server accept a new client | unverified | lead: [industrialmonitordirect: connection already initialized](https://industrialmonitordirect.com/blogs/knowledgebase/kuka-ethernetkrl-connection-already-initialized-fix) |
| 17 | `EKI_SetReal` and `EKI_SetInt` fill a SEND element, and `EKI_Send(channel, "State")` sends that subtree as one message | unverified | – |
| 18 | A submit program may call a `GLOBAL DEF` with no motion (`EkiSendState`). If not, paste its body into the snippet. | unverified | – |
| 19 | The submit cycle is about 12 ms (`EKI_LOSS_CYCLES = 40` ≈ 0.5 s) | unverified; measure it on site | – |

## Before the first hardware test

1. Check rules 14, 11 and 5, in that order, against the EthernetKRL manual for the installed version, then on a
   bench controller.
2. Set `EKI_OV_SAFE` to the print's `ov_default`, and make `EKI_OV_MIN` and `EKI_OV_MAX` no wider than the bridge's
   `ov_min`/`ov_floor` and `ov_max`.
3. Follow the legacy order: shadow mode first (the bridge connected, the `$OV_PRO` write commented out), then an air
   test, then printing.
