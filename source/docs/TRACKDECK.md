# TrackDeck: GPS lap timer (proposal)

TrackDeck would time laps on a circuit, or runs between two points, from the head unit's own GPS. It reads
no diagnostics, so it keeps working while an OBD-II dongle or a piggyback tune holds the gateway
([#88](https://github.com/inotbarnacle/ID6_apps_prerelease/issues/88),
[#130](https://github.com/inotbarnacle/ID6_apps_prerelease/issues/130)).

This change adds only the **lap engine** (`native/src/common/lap_timer.c`) and its host tests. The plugin,
its menus and the GPS source are the next steps below and need a QNX build and an on-car check.

## Lap engine

- **Gate**: a start/finish line given by a centre point, the direction of travel and a half-width. A crossing
  counts only forwards and within the half-width, so driving back over the line or passing beside it in the
  paddock does nothing. `lap_gate_from_motion` makes a gate from two fixes, as when the driver marks the line
  while crossing it.
- **Interpolated crossing time**: positions are projected on a local plane; the crossing time is linear between
  the fixes on either side of the line. A 1 Hz GPS still times a lap to a fraction of a second.
- **Circuit** (one gate) or **point to point** (gate A starts, gate B finishes: sprints, hill climbs, the
  "custom A/B lap" idea).
- **Last, best and running lap**. A crossing sooner than `min_lap_ms` (10 s) after the start is ignored, which
  absorbs GPS jitter while standing on the line.
- **Dropouts and jumps**: a time gap over `max_gap_ms` (3 s), or a move faster than `max_speed_mps` (120 m/s)
  plus `jump_slack_m` (30 m) of noise allowance, is a discontinuity. No crossing is detected across it, and the
  lap in progress is marked invalid so it cannot become the best lap.
- **Gate files**: `LTG1 lat lon heading half_width`, one per line, for saved custom gates and a track list.

Pure C99 with no HMI, I/O or diagnostics dependencies; it builds for ARM and from C++98.

### Measured accuracy (simulation)

200 m radius circle at 30 m/s, 28 laps per case, Gaussian position noise on each axis:

| GPS rate | Noise | Mean error | Worst error |
| --- | --- | --- | --- |
| 1 Hz | 0 m | 0 ms | 1 ms |
| 1 Hz | 1.5 m | 47 ms | 126 ms |
| 1 Hz | 3 m | 76 ms | 277 ms |
| 5 Hz | 3 m | 98 ms | 382 ms |
| 10 Hz | 3 m | 105 ms | 253 ms |

Position noise along the track dominates: 3 m at 30 m/s is 0.1 s. A higher rate does not remove it.

### The head unit's GPS is 1 Hz

Read on an `NBTevo_S19322I` car, without opening any device: `dev-navsens` runs as
`/bin/dev-navsens -C/etc/gps_G6_V7_ver20.cfg -Gd=/dev/ser3` on hu-omap and drives a u-blox receiver. The config
sets 115200 baud, the automotive dynamic model and SBAS, and turns NMEA off in favour of binary UBX (NAV-POSLLH,
NAV-VELNED, NAV-SOL, TIMEGPS…) once per solution. It sends no `CFG-RATE`, so the receiver keeps its default of one
solution per second. Raising the rate would mean changing the head unit's system config or commanding the
receiver behind the navigation, so TrackDeck should take 1 Hz as given.

What that means:

- Expect the 1 Hz rows above: tens of milliseconds on average, a few tenths at worst. Good enough to compare
  laps within a session. Not a replacement for a dedicated 10-25 Hz logger (Dragy, RaceBox) when tenths matter.
- Each fix carries the receiver's GPS time of week, so the interpolated crossing does not pick up arrival-time
  jitter. A constant processing delay is the same on every lap and cancels out of lap times.
- Between fixes, the crossing could be refined with the vehicle speed (the drive apps' OBD speed at about 15 Hz,
  or NAV-VELNED). The OBD part is lost while another device holds the port.
- **No inertial data.** The location capabilities the head unit gives CarPlay (iAP2,
  `/pps/services/multimedia/iap2_wifi/location`) list GGA, RMC and vehicle speed, with no accelerometer, gyro or
  heading. Longitudinal g can come from speed, as in `race_calc`. A lateral g from GPS (speed × heading rate) at
  1 Hz is smoothed and late: fine for a corner's peak, too slow for a live g-ball. Live lateral g would need the
  DSC sensors over ENET.

## Next steps

1. **GPS source (to confirm on a car).** `libAppFramework.A.so` on `NBTevo_S19322I` exports the Car Data Service
   API: `cdsIsPropertyAvailable`, `cdsGetPropertyAsync`, `cdsAddPropertyChangedEventHandler`,
   `cdsRemovePropertyChangedEventHandler` and the `invokeCDSUpdate` virtual (slot 30, already in
   `sdk/bmw_list.h`). In BMW's Connected Apps API the position is `navigation.GPSPosition`
   (`{"GPSPosition":{"latitude":…,"longitude":…,"altitude":…}}`) and heading/speed come from
   `navigation.GPSExtendedInfo`; the exact names on NBT Evo still need checking (the receiver itself is 1 Hz,
   see above).
   As #130 found, query a handful of known properties from posted tasks — never walk the whole property list
   on the HMI dispatcher thread.
2. **Plugin** (`TrackDeck`): one screen with the running lap, last and best, a "Ready / cross the line" state,
   and options to mark gate A (and B) at the current position, choose circuit or A→B, and reset bests. Values
   only; nothing is sent to any ECU.
3. **Storage**: saved gates and best laps outside `Plugins`, like the other apps' preferences.
4. **Later**: a small track list, live delta to the best lap, sectors, CSV session logs.
