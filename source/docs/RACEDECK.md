# Race screen calculations (proposal)

A "Race" screen would combine ShiftDeck (RPM bar, shift lights) and LaunchDeck (0-100, 100-200) with a gear
indicator, longitudinal g and standing-start distance marks. Everything comes from the two readings the drive
apps already take from the DME: RPM (`01 0C`) and the integer km/h (`01 0D`). No EGS or DSC request, so it also
works on the MOST tunnel without ENET, where those modules are out of reach.

This change adds only the calculations (`native/src/common/race_calc.c`) and host tests; 0-100 and 100-200
timing stay in `launch_timer`. The plugin screen needs a QNX build and an on-car check.

## Gear from RPM per km/h

The steps between gears are fixed by the gearbox (ZF 8HP: 4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839,
0.667), so one unknown overall factor (final drive and tyre) relates RPM per km/h to each gear. A factor can be
configured if known (an F22 M240i is roughly 26.8 RPM per km/h at a 1.00 ratio), or learned:

- Each steady stretch (RPM per km/h within 2 % plus the km/h resolution, from 30 km/h up) casts one vote per
  gear hypothesis for the factor it implies. The true factor gets a vote from every gear driven in; a wrong one
  only where two gearbox steps happen to match.
- The winner is trusted after 6 votes and a 1.5x lead, so cruising in a single gear never decides it.
- The display shows 0 (unknown) at a standstill, while the converter slips or during a shift, and needs the
  same gear twice in a row before it changes.

In a simulated drive the factor was learned within 0.1 % after one pull through first to fifth gear, and every
gear read correctly afterwards (0 of 120 samples wrong).

## Longitudinal g

Least-squares slope of speed over the last second (at least 4 samples spanning 500 ms). With 1 km/h resolution
a half-second window inflated a run's peak by over 10 %; one second keeps a constant 0.46 g run's peak within
about 4 %.

## Standing-start run

- Armed after 300 ms at 0 km/h. The launch instant is extrapolated from the first two speed steps back to
  0 km/h, no earlier than the last standstill sample, as `launch_timer` does.
- Distance by trapezoids from the launch; the 100 m and 402 m (quarter mile) times and trap speeds are
  interpolated between samples. A simulated 4.5 m/s^2 run matched the exact 100 m and 402 m times within 1 ms.
- Peak g and top speed for a summary. The run ends at 402 m, on a lift of 4 km/h below the peak, at a stop or
  after 60 s; marks already reached stand.

## Not covered

EGS converter lock-up, transmission torque and DSC lateral g need ENET (HSFZ). Lateral g, a G-ball and lap
timing belong with the GPS-based TrackDeck proposal.
