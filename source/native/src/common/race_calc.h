/* Race screen calculations from the two readings every drive app already has: engine RPM (OBD 01 0C)
 * and the DME's integer km/h (OBD 01 0D). Pure C99: no HMI, no diagnostics, no clocks; the caller passes
 * monotonic milliseconds. 0-100 and 100-200 timing stays in launch_timer.
 *
 *  - Gear: from RPM per km/h. The steps between gears are fixed by the gearbox (ZF 8HP by default), so the
 *    overall factor (final drive and tyre) is learned from driving and needs no ECU access. Unknown (0)
 *    at a standstill, while the converter slips or during a shift.
 *  - Longitudinal acceleration: a least-squares slope of speed over the last second, in g.
 *  - Run: distance from a standing start; times and trap speeds at 100 m and 402 m (a quarter mile), peak g
 *    and top speed, for a summary after the run. */
#ifndef ID6_RACE_CALC_H
#define ID6_RACE_CALC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { RC_MAX_GEARS = 10, RC_K_BINS = 400, RC_ACCEL_SAMPLES = 24 };

typedef struct {
    unsigned gears;
    double ratio[RC_MAX_GEARS];  /* gearbox ratios, first gear first */
    double k_prior;              /* RPM per km/h at a 1.00 ratio if known, else 0 (learn it) */
    double tolerance;            /* relative RPM error still matched to a gear */
} rc_gear_config;

typedef struct {
    rc_gear_config cfg;
    unsigned votes[RC_K_BINS];   /* candidate overall factors, log spaced */
    unsigned samples;            /* steady samples seen */
    double k;                    /* learned factor, 0 until confident */
    double last_ratio;
    unsigned steady;
    int gear, candidate;
    unsigned candidate_count;
} rc_gear;

/* ZF 8HP ratios (4.714 ... 0.667). k_prior 0 learns the factor from driving. */
void rc_gear_config_zf8(rc_gear_config *cfg, double k_prior);
void rc_gear_init(rc_gear *g, const rc_gear_config *cfg);
/* rpm or kph < 0 means unavailable. Returns the gear (1..gears) or 0 when unknown. */
int rc_gear_update(rc_gear *g, int rpm, int kph);
/* The factor in use: learned, else the prior, else 0. */
double rc_gear_factor(const rc_gear *g);

typedef struct {
    uint64_t t_ms[RC_ACCEL_SAMPLES];
    double mps[RC_ACCEL_SAMPLES];
    unsigned count, next;
    uint64_t window_ms;
} rc_accel;

void rc_accel_init(rc_accel *a);
void rc_accel_add(rc_accel *a, uint64_t t_ms, int kph);
/* 1 and *g when at least 4 samples span 500 ms inside the 1 s window. */
int rc_accel_g(const rc_accel *a, double *g);

enum { RC_IDLE, RC_READY, RC_RUNNING, RC_DONE };                 /* run state */
enum { RC_EV_READY = 1, RC_EV_START = 2, RC_EV_100M = 4,         /* rc_run_update result bits */
       RC_EV_402M = 8, RC_EV_DONE = 16 };

typedef struct {
    int state;
    int last_kph, peak_kph, first_kph, change_kph, start_refined;
    uint64_t last_ms, zero_since_ms, last_zero_ms, start_ms, first_ms, change_ms;
    double dist_m;                    /* since the launch */
    uint32_t t100_ms, t402_ms;        /* 0 until reached */
    double v100_kph, v402_kph;
    double peak_g;
    int completed;                    /* reached 402 m */
    rc_accel accel;
} rc_run;

void rc_run_init(rc_run *r);
/* kph < 0 means unavailable. Returns RC_EV_* bits. */
unsigned rc_run_update(rc_run *r, uint64_t now_ms, int kph);

#ifdef __cplusplus
}
#endif
#endif
