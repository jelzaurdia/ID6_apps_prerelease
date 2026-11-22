/* GPS lap timing for a TrackDeck plugin. Pure C99: no HMI, no diagnostics, no I/O.
 *
 * A gate is a start/finish line: a centre point, the direction of travel across it, and a half-width.
 * Crossings count only in the direction of travel and within the half-width. The crossing time is
 * interpolated between the two fixes on either side of the line, so a 1 Hz GPS still times laps to a
 * fraction of a second.
 *
 * Two layouts:
 *  - circuit: one gate is both start and finish; every crossing ends a lap and starts the next.
 *  - point to point: gate A starts a run, gate B finishes it (sprints, hill climbs, custom A/B laps).
 *
 * The caller feeds fixes in time order (lt_session_update) and shows the results it returns. A fix
 * that is too far from the previous one in time or distance is a discontinuity: no crossing is detected
 * across it, and a lap in progress is marked invalid so it cannot become the best lap. */
#ifndef ID6_LAP_TIMER_H
#define ID6_LAP_TIMER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double lat, lon;          /* WGS84 degrees */
    uint64_t t_ms;            /* fix time, monotonic milliseconds */
} lt_fix;

typedef struct {
    double lat, lon;          /* centre of the line, WGS84 degrees */
    double heading_deg;       /* direction of travel across the line, degrees clockwise from north */
    double half_width_m;      /* the line reaches this far either side of the centre */
} lt_gate;

typedef struct {
    uint64_t min_lap_ms;      /* crossings sooner than this after a start are ignored (GPS jitter) */
    uint64_t max_gap_ms;      /* fixes further apart in time are a discontinuity */
    double max_speed_mps;     /* fixes implying a faster move are a discontinuity... */
    double jump_slack_m;      /* ...beyond this allowance for position noise, which dominates at 5-10 Hz */
} lt_config;

enum { LT_WAITING, LT_RUNNING };                  /* session state */
enum { LT_EVENT_START = 1, LT_EVENT_LAP = 2,      /* lt_session_update result bits */
       LT_EVENT_FINISH = 4, LT_EVENT_BEST = 8, LT_EVENT_INVALID = 16 };

typedef struct {
    lt_gate start, finish;
    int circuit;              /* 1: start is also the finish */
    lt_config cfg;
    int state;
    int have_prev, lap_invalid;
    lt_fix prev;
    uint64_t lap_start_ms;    /* interpolated start of the lap or run in progress */
    uint64_t last_ms, best_ms;/* 0 when none */
    int last_valid;
    unsigned laps;            /* completed laps or runs, valid or not */
} lt_session;

/* 10 s minimum lap, 3 s gap, 120 m/s plus 30 m of noise allowance. */
void lt_config_default(lt_config *cfg);

/* Distance in metres on a local plane; accurate for the few kilometres of a track. */
double lt_distance_m(double lat1, double lon1, double lat2, double lon2);

/* A gate at (lat, lon) facing the direction from (from_lat, from_lon) to (lat, lon), as when the driver
 * marks the line while driving across it. 0 when the two points are less than 1 m apart or invalid. */
int lt_gate_from_motion(lt_gate *gate, double from_lat, double from_lon, double lat, double lon,
                        double half_width_m);

/* 1 when the segment a->b crosses the gate forwards within its half-width; *fraction is where along the
 * segment (0..1]. */
int lt_gate_crossing(const lt_gate *gate, const lt_fix *a, const lt_fix *b, double *fraction);

/* finish == NULL makes a circuit. cfg == NULL uses lt_config_default. */
void lt_session_init(lt_session *s, const lt_gate *start, const lt_gate *finish, const lt_config *cfg);

/* Feed the next fix; returns LT_EVENT_* bits. Out-of-order and invalid fixes are ignored. */
unsigned lt_session_update(lt_session *s, const lt_fix *fix);

/* The running lap's time at now_ms, or 0 while waiting. */
uint64_t lt_session_elapsed(const lt_session *s, uint64_t now_ms);

/* One line, "LTG1 lat lon heading half_width\n", for saving custom gates. Returns the length or -1. */
int lt_gate_format(const lt_gate *gate, char *out, size_t len);
/* 1 when text holds exactly one valid gate line. */
int lt_gate_parse(const char *text, lt_gate *gate);

#ifdef __cplusplus
}
#endif
#endif
