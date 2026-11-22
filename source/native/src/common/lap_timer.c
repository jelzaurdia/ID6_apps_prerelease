#include "lap_timer.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LAP_PI 3.14159265358979323846
#define LAP_EARTH_M 6371008.8              /* mean Earth radius */
#define LAP_DEG_M (LAP_PI / 180.0 * LAP_EARTH_M)

static int valid_point(double lat, double lon) {
    return isfinite(lat) && isfinite(lon) && fabs(lat) <= 90 && fabs(lon) <= 180;
}

/* Equirectangular projection around (lat0, lon0): x east, y north, metres. Its error is far below GPS
 * noise at track scale. */
static void local_xy(double lat0, double lon0, double lat, double lon, double *x, double *y) {
    double dlon = lon - lon0;
    if (dlon > 180) dlon -= 360;
    else if (dlon < -180) dlon += 360;
    *x = dlon * LAP_DEG_M * cos(lat0 * LAP_PI / 180.0);
    *y = (lat - lat0) * LAP_DEG_M;
}

void lap_config_default(lap_config *cfg) {
    cfg->min_lap_ms = 10000;
    cfg->max_gap_ms = 3000;
    cfg->max_speed_mps = 120;
    cfg->jump_slack_m = 30;
}

double lap_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double dlon = lon2 - lon1, x, y;
    if (dlon > 180) dlon -= 360;
    else if (dlon < -180) dlon += 360;
    x = dlon * LAP_DEG_M * cos((lat1 + lat2) / 2 * LAP_PI / 180.0);
    y = (lat2 - lat1) * LAP_DEG_M;
    return sqrt(x * x + y * y);
}

int lap_gate_from_motion(lap_gate *gate, double from_lat, double from_lon, double lat, double lon,
                        double half_width_m) {
    double x, y, heading;
    if (!gate || !valid_point(from_lat, from_lon) || !valid_point(lat, lon) ||
        !isfinite(half_width_m) || half_width_m <= 0) return 0;
    local_xy(lat, lon, from_lat, from_lon, &x, &y);   /* where the car came from, seen from the line */
    if (x * x + y * y < 1.0) return 0;
    heading = atan2(-x, -y) * 180.0 / LAP_PI;          /* direction of travel: from -> line */
    if (heading < 0) heading += 360;
    gate->lat = lat;
    gate->lon = lon;
    gate->heading_deg = heading;
    gate->half_width_m = half_width_m;
    return 1;
}

int lap_gate_crossing(const lap_gate *g, const lap_fix *a, const lap_fix *b, double *fraction) {
    double ax, ay, bx, by, h, ux, uy, da, db, la, lb, f, lateral;
    if (!g || !a || !b || !valid_point(a->lat, a->lon) || !valid_point(b->lat, b->lon)) return 0;
    local_xy(g->lat, g->lon, a->lat, a->lon, &ax, &ay);
    local_xy(g->lat, g->lon, b->lat, b->lon, &bx, &by);
    h = g->heading_deg * LAP_PI / 180.0;
    ux = sin(h);
    uy = cos(h);
    da = ax * ux + ay * uy;           /* signed distance along the direction of travel */
    db = bx * ux + by * uy;
    if (!(da < 0 && db >= 0)) return 0;
    la = ax * uy - ay * ux;           /* signed distance along the line */
    lb = bx * uy - by * ux;
    f = -da / (db - da);              /* in (0, 1] */
    lateral = la + f * (lb - la);
    if (!isfinite(lateral) || fabs(lateral) > g->half_width_m) return 0;
    if (fraction) *fraction = f;
    return 1;
}

void lap_session_init(lap_session *s, const lap_gate *start, const lap_gate *finish, const lap_config *cfg) {
    memset(s, 0, sizeof *s);
    s->start = *start;
    s->circuit = finish == 0;
    s->finish = finish ? *finish : *start;
    if (cfg) s->cfg = *cfg;
    else lap_config_default(&s->cfg);
    s->state = LAP_WAITING;
}

static uint64_t crossing_time(const lap_fix *a, const lap_fix *b, double fraction) {
    return a->t_ms + (uint64_t)(fraction * (double)(b->t_ms - a->t_ms) + 0.5);
}

unsigned lap_session_update(lap_session *s, const lap_fix *f) {
    unsigned events = 0;
    double fraction;
    uint64_t dt;
    if (!s || !f || !valid_point(f->lat, f->lon)) return 0;
    if (!s->have_prev) {
        s->prev = *f;
        s->have_prev = 1;
        return 0;
    }
    if (f->t_ms <= s->prev.t_ms) return 0;
    dt = f->t_ms - s->prev.t_ms;
    if (dt > s->cfg.max_gap_ms ||
        lap_distance_m(s->prev.lat, s->prev.lon, f->lat, f->lon) >
            s->cfg.max_speed_mps * (double)dt / 1000.0 + s->cfg.jump_slack_m) {
        /* A dropout or a jump: a crossing hidden in it cannot be timed. */
        if (s->state == LAP_RUNNING && !s->lap_invalid) {
            s->lap_invalid = 1;
            events |= LAP_EVENT_INVALID;
        }
        s->prev = *f;
        return events;
    }
    if (s->state == LAP_RUNNING && lap_gate_crossing(&s->finish, &s->prev, f, &fraction)) {
        uint64_t t = crossing_time(&s->prev, f, fraction);
        if (t - s->lap_start_ms >= s->cfg.min_lap_ms) {
            uint64_t lap = t - s->lap_start_ms;
            s->last_ms = lap;
            s->last_valid = !s->lap_invalid;
            ++s->laps;
            if (s->last_valid && (!s->best_ms || lap < s->best_ms)) {
                s->best_ms = lap;
                events |= LAP_EVENT_BEST;
            }
            if (s->circuit) {
                events |= LAP_EVENT_LAP;
                s->lap_start_ms = t;          /* the line that ended this lap starts the next */
                s->lap_invalid = 0;
            } else {
                events |= LAP_EVENT_FINISH;
                s->state = LAP_WAITING;
            }
        }
    } else if (s->state == LAP_WAITING && lap_gate_crossing(&s->start, &s->prev, f, &fraction)) {
        s->state = LAP_RUNNING;
        s->lap_start_ms = crossing_time(&s->prev, f, fraction);
        s->lap_invalid = 0;
        events |= LAP_EVENT_START;
    }
    s->prev = *f;
    return events;
}

uint64_t lap_session_elapsed(const lap_session *s, uint64_t now_ms) {
    return s && s->state == LAP_RUNNING && now_ms > s->lap_start_ms ? now_ms - s->lap_start_ms : 0;
}

int lap_gate_format(const lap_gate *gate, char *out, size_t len) {
    int n;
    if (!gate || !out || !len) return -1;
    n = snprintf(out, len, "LTG1 %.7f %.7f %.2f %.2f\n", gate->lat, gate->lon, gate->heading_deg,
                 gate->half_width_m);
    return n < 0 || (size_t)n >= len ? -1 : n;
}

static int number(const char **cursor, double *out) {
    char *end;
    while (isspace((unsigned char)**cursor)) ++*cursor;
    *out = strtod(*cursor, &end);
    if (end == *cursor || !isfinite(*out)) return 0;
    *cursor = end;
    return 1;
}

int lap_gate_parse(const char *text, lap_gate *gate) {
    const char *p = text;
    lap_gate g;
    if (!text || !gate) return 0;
    while (isspace((unsigned char)*p)) ++p;
    if (strncmp(p, "LTG1", 4) || !isspace((unsigned char)p[4])) return 0;
    p += 4;
    if (!number(&p, &g.lat) || !number(&p, &g.lon) || !number(&p, &g.heading_deg) ||
        !number(&p, &g.half_width_m)) return 0;
    while (isspace((unsigned char)*p)) ++p;
    if (*p || !valid_point(g.lat, g.lon) || g.heading_deg < 0 || g.heading_deg >= 360 ||
        g.half_width_m <= 0 || g.half_width_m > 200) return 0;
    *gate = g;
    return 1;
}
