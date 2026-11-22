#include "lap_timer.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LT_PI 3.14159265358979323846
#define LT_EARTH_M 6371008.8              /* mean Earth radius */
#define LT_DEG_M (LT_PI / 180.0 * LT_EARTH_M)

static int valid_point(double lat, double lon) {
    return isfinite(lat) && isfinite(lon) && fabs(lat) <= 90 && fabs(lon) <= 180;
}

/* Equirectangular projection around (lat0, lon0): x east, y north, metres. Its error is far below GPS
 * noise at track scale. */
static void local_xy(double lat0, double lon0, double lat, double lon, double *x, double *y) {
    double dlon = lon - lon0;
    if (dlon > 180) dlon -= 360;
    else if (dlon < -180) dlon += 360;
    *x = dlon * LT_DEG_M * cos(lat0 * LT_PI / 180.0);
    *y = (lat - lat0) * LT_DEG_M;
}

void lt_config_default(lt_config *cfg) {
    cfg->min_lap_ms = 10000;
    cfg->max_gap_ms = 3000;
    cfg->max_speed_mps = 120;
    cfg->jump_slack_m = 30;
}

double lt_distance_m(double lat1, double lon1, double lat2, double lon2) {
    double dlon = lon2 - lon1, x, y;
    if (dlon > 180) dlon -= 360;
    else if (dlon < -180) dlon += 360;
    x = dlon * LT_DEG_M * cos((lat1 + lat2) / 2 * LT_PI / 180.0);
    y = (lat2 - lat1) * LT_DEG_M;
    return sqrt(x * x + y * y);
}

int lt_gate_from_motion(lt_gate *gate, double from_lat, double from_lon, double lat, double lon,
                        double half_width_m) {
    double x, y, heading;
    if (!gate || !valid_point(from_lat, from_lon) || !valid_point(lat, lon) ||
        !isfinite(half_width_m) || half_width_m <= 0) return 0;
    local_xy(lat, lon, from_lat, from_lon, &x, &y);   /* where the car came from, seen from the line */
    if (x * x + y * y < 1.0) return 0;
    heading = atan2(-x, -y) * 180.0 / LT_PI;          /* direction of travel: from -> line */
    if (heading < 0) heading += 360;
    gate->lat = lat;
    gate->lon = lon;
    gate->heading_deg = heading;
    gate->half_width_m = half_width_m;
    return 1;
}

int lt_gate_crossing(const lt_gate *g, const lt_fix *a, const lt_fix *b, double *fraction) {
    double ax, ay, bx, by, h, ux, uy, da, db, la, lb, f, lateral;
    if (!g || !a || !b || !valid_point(a->lat, a->lon) || !valid_point(b->lat, b->lon)) return 0;
    local_xy(g->lat, g->lon, a->lat, a->lon, &ax, &ay);
    local_xy(g->lat, g->lon, b->lat, b->lon, &bx, &by);
    h = g->heading_deg * LT_PI / 180.0;
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

void lt_session_init(lt_session *s, const lt_gate *start, const lt_gate *finish, const lt_config *cfg) {
    memset(s, 0, sizeof *s);
    s->start = *start;
    s->circuit = finish == 0;
    s->finish = finish ? *finish : *start;
    if (cfg) s->cfg = *cfg;
    else lt_config_default(&s->cfg);
    s->state = LT_WAITING;
}

static uint64_t crossing_time(const lt_fix *a, const lt_fix *b, double fraction) {
    return a->t_ms + (uint64_t)(fraction * (double)(b->t_ms - a->t_ms) + 0.5);
}

unsigned lt_session_update(lt_session *s, const lt_fix *f) {
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
        lt_distance_m(s->prev.lat, s->prev.lon, f->lat, f->lon) >
            s->cfg.max_speed_mps * (double)dt / 1000.0 + s->cfg.jump_slack_m) {
        /* A dropout or a jump: a crossing hidden in it cannot be timed. */
        if (s->state == LT_RUNNING && !s->lap_invalid) {
            s->lap_invalid = 1;
            events |= LT_EVENT_INVALID;
        }
        s->prev = *f;
        return events;
    }
    if (s->state == LT_RUNNING && lt_gate_crossing(&s->finish, &s->prev, f, &fraction)) {
        uint64_t t = crossing_time(&s->prev, f, fraction);
        if (t - s->lap_start_ms >= s->cfg.min_lap_ms) {
            uint64_t lap = t - s->lap_start_ms;
            s->last_ms = lap;
            s->last_valid = !s->lap_invalid;
            ++s->laps;
            if (s->last_valid && (!s->best_ms || lap < s->best_ms)) {
                s->best_ms = lap;
                events |= LT_EVENT_BEST;
            }
            if (s->circuit) {
                events |= LT_EVENT_LAP;
                s->lap_start_ms = t;          /* the line that ended this lap starts the next */
                s->lap_invalid = 0;
            } else {
                events |= LT_EVENT_FINISH;
                s->state = LT_WAITING;
            }
        }
    } else if (s->state == LT_WAITING && lt_gate_crossing(&s->start, &s->prev, f, &fraction)) {
        s->state = LT_RUNNING;
        s->lap_start_ms = crossing_time(&s->prev, f, fraction);
        s->lap_invalid = 0;
        events |= LT_EVENT_START;
    }
    s->prev = *f;
    return events;
}

uint64_t lt_session_elapsed(const lt_session *s, uint64_t now_ms) {
    return s && s->state == LT_RUNNING && now_ms > s->lap_start_ms ? now_ms - s->lap_start_ms : 0;
}

int lt_gate_format(const lt_gate *gate, char *out, size_t len) {
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

int lt_gate_parse(const char *text, lt_gate *gate) {
    const char *p = text;
    lt_gate g;
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
