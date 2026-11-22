#include "lap_timer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.14159265358979323846
#define DEG_M (PI / 180.0 * 6371008.8)
static const double LAT0 = 35.49, LON0 = -119.81;

/* A fix at east/north metres from the origin, at t_ms. */
static lap_fix at(double east, double north, uint64_t t_ms) {
    lap_fix f;
    f.lat = LAT0 + north / DEG_M;
    f.lon = LON0 + east / (DEG_M * cos(LAT0 * PI / 180.0));
    f.t_ms = t_ms;
    return f;
}

static lap_gate gate_at(double east, double north, double heading, double half_width) {
    lap_fix p = at(east, north, 0);
    lap_gate g;
    g.lat = p.lat; g.lon = p.lon; g.heading_deg = heading; g.half_width_m = half_width;
    return g;
}

static void test_geometry(void) {
    lap_gate g;
    lap_fix a, b;
    double f = 0;
    assert(fabs(lap_distance_m(LAT0, LON0, LAT0 + .001, LON0) - 111.195) < .01);
    {   lap_fix e = at(300, 400, 0);
        assert(fabs(lap_distance_m(LAT0, LON0, e.lat, e.lon) - 500) < .05); }
    /* Heading of a marked line follows the direction of travel. */
    a = at(0, -100, 0); b = at(0, 0, 0);
    assert(lap_gate_from_motion(&g, a.lat, a.lon, b.lat, b.lon, 15));
    assert(fabs(g.heading_deg) < 1e-6 || fabs(g.heading_deg - 360) < 1e-6);
    a = at(-100, 0, 0);
    assert(lap_gate_from_motion(&g, a.lat, a.lon, b.lat, b.lon, 15) && fabs(g.heading_deg - 90) < 1e-6);
    a = at(0, .5, 0);
    assert(!lap_gate_from_motion(&g, a.lat, a.lon, b.lat, b.lon, 15));   /* under 1 m */
    assert(!lap_gate_from_motion(&g, 91, 0, b.lat, b.lon, 15));
    assert(!lap_gate_from_motion(&g, a.lat - .1, a.lon, b.lat, b.lon, 0));

    /* Interpolated crossing: 30 m before the line at 1.0 s, 20 m past it at 2.0 s -> 0.6 of the way. */
    g = gate_at(0, 0, 0, 15);
    a = at(0, -30, 1000); b = at(0, 20, 2000);
    assert(lap_gate_crossing(&g, &a, &b, &f) && fabs(f - .6) < 1e-6);
    assert(!lap_gate_crossing(&g, &b, &a, &f));                          /* backwards */
    a = at(20, -30, 1000); b = at(20, 20, 2000);
    assert(!lap_gate_crossing(&g, &a, &b, &f));                          /* beside the line */
    a = at(10, -30, 1000); b = at(10, 20, 2000);
    assert(lap_gate_crossing(&g, &a, &b, &f));
    a = at(-30, -30, 1000); b = at(30, 20, 2000);                       /* diagonal through the centre */
    assert(lap_gate_crossing(&g, &a, &b, &f) && fabs(f - .6) < 1e-6);
    a = at(0, 5, 1000); b = at(0, 20, 2000);
    assert(!lap_gate_crossing(&g, &a, &b, &f));                          /* already past */
}

/* Counter-clockwise circle of radius r around (0, r): the bottom point (0, 0) is crossed heading east. */
static lap_fix circle(double r, double speed, double start_deg, uint64_t t_ms) {
    double theta = (start_deg - 90) * PI / 180.0 + speed / r * (double)t_ms / 1000.0;
    return at(r * cos(theta), r + r * sin(theta), t_ms);
}

static void test_circuit(void) {
    const double r = 200, v = 30;
    const double lap = 2 * PI * r / v * 1000;          /* 41887.9 ms */
    lap_gate g = gate_at(0, 0, 90, 15);
    lap_session s;
    unsigned events, starts = 0, laps = 0;
    uint64_t t;
    lap_session_init(&s, &g, 0, 0);
    assert(s.circuit && s.state == LAP_WAITING && lap_session_elapsed(&s, 5000) == 0);
    for (t = 0; t <= 175000; t += 1000) {                /* 1 Hz, starting 10 degrees before the line */
        lap_fix f = circle(r, v, -10, t);
        events = lap_session_update(&s, &f);
        if (events & LAP_EVENT_START) {
            ++starts;
            assert(fabs((double)s.lap_start_ms - 10 * PI / 180 * r / v * 1000) < 20);
        }
        if (events & LAP_EVENT_LAP) {
            ++laps;
            assert(fabs((double)s.last_ms - lap) < 30);    /* chord interpolation at 1 Hz, 30 m/s */
            assert(s.last_valid && s.best_ms && s.best_ms <= s.last_ms);
        }
    }
    assert(starts == 1 && laps == 4 && s.laps == 4 && s.state == LAP_RUNNING);
    assert(lap_session_elapsed(&s, s.lap_start_ms + 1234) == 1234);
}

static void test_jitter_and_order(void) {
    lap_gate g = gate_at(0, 0, 0, 15);
    lap_session s;
    lap_fix f;
    lap_session_init(&s, &g, 0, 0);
    f = at(0, -5, 1000); lap_session_update(&s, &f);
    f = at(0, 2, 2000);  assert(lap_session_update(&s, &f) & LAP_EVENT_START);
    /* Standing on the line, the fix wanders back and forth: no lap before the minimum lap time. */
    f = at(0, -2, 3000); assert(lap_session_update(&s, &f) == 0);
    f = at(0, 3, 4000);  assert(lap_session_update(&s, &f) == 0);
    f = at(0, -1, 5000); lap_session_update(&s, &f);
    f = at(0, 1, 6000);  lap_session_update(&s, &f);
    assert(s.laps == 0 && s.state == LAP_RUNNING);
    f = at(0, 50, 5500); assert(lap_session_update(&s, &f) == 0);       /* out of order: ignored */
    assert(fabs(s.prev.lat - at(0, 1, 0).lat) < 1e-12);
}

static void test_dropout_and_jump(void) {
    const double r = 200, v = 30, lap = 2 * PI * r / v * 1000;
    lap_gate g = gate_at(0, 0, 90, 15);
    lap_session s;
    uint64_t t;
    unsigned completed = 0;
    int saw_invalid = 0;
    lap_session_init(&s, &g, 0, 0);
    for (t = 0; t <= 175000; t += 1000) {
        /* Lose the GPS for 5 s around the second crossing (about 43 s). */
        if (t > 41000 && t < 46000) continue;
        {   lap_fix f = circle(r, v, -10, t);
            unsigned events = lap_session_update(&s, &f);
            if (events & LAP_EVENT_INVALID) saw_invalid = 1;
            if (events & LAP_EVENT_LAP) {
                ++completed;
                if (completed == 1) {   /* the missed line makes this one two laps long */
                    assert(!s.last_valid && fabs((double)s.last_ms - 2 * lap) < 60 && s.best_ms == 0);
                } else {
                    assert(s.last_valid && fabs((double)s.last_ms - lap) < 30 && s.best_ms == s.last_ms);
                }
            } }
    }
    assert(saw_invalid && completed == 3);

    /* A fix 5 km away one second later is a jump, not a crossing. */
    g = gate_at(0, 0, 0, 15);
    lap_session_init(&s, &g, 0, 0);
    {   lap_fix a = at(0, -5, 1000), b = at(0, 5, 2000), c = at(0, 5000, 3000), d = at(0, 5010, 4000);
        lap_session_update(&s, &a);
        assert(lap_session_update(&s, &b) & LAP_EVENT_START);
        assert(lap_session_update(&s, &c) & LAP_EVENT_INVALID);
        assert(lap_session_update(&s, &d) == 0 && s.lap_invalid); }
}

/* 10 Hz at 30 m/s moves 3 m per fix; +-7 m of sideways noise must not read as a jump and hide the line. */
static void test_noisy_high_rate(void) {
    lap_gate g = gate_at(0, 0, 0, 15);
    lap_session s;
    uint64_t t;
    unsigned starts = 0;
    lap_session_init(&s, &g, 0, 0);
    for (t = 0; t <= 4000; t += 100) {
        lap_fix f = at((t / 100) % 2 ? 7 : -7, -60 + 30.0 * (double)t / 1000.0, t);
        unsigned events = lap_session_update(&s, &f);
        assert(!(events & LAP_EVENT_INVALID));
        if (events & LAP_EVENT_START) {
            ++starts;
            assert(llabs((long long)s.lap_start_ms - 2000) <= 1);
        }
    }
    assert(starts == 1 && !s.lap_invalid);
}

static void test_point_to_point(void) {
    lap_gate a = gate_at(0, 0, 0, 15), b = gate_at(0, 400, 0, 15);
    lap_session s;
    unsigned run, starts = 0, finishes = 0;
    lap_session_init(&s, &a, &b, 0);
    assert(!s.circuit);
    for (run = 0; run < 2; ++run) {
        /* 25 m/s from 50 m before A; fixes at 1 Hz, off phase with the gates. */
        uint64_t base = 100000 * run + 370, t;
        for (t = 0; t <= 22000; t += 1000) {
            lap_fix f = at(0, -50 + 25.0 * (double)t / 1000.0, base + t);
            unsigned events = lap_session_update(&s, &f);
            if (events & LAP_EVENT_START) ++starts;
            if (events & LAP_EVENT_FINISH) {
                ++finishes;
                assert(fabs((double)s.last_ms - 16000) <= 1 && s.state == LAP_WAITING);
            }
            assert(!(events & LAP_EVENT_LAP));
        }
        {   /* drive back to before A without timing anything */
            lap_fix back = at(30, -60, base + 60000);
            lap_session_update(&s, &back);
        }
    }
    assert(starts == 2 && finishes == 2 && s.laps == 2 && s.best_ms >= 15999 && s.best_ms <= 16001);
}

static void test_gate_text(void) {
    lap_gate g = gate_at(12.5, -7.25, 271.5, 18), h;
    char line[96];
    int n = lap_gate_format(&g, line, sizeof line);
    assert(n > 0 && line[n - 1] == '\n' && !strncmp(line, "LTG1 ", 5));
    assert(lap_gate_parse(line, &h));
    assert(fabs(h.lat - g.lat) < 1e-7 && fabs(h.lon - g.lon) < 1e-7);
    assert(fabs(h.heading_deg - 271.5) < 1e-9 && fabs(h.half_width_m - 18) < 1e-9);
    assert(lap_gate_parse("  LTG1 35.1 -119.2 0 10  \n", &h));
    assert(!lap_gate_parse("LTG1 35.1 -119.2 0", &h));
    assert(!lap_gate_parse("LTG1 91 0 0 10", &h));
    assert(!lap_gate_parse("LTG1 0 181 0 10", &h));
    assert(!lap_gate_parse("LTG1 0 0 360 10", &h));
    assert(!lap_gate_parse("LTG1 0 0 0 0", &h));
    assert(!lap_gate_parse("LTG1 0 0 0 201", &h));
    assert(!lap_gate_parse("LTG1 0 0 0 10 x", &h));
    assert(!lap_gate_parse("LTG1 nan 0 0 10", &h));
    assert(!lap_gate_parse("LTG2 0 0 0 10", &h));
    assert(!lap_gate_parse("LTG10 0 0 10", &h));
    assert(lap_gate_format(&g, line, 10) == -1);
}

int main(void) {
    test_geometry();
    test_circuit();
    test_jitter_and_order();
    test_dropout_and_jump();
    test_noisy_high_rate();
    test_point_to_point();
    test_gate_text();
    puts("Lap timer: geometry, interpolated crossings, circuit and A-B timing, jitter, dropouts and gate files passed.");
    return 0;
}
