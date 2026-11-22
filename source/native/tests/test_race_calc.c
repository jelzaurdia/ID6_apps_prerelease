#include "race_calc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* RPM per km/h at a 1.00 ratio for a ZF 8HP car: 8th gear at 100 km/h is about 1790 RPM. */
#define K_TRUE 26.8
static const double ZF8[] = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};

static int rpm_for(int gear, double kph) { return (int)(K_TRUE * ZF8[gear - 1] * kph + .5); }
static int kph_of(double mps) { return (int)(mps * 3.6 + .5); }   /* the DME's integer km/h */

static void test_gear_with_prior(void) {
    rc_gear_config cfg;
    rc_gear g;
    int gear;
    rc_gear_config_zf8(&cfg, K_TRUE);
    rc_gear_init(&g, &cfg);
    assert(cfg.gears == 8 && fabs(rc_gear_factor(&g) - K_TRUE) < 1e-9);
    for (gear = 1; gear <= 8; ++gear) {
        /* A speed where this gear turns about 3000 RPM. */
        int kph = (int)(3000 / (K_TRUE * ZF8[gear - 1]) + .5), rpm = rpm_for(gear, kph);
        rc_gear_update(&g, rpm, kph);
        assert(rc_gear_update(&g, rpm, kph) == gear);
    }
    /* A single odd sample (mid-shift) does not change the display; two in a row do. */
    assert(rc_gear_update(&g, rpm_for(8, 100), 100) == 8);
    assert(rc_gear_update(&g, rpm_for(8, 100), 100) == 8);
    assert(rc_gear_update(&g, (int)(rpm_for(8, 100) * 1.15), 100) == 8);
    assert(rc_gear_update(&g, rpm_for(8, 100), 100) == 8);
    assert(rc_gear_update(&g, (int)(rpm_for(8, 100) * 1.15), 100) == 8);
    assert(rc_gear_update(&g, (int)(rpm_for(8, 100) * 1.15), 100) == 0);  /* converter slip */
    assert(rc_gear_update(&g, 800, 0) == 0 && rc_gear_update(&g, 800, 0) == 0);  /* standstill */
    assert(rc_gear_update(&g, -1, 50) == 0);                               /* unavailable: keep */
}

/* Accelerate through gears 1..last, 1.5 s per gear at 15 Hz, from 30 km/h up. */
static void pull(rc_gear *g, int last, int check) {
    int gear;
    double kph = 30;
    for (gear = 1; gear <= last; ++gear) {
        int i;
        for (i = 0; i < 22; ++i) {
            int shown, rpm = rpm_for(gear, kph);
            shown = rc_gear_update(g, rpm, (int)(kph + .5));
            if (check && i >= 2) assert(shown == gear);
            kph += (gear < 3 ? 1.2 : .9);
        }
    }
}

static void test_gear_learning(void) {
    rc_gear_config cfg;
    rc_gear g;
    unsigned i;
    rc_gear_config_zf8(&cfg, 0);
    rc_gear_init(&g, &cfg);
    assert(rc_gear_factor(&g) == 0 && rc_gear_update(&g, 2000, 80) == 0);
    /* Cruising in one gear, however long, never decides the factor. */
    for (i = 0; i < 400; ++i) {
        int kph = 60 + (int)(i % 50);
        if (i % 50 == 49) rc_gear_update(&g, 900, 10);                  /* a stop ends the segment */
        else rc_gear_update(&g, rpm_for(8, kph), kph);
    }
    assert(rc_gear_factor(&g) == 0);
    /* Two pulls through first to fifth do. */
    pull(&g, 5, 0);
    pull(&g, 5, 0);
    assert(fabs(rc_gear_factor(&g) / K_TRUE - 1) < .01);
    /* Then every gear reads correctly. */
    pull(&g, 5, 1);
}

static void test_accel(void) {
    rc_accel a;
    double g = 0, v = 0;
    uint64_t t;
    rc_accel_init(&a);
    assert(!rc_accel_g(&a, &g));
    for (t = 0; t <= 3000; t += 65) {               /* 0.5 g from rest, 15 Hz integer km/h */
        rc_accel_add(&a, t, kph_of(v));
        v += .5 * 9.80665 * .065;
    }
    assert(rc_accel_g(&a, &g) && fabs(g - .5) < .03);
    rc_accel_init(&a);
    for (t = 0; t <= 2000; t += 65) rc_accel_add(&a, t, 100);
    assert(rc_accel_g(&a, &g) && fabs(g) < .01);
    rc_accel_init(&a);
    rc_accel_add(&a, 0, 50); rc_accel_add(&a, 65, 51); rc_accel_add(&a, 130, 52);
    assert(!rc_accel_g(&a, &g));                    /* too few samples */
    rc_accel_add(&a, 100, 99);                      /* out of order: ignored */
    rc_accel_add(&a, 195, 53);
    assert(!rc_accel_g(&a, &g));                    /* four samples but under 500 ms */
    rc_accel_add(&a, 260, 54); rc_accel_add(&a, 325, 55); rc_accel_add(&a, 390, 56);
    rc_accel_add(&a, 455, 57); rc_accel_add(&a, 520, 58);
    assert(rc_accel_g(&a, &g) && fabs(g - 1 / 3.6 / .065 / 9.80665) < .01);   /* 1 km/h per sample */
}

/* Standing start at a constant acceleration; 15 Hz integer km/h. */
static void drive(rc_run *r, double accel, double stop_after_s, unsigned *events_seen, uint64_t *t) {
    double v = 0, s;
    uint64_t start = *t;
    for (;; *t += 65) {
        s = (double)(*t - start) / 1000.0;
        v = s < stop_after_s ? accel * s : v - 6 * .065;
        if (v < 0) v = 0;
        *events_seen |= rc_run_update(r, *t, kph_of(v));
        if (s > stop_after_s && v == 0) break;
    }
}

static void test_run(void) {
    rc_run r;
    unsigned seen = 0;
    uint64_t t;
    const double a = 4.5;                                          /* about 0.46 g */
    const double t100 = sqrt(200 / a) * 1000, t402 = sqrt(2 * 402.336 / a) * 1000;
    rc_run_init(&r);
    assert(r.state == RC_IDLE);
    /* Rolling without ever stopping does not arm. */
    assert(rc_run_update(&r, 0, 20) == 0 && rc_run_update(&r, 65, 25) == 0 && r.state == RC_IDLE);
    /* Stopped for 300 ms arms it. */
    for (t = 1000; t < 1300; t += 65) assert(!(rc_run_update(&r, t, 0) & RC_EV_READY));
    assert(rc_run_update(&r, 1300, 0) & RC_EV_READY);
    assert(r.state == RC_READY);
    t = 2000;
    drive(&r, a, 14, &seen, &t);                                   /* past 402 m, then brake */
    assert((seen & (RC_EV_START | RC_EV_100M | RC_EV_402M | RC_EV_DONE)) ==
           (RC_EV_START | RC_EV_100M | RC_EV_402M | RC_EV_DONE));
    assert(r.completed && r.state != RC_RUNNING);
    assert(fabs((double)r.t100_ms - t100) < 20);   /* interpolated well inside the 65 ms sample gap */
    assert(fabs((double)r.t402_ms - t402) < 20);
    assert(fabs(r.v402_kph - a * t402 / 1000 * 3.6) < 1);
    assert(fabs(r.peak_g - a / 9.80665) < .03);
    assert(r.peak_kph >= 216);

    /* Stopped again: re-armed; a lift before 402 m keeps the 100 m mark only. */
    seen = 0;
    {   uint64_t end = t + 400;
        for (t += 65; t <= end; t += 65) seen |= rc_run_update(&r, t, 0); }
    assert(seen & RC_EV_READY);
    seen = 0;
    drive(&r, a, 7.5, &seen, &t);                                  /* about 127 m, then lift */
    assert((seen & RC_EV_START) && (seen & RC_EV_100M) && (seen & RC_EV_DONE) && !(seen & RC_EV_402M));
    assert(!r.completed && r.t402_ms == 0 && fabs((double)r.t100_ms - t100) < 20);
}

int main(void) {
    test_gear_with_prior();
    test_gear_learning();
    test_accel();
    test_run();
    puts("Race calc: gear from RPM per km/h (prior and learned), longitudinal g and standing-start runs passed.");
    return 0;
}
