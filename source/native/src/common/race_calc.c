#include "race_calc.h"
#include <math.h>
#include <string.h>

/* ---- gear ---------------------------------------------------------------------------------------- */

#define RC_K_MIN 8.0          /* RPM per km/h at a 1.00 ratio: covers small wheels and short final drives... */
#define RC_K_MAX 80.0         /* ...to large wheels and long ones */
#define RC_GEAR_MIN_KPH 8     /* below this the integer km/h is too coarse to name a gear */
#define RC_LEARN_MIN_KPH 30   /* 1 km/h of integer resolution is under 2 % from here */
#define RC_STEADY 0.02        /* RPM per km/h within 2 % of the segment mean (plus the km/h resolution):
                                 same gear, no slip */
#define RC_SEGMENT_VOTE 6     /* steady samples before a segment votes, once */
#define RC_LEARN_VOTES 6      /* votes for the winning factor before it is trusted... */
#define RC_LEARN_MARGIN 1.5   /* ...and how far it must lead any other candidate */

static double bin_factor(int bin) {
    return RC_K_MIN * exp(log(RC_K_MAX / RC_K_MIN) * ((double)bin + .5) / RC_K_BINS);
}

static int factor_bin(double k) {
    if (!(k >= RC_K_MIN && k < RC_K_MAX)) return -1;
    return (int)(log(k / RC_K_MIN) / log(RC_K_MAX / RC_K_MIN) * RC_K_BINS);
}

void rc_gear_config_zf8(rc_gear_config *cfg, double k_prior) {
    static const double zf8[] = {4.714, 3.143, 2.106, 1.667, 1.285, 1.000, 0.839, 0.667};
    unsigned i;
    memset(cfg, 0, sizeof *cfg);
    cfg->gears = sizeof zf8 / sizeof zf8[0];
    for (i = 0; i < cfg->gears; ++i) cfg->ratio[i] = zf8[i];
    cfg->k_prior = k_prior;
    cfg->tolerance = .05;
}

void rc_gear_init(rc_gear *g, const rc_gear_config *cfg) {
    memset(g, 0, sizeof *g);
    g->cfg = *cfg;
    if (g->cfg.gears > RC_MAX_GEARS) g->cfg.gears = RC_MAX_GEARS;
}

double rc_gear_factor(const rc_gear *g) { return g->k > 0 ? g->k : g->cfg.k_prior; }

static unsigned window(const rc_gear *g, int bin) {
    unsigned sum = g->votes[bin];
    if (bin > 0) sum += g->votes[bin - 1];
    if (bin + 1 < RC_K_BINS) sum += g->votes[bin + 1];
    return sum;
}

/* Every gear hypothesis votes for the factor it implies. The true factor collects a vote from every gear
 * that was driven in, while a wrong one collects votes only where two gearbox steps happen to match, so a
 * single segment (or one gear only) never decides. */
static void vote(rc_gear *g, double ratio) {
    unsigned i, best = 0, second = 0;
    int b, best_bin = -1;
    for (i = 0; i < g->cfg.gears; ++i) {
        b = factor_bin(ratio / g->cfg.ratio[i]);
        if (b >= 0) ++g->votes[b];
    }
    for (b = 0; b < RC_K_BINS; ++b) {
        unsigned w = window(g, b);
        if (w > best) { best = w; best_bin = b; }
    }
    for (b = 0; b < RC_K_BINS; ++b) {
        unsigned w = window(g, b);
        if ((b < best_bin - 6 || b > best_bin + 6) && w > second) second = w;
    }
    if (best_bin >= 0 && best >= RC_LEARN_VOTES && best >= RC_LEARN_MARGIN * second) {
        double sum = 0, weight = 0;
        for (b = best_bin - 1; b <= best_bin + 1; ++b) {
            if (b < 0 || b >= RC_K_BINS) continue;
            sum += bin_factor(b) * g->votes[b];
            weight += g->votes[b];
        }
        g->k = sum / weight;
    }
}

static int nearest_gear(const rc_gear *g, double ratio, double k, int kph) {
    unsigned i;
    int gear = 0;
    double best = g->cfg.tolerance + .5 / kph;   /* plus half a km/h of resolution */
    for (i = 0; i < g->cfg.gears; ++i) {
        double error = fabs(ratio / (k * g->cfg.ratio[i]) - 1);
        if (error <= best) { best = error; gear = (int)i + 1; }
    }
    return gear;
}

int rc_gear_update(rc_gear *g, int rpm, int kph) {
    double ratio, k;
    int candidate = 0;
    if (rpm < 0 || kph < 0) return g->gear;
    ratio = kph > 0 ? (double)rpm / kph : 0;
    /* Learning: one vote per steady segment, i.e. per gear engagement. */
    if (kph >= RC_LEARN_MIN_KPH && rpm >= 800) {
        if (g->steady && fabs(ratio / g->last_ratio - 1) <= RC_STEADY + 1.0 / kph) {
            g->last_ratio += (ratio - g->last_ratio) / (double)(++g->steady);
            if (g->steady == RC_SEGMENT_VOTE) { ++g->samples; vote(g, g->last_ratio); }
        } else {
            g->last_ratio = ratio;
            g->steady = 1;
        }
    } else g->steady = 0;
    k = rc_gear_factor(g);
    if (k > 0 && kph >= RC_GEAR_MIN_KPH && rpm >= 500) candidate = nearest_gear(g, ratio, k, kph);
    /* Two samples in a row before the display changes: no flicker through a shift. */
    if (candidate == g->candidate) ++g->candidate_count;
    else { g->candidate = candidate; g->candidate_count = 1; }
    if (g->candidate_count >= 2) g->gear = candidate;
    return g->gear;
}

/* ---- longitudinal acceleration ------------------------------------------------------------------- */

void rc_accel_init(rc_accel *a) {
    memset(a, 0, sizeof *a);
    /* 1 s: with integer km/h a shorter slope is noisy enough to inflate a peak by 10 % or more. */
    a->window_ms = 1000;
}

void rc_accel_add(rc_accel *a, uint64_t t_ms, int kph) {
    if (kph < 0) return;
    if (a->count && t_ms <= a->t_ms[(a->next + RC_ACCEL_SAMPLES - 1) % RC_ACCEL_SAMPLES]) return;
    a->t_ms[a->next] = t_ms;
    a->mps[a->next] = kph / 3.6;
    a->next = (a->next + 1) % RC_ACCEL_SAMPLES;
    if (a->count < RC_ACCEL_SAMPLES) ++a->count;
}

int rc_accel_g(const rc_accel *a, double *g) {
    unsigned i, n = 0;
    uint64_t newest, oldest = 0;
    double st = 0, sv = 0, stt = 0, stv = 0, d;
    if (!a->count) return 0;
    newest = a->t_ms[(a->next + RC_ACCEL_SAMPLES - 1) % RC_ACCEL_SAMPLES];
    for (i = 0; i < a->count; ++i) {
        unsigned k = (a->next + RC_ACCEL_SAMPLES - 1 - i) % RC_ACCEL_SAMPLES;
        double t;
        if (newest - a->t_ms[k] > a->window_ms) break;
        t = (double)(newest - a->t_ms[k]) / 1000.0;
        st += t; sv += a->mps[k]; stt += t * t; stv += t * a->mps[k];
        oldest = a->t_ms[k];
        ++n;
    }
    if (n < 4 || newest - oldest < 500) return 0;
    d = n * stt - st * st;
    if (d <= 0) return 0;
    /* t counts backwards from the newest sample, so the slope's sign flips. */
    *g = -(n * stv - st * sv) / d / 9.80665;
    return 1;
}

/* ---- standing-start run -------------------------------------------------------------------------- */

#define RC_READY_MS 300        /* stopped this long arms a run */
#define RC_LIFT_KPH 4          /* falling this far below the run's peak ends it */
#define RC_TIMEOUT_MS 60000
#define RC_QUARTER_M 402.336

void rc_run_init(rc_run *r) {
    memset(r, 0, sizeof *r);
    r->state = RC_IDLE;
    r->last_kph = -1;
    rc_accel_init(&r->accel);
}

static void launch(rc_run *r, uint64_t now_ms, int kph) {
    rc_accel saved = r->accel;
    uint64_t zero = r->last_zero_ms;
    memset(r, 0, sizeof *r);
    r->accel = saved;
    r->last_zero_ms = zero;
    r->state = RC_RUNNING;
    /* Until a second speed step allows extrapolation, assume the launch half way since standstill. */
    r->start_ms = zero + (now_ms - zero) / 2;
    r->first_ms = r->change_ms = now_ms;
    r->first_kph = r->change_kph = r->peak_kph = kph;
    r->dist_m = .5 * (kph / 3.6) * (double)(now_ms - r->start_ms) / 1000.0;
}

/* Extrapolate the first two speed steps back to 0 km/h, no earlier than the last standstill sample,
 * as launch_timer does; the distance covered before the first step follows the new start. */
static void refine_start(rc_run *r, uint64_t now_ms, int kph) {
    uint64_t span = now_ms - r->first_ms, back, start;
    double before, after;
    back = (uint64_t)r->first_kph * span / (uint64_t)(kph - r->first_kph);
    start = back > r->first_ms ? 0 : r->first_ms - back;
    if (start < r->last_zero_ms) start = r->last_zero_ms;
    before = .5 * (r->first_kph / 3.6) * (double)(r->first_ms - r->start_ms) / 1000.0;
    after = .5 * (r->first_kph / 3.6) * (double)(r->first_ms - start) / 1000.0;
    r->dist_m += after - before;
    r->start_ms = start;
    r->start_refined = 1;
}

static void mark(rc_run *r, double target, double from_m, double to_m, uint64_t to_ms, int to_kph,
                 uint32_t *t_out, double *v_out) {
    double f;
    if (*t_out || !(from_m < target && target <= to_m)) return;
    f = (target - from_m) / (to_m - from_m);
    *t_out = (uint32_t)((double)(r->last_ms - r->start_ms) + f * (double)(to_ms - r->last_ms) + .5);
    *v_out = r->last_kph + f * (to_kph - r->last_kph);
}

static unsigned stopped(rc_run *r, uint64_t now_ms) {
    r->last_zero_ms = now_ms;
    if (!r->zero_since_ms) r->zero_since_ms = now_ms;
    if (r->state != RC_READY && now_ms - r->zero_since_ms >= RC_READY_MS) {
        r->state = RC_READY;
        return RC_EV_READY;
    }
    return 0;
}

unsigned rc_run_update(rc_run *r, uint64_t now_ms, int kph) {
    unsigned events = 0;
    if (kph < 0 || (r->last_kph >= 0 && now_ms <= r->last_ms)) return 0;
    rc_accel_add(&r->accel, now_ms, kph);
    if (r->state == RC_RUNNING) {
        /* Trapezoid between the previous sample and this one. */
        double step = .5 * (r->last_kph + kph) / 3.6 * (double)(now_ms - r->last_ms) / 1000.0, from, g;
        if (!r->start_refined && kph > r->first_kph) refine_start(r, now_ms, kph);
        from = r->dist_m;
        r->dist_m += step;
        if (rc_accel_g(&r->accel, &g) && g > r->peak_g) r->peak_g = g;
        if (kph > r->peak_kph) r->peak_kph = kph;
        if (!r->t100_ms) {
            mark(r, 100, from, r->dist_m, now_ms, kph, &r->t100_ms, &r->v100_kph);
            if (r->t100_ms) events |= RC_EV_100M;
        }
        mark(r, RC_QUARTER_M, from, r->dist_m, now_ms, kph, &r->t402_ms, &r->v402_kph);
        if (r->t402_ms) {
            events |= RC_EV_402M | RC_EV_DONE;
            r->completed = 1;
            r->state = RC_DONE;
        } else if (!kph || kph <= r->peak_kph - RC_LIFT_KPH || now_ms - r->start_ms > RC_TIMEOUT_MS) {
            events |= RC_EV_DONE;     /* lifted, stopped or too slow: the marks reached so far stand */
            r->state = RC_DONE;
        }
        if (r->state == RC_DONE && kph == 0) events |= stopped(r, now_ms);
    } else if (kph == 0) {
        events |= stopped(r, now_ms);
    } else {
        r->zero_since_ms = 0;
        if (r->state == RC_READY) {
            launch(r, now_ms, kph);
            events |= RC_EV_START;
        }
    }
    r->last_kph = kph;
    r->last_ms = now_ms;
    return events;
}
