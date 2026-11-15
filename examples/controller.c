#include "gatecar_api.h"
#include <string.h>
/* Sensor-only example; replace with your own gate/vision state machine. */
static double last_error;
void gatecar_reset(void) { last_error = 0; }
void gatecar_step(const GateCarInput *in, GateCarOutput *out) {
    double error = in->visual_error_deg;
    double steer = 1.7*error + 0.006*(error-last_error)/in->dt_s;
    last_error = error;
    memset(out,0,sizeof(*out));
    out->enabled = in->line_valid && !in->kill;
    out->speed_m_s = out->enabled ? 0.7 : 0;
    out->steer_deg = steer < -30 ? -30 : steer > 30 ? 30 : steer;
    out->fan = 0.35;
    out->fault = in->kill ? 6 : in->line_valid ? 0 : 3;
    /* in->camera_gray[row*80+column] exposes the camera mask for real vision code. */
}
