#include "CarCore.h"
/* INITIAL BENCH VALUES ONLY. Speed is encoder counts per 5 ms, NOT cm/s.
   Each row owns a separate steering PID state. ki=0 starts as PD. */
const CarProfile car_profiles[CAR_PROFILE_COUNT] = {
/* kp ki kd gyro speed ahead limit slew   wheelP I D fan diff */
 {180,0,35, 1, 80,26,300,12, 1000,20,0,300,1}, /* straight */
 {220,0,50, 1, 55,18,340,12, 1000,20,0,650,1}, /* bend */
 {200,0,40, 1, 40,18,300,10, 1000,20,0,450,1}, /* cross */
 {210,0,45, 1, 40,16,330,10, 1000,20,0,700,1}, /* left entry */
 {230,0,50, 1, 35,14,340,10, 1000,20,0,800,1}, /* left inside */
 {200,0,40, 1, 40,22,320,10, 1000,20,0,650,1}, /* left exit */
 {210,0,45, 1, 40,16,330,10, 1000,20,0,700,1}, /* right entry */
 {230,0,50, 1, 35,14,340,10, 1000,20,0,800,1}, /* right inside */
 {200,0,40, 1, 40,22,320,10, 1000,20,0,650,1}, /* right exit */
 {160,0,25, 1, 30,16,260,8,  1000,20,0,500,1}, /* approach: visual */
 {180,0,30, 1, 25,14,260,8,  1000,20,0,550,1}, /* guide: magnetic mapped to degrees */
 {160,0,25, 1, 25,16,260,8,  1000,20,0,500,1}  /* reacquire: blended */
};
void car_default_config(CarConfig *c) {
 c->mag_zero_l=0;c->mag_zero_r=0;c->mag_gain_l=1000;c->mag_gain_r=1000;
 c->mag_min_sum=160;c->mag_saturation=4000;c->mag_sign=1;
 c->image_timeout_ticks=40;c->line_grace_ticks=6;c->mag_grace_ticks=10;
 c->prepare_ticks=20;c->gate_timeout_ticks=800;
 c->gate_max_counts=20000;c->gate_min_counts=500;c->clear_counts=1000;
 c->confirm_ticks=4;c->exit_frames=6;c->clear_ticks=60;c->steer_mid=2975;
 c->fan_slew=5;c->fan_min=500;c->fan_max=1000;c->fan_idle=500;c->fan_arm_ticks=400;
 c->finish_enabled=1;c->finish_confirm_frames=4;c->finish_depart_frames=8;
 c->finish_target=2; /* Start stripe + one completed lap; adjust to actual event. */
 c->calibrated=0; /* Explicit commissioning required before arming. */
}
