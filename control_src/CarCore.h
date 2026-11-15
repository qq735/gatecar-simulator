/* Portable control core. No peripheral headers. Inputs updated every 5 ms. */
#ifndef CAR_CORE_H
#define CAR_CORE_H
#ifdef CAR_HOST
#include <stdint.h>
typedef int16_t car_i16;
typedef int32_t car_i32;
typedef uint16_t car_u16;
typedef uint32_t car_u32;
typedef uint8_t car_u8;
#else
#include "zf_common_typedef.h"
typedef int16 car_i16;
typedef int32 car_i32;
typedef uint16 car_u16;
typedef uint32 car_u32;
typedef uint8 car_u8;
#endif
#define CAR_TICK_MS 5
/* Profiles are independent controllers, not just one shared error history. */
typedef enum {
 CAR_STRAIGHT, CAR_BEND, CAR_CROSS,
 CAR_L_ENTRY, CAR_L_INSIDE, CAR_L_EXIT,
 CAR_R_ENTRY, CAR_R_INSIDE, CAR_R_EXIT,
 CAR_GATE_APPROACH, CAR_GATE_GUIDE, CAR_GATE_REACQUIRE,
 CAR_PROFILE_COUNT
} CarElement;
typedef enum {GATE_IDLE, GATE_APPROACH, GATE_GUIDE, GATE_REACQUIRE, GATE_CLEAR, GATE_FAULT} GateState;
typedef enum {FAULT_NONE, FAULT_NOT_READY, FAULT_CAMERA, FAULT_LINE, FAULT_MAGNETIC, FAULT_GATE_LIMIT, FAULT_DISARMED} CarFault;
typedef struct {
 car_i16 kp,ki,kd,gyro_gain; /* gains /100; integral is error*ticks */
 car_i16 speed,lookahead,limit,slew;
 car_i16 wheel_kp,wheel_ki,wheel_kd;
 car_u16 fan; /* ESC command 0..1000, not measured RPM */
 car_u8 differential;
} CarProfile;
typedef struct {
 car_i32 integral;
 car_i16 previous;
 car_u8 primed;
} CarPid;
typedef struct {
 car_u16 mag_zero_l,mag_zero_r,mag_gain_l,mag_gain_r; /* gain /1000 */
 car_u16 mag_min_sum,mag_saturation;
 car_i16 mag_sign;
 car_u16 image_timeout_ticks,line_grace_ticks,mag_grace_ticks;
 car_u16 prepare_ticks,gate_timeout_ticks;
 car_u32 gate_max_counts,gate_min_counts,clear_counts;
 car_u16 confirm_ticks,exit_frames,clear_ticks;
 car_i16 steer_mid;
 car_u16 fan_slew,fan_min,fan_max,fan_idle,fan_arm_ticks;
 car_u16 finish_confirm_frames,finish_depart_frames,finish_target;
 car_u8 finish_enabled;
 car_u8 calibrated;
} CarConfig;
typedef struct {
 car_u16 adc_l,adc_r;
 car_i16 wheel_l,wheel_r,gyro_z;
 car_i16 visual_error; /* signed pursuit angle in degrees, -30..30 */
 CarElement visual_element;
 car_u8 frame_new,line_valid,gap_hint,arm,kill,finish_mark;
} CarInput;
typedef struct {
 car_i16 steer,left_pwm,right_pwm;
 car_u16 fan_command,finish_count;
 car_u8 finished;
 car_i16 target_l,target_r,mag_error;
 car_u8 mag_valid,enabled;
 CarElement element;
 GateState gate;
 CarFault fault;
} CarOutput;
typedef struct {
 CarPid steering[CAR_PROFILE_COUNT],wheel_l,wheel_r;
 CarElement active;
 GateState gate;
 CarFault fault;
 car_u8 armed,filter_ready,saw_gap,last_line;
 car_i32 filtered_l,filtered_r;
 car_u16 image_age,line_bad,mag_bad,confirm,exit_good,gate_ticks,clear_age;
 car_u32 gate_counts,clear_distance;
 car_i16 last_steer,last_speed,entry_visual;
 car_u16 fan_command,arm_age,finish_good,finish_clear,finish_count;
 car_u8 finish_latched,finished;
} CarController;
extern const CarProfile car_profiles[CAR_PROFILE_COUNT];
void car_default_config(CarConfig *c);
void car_init(CarController *c);
void car_step(CarController *c,const CarConfig *cfg,const CarInput *in,CarOutput *out);
#endif
