#define CAR_HOST 1
#include "CarCore.h"
#include <stdlib.h>
#include <stdint.h>
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif
typedef struct {CarController car;CarConfig cfg;} Host;
API void *sim_create(void) {Host *h=(Host*)calloc(1,sizeof(Host));if(!h)return 0;car_init(&h->car);car_default_config(&h->cfg);return h;}
API void sim_destroy(void *ptr) {free(ptr);}
API void sim_reset(void *ptr,const int32_t *v) {
 Host *h=(Host*)ptr;car_init(&h->car);car_default_config(&h->cfg);
 /* These values apply only to the host simulator, never MCU commissioning. */
 h->cfg.calibrated=v[0]?1:0;h->cfg.finish_enabled=0;
 h->cfg.fan_arm_ticks=(car_u16)v[1];h->cfg.mag_min_sum=(car_u16)v[2];
 h->cfg.mag_sign=(car_i16)v[3];h->cfg.mag_zero_l=(car_u16)v[4];h->cfg.mag_zero_r=(car_u16)v[5];
 h->cfg.gate_timeout_ticks=(car_u16)v[6];h->cfg.gate_min_counts=(car_u32)v[7];h->cfg.clear_counts=(car_u32)v[8];
}
API void sim_tick(void *ptr,const int32_t *v,int32_t *r) {
 Host *h=(Host*)ptr;CarInput i={0};CarOutput o={0};
 i.adc_l=(car_u16)v[0];i.adc_r=(car_u16)v[1];i.wheel_l=(car_i16)v[2];i.wheel_r=(car_i16)v[3];i.gyro_z=(car_i16)v[4];
 i.visual_error=(car_i16)v[5];i.visual_element=(CarElement)v[6];i.frame_new=(car_u8)v[7];i.line_valid=(car_u8)v[8];i.gap_hint=(car_u8)v[9];
 i.arm=(car_u8)v[10];i.kill=(car_u8)v[11];i.finish_mark=(car_u8)v[12];car_step(&h->car,&h->cfg,&i,&o);
 r[0]=o.steer;r[1]=o.left_pwm;r[2]=o.right_pwm;r[3]=o.fan_command;r[4]=o.target_l;r[5]=o.target_r;r[6]=o.mag_error;r[7]=o.mag_valid;
 r[8]=o.enabled;r[9]=o.element;r[10]=o.gate;r[11]=o.fault;r[12]=o.finish_count;r[13]=o.finished;
}
