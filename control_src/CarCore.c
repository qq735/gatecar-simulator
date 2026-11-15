#include "CarCore.h"
#include <string.h>
static car_i32 clamp32(car_i32 x,car_i32 lo,car_i32 hi) {return x<lo?lo:(x>hi?hi:x);}
static car_i32 abs32(car_i32 x) {return x<0?-x:x;}
static car_u16 inc16(car_u16 x) {return x<65535?x+1:x;}
static void pid_reset(CarPid *p) {p->integral=0;p->previous=0;p->primed=0;}
void car_init(CarController *c) {memset(c,0,sizeof(*c));c->active=CAR_STRAIGHT;c->gate=GATE_IDLE;}
static car_i16 steer_pid(CarPid *p,const CarProfile *g,car_i16 e,car_i16 gyro) {
 car_i32 d,u,next_i;
 d=p->primed?(car_i32)e-p->previous:0;
 next_i=clamp32(p->integral+e,-10000,10000);
 u=((car_i32)g->kp*e+(car_i32)g->ki*next_i+(car_i32)g->kd*d+(car_i32)g->gyro_gain*gyro)/100;
 if (u<=g->limit && u>=-g->limit) p->integral=next_i;
 p->previous=e;p->primed=1;
 return (car_i16)clamp32(u,-g->limit,g->limit);
}
static car_i16 wheel_pid(CarPid *p,const CarProfile *g,car_i16 target,car_i16 actual) {
 car_i32 e,d,ni,u;
 e=(car_i32)target-actual;d=p->primed?e-p->previous:0;
 ni=clamp32(p->integral+e,-20000,20000);
 u=((car_i32)g->wheel_kp*e+(car_i32)g->wheel_ki*ni+(car_i32)g->wheel_kd*d)/100;
 if (u>=-10000 && u<=10000) p->integral=ni;
 p->previous=(car_i16)clamp32(e,-32767,32767);p->primed=1;
 return (car_i16)clamp32(u,-10000,10000);
}
static void magnet(CarController *c,const CarConfig *cfg,const CarInput *in,CarOutput *o) {
 car_i32 l,r,sum;
 l=clamp32((car_i32)in->adc_l-cfg->mag_zero_l,0,4095)*cfg->mag_gain_l/1000;
 r=clamp32((car_i32)in->adc_r-cfg->mag_zero_r,0,4095)*cfg->mag_gain_r/1000;
 if (!c->filter_ready) {c->filtered_l=l;c->filtered_r=r;c->filter_ready=1;}
 else {c->filtered_l+=(l-c->filtered_l)/4;c->filtered_r+=(r-c->filtered_r)/4;}
 sum=c->filtered_l+c->filtered_r;
 /* Both instantaneous and filtered levels must be usable. */
 o->mag_valid=(l+r>=cfg->mag_min_sum && sum>=cfg->mag_min_sum && sum>0 &&
  in->adc_l<cfg->mag_saturation && in->adc_r<cfg->mag_saturation);
 o->mag_error=o->mag_valid?(car_i16)clamp32((c->filtered_l-c->filtered_r)*1000/sum*cfg->mag_sign,-1000,1000):0;
}
static void fail(CarController *c,CarFault f) {c->fault=f;c->gate=GATE_FAULT;c->armed=0;}
void car_step(CarController *c,const CarConfig *cfg,const CarInput *in,CarOutput *o) {
 const CarProfile *g;
 CarElement element;
 car_i16 error,desired,delta,speed;
 car_i32 distance,blend;
 memset(o,0,sizeof(*o));o->steer=cfg->steer_mid;
 magnet(c,cfg,in,o);
 if (in->frame_new) {c->image_age=0;c->last_line=in->line_valid;} else c->image_age=inc16(c->image_age);
 if (in->kill) fail(c,FAULT_DISARMED);
 if (in->arm && !c->armed && c->fault==FAULT_NONE) {
  if (!cfg->calibrated || c->finished || cfg->fan_min>cfg->fan_max || cfg->fan_max>10000 || cfg->fan_slew==0 || (cfg->finish_enabled && (cfg->finish_target==0 || cfg->finish_confirm_frames==0 || cfg->finish_depart_frames==0)) || cfg->exit_frames==0 || cfg->confirm_ticks==0 || cfg->mag_gain_l==0 || cfg->mag_gain_r==0 || cfg->mag_gain_l>4000 || cfg->mag_gain_r>4000 || (cfg->mag_sign!=1 && cfg->mag_sign!=-1) || !c->last_line || c->image_age>=cfg->image_timeout_ticks) fail(c,FAULT_NOT_READY);
  else c->armed=1;
 }
 if (!c->armed) goto finished;
 c->arm_age=inc16(c->arm_age);
 /* Count a stripe once until clear for consecutive NEW camera frames. */
 if (cfg->finish_enabled && in->frame_new && c->gate==GATE_IDLE) {
  if (in->finish_mark) {
   c->finish_clear=0;c->finish_good=inc16(c->finish_good);
   if (!c->finish_latched && c->finish_good>=cfg->finish_confirm_frames) {
    c->finish_latched=1;c->finish_count=inc16(c->finish_count);
    if (c->finish_count>=cfg->finish_target) {c->finished=1;c->armed=0;goto finished;}
   }
  } else {
   c->finish_good=0;c->finish_clear=inc16(c->finish_clear);
   if(c->finish_clear>=cfg->finish_depart_frames)c->finish_latched=0;
  }
 }

 if (c->image_age>=cfg->image_timeout_ticks) {fail(c,FAULT_CAMERA);goto finished;}
 c->line_bad=c->last_line?0:inc16(c->line_bad);
 c->mag_bad=o->mag_valid?0:inc16(c->mag_bad);
 distance=(abs32(in->wheel_l)+abs32(in->wheel_r))/2;
 element=in->visual_element;
 if (element< CAR_STRAIGHT || element> CAR_R_EXIT) element=CAR_STRAIGHT;
 error=in->visual_error;
 if (c->gate==GATE_IDLE) {
  /* Confirm on distinct camera frames, never repeatedly count one frame. */
  if (in->frame_new) {
   if (in->gap_hint && o->mag_valid) c->confirm=inc16(c->confirm);else c->confirm=0;
  }
  if (c->confirm>=cfg->confirm_ticks) {
   c->gate=GATE_APPROACH;c->gate_ticks=0;c->gate_counts=0;c->saw_gap=0;
   c->exit_good=0;c->entry_visual=in->visual_error;
  }
  else if (c->line_bad>=cfg->line_grace_ticks) {fail(c,FAULT_LINE);goto finished;}
 }
 if (c->gate==GATE_APPROACH || c->gate==GATE_GUIDE || c->gate==GATE_REACQUIRE) {
  c->gate_ticks=inc16(c->gate_ticks);c->gate_counts+=(car_u32)distance;
  if (c->gate_ticks>=cfg->gate_timeout_ticks || c->gate_counts>=cfg->gate_max_counts) {
   fail(c,FAULT_GATE_LIMIT);goto finished;
  }
  if (c->mag_bad>=cfg->mag_grace_ticks && c->gate!=GATE_REACQUIRE) {fail(c,FAULT_MAGNETIC);goto finished;}
 }
 if (c->gate==GATE_APPROACH) {
  element=CAR_GATE_APPROACH;
  if (!c->last_line) {c->saw_gap=1;c->gate=GATE_GUIDE;}
  else if (c->gate_ticks>=cfg->prepare_ticks) c->gate=GATE_GUIDE;
 }
 if (c->gate==GATE_GUIDE) {
  element=CAR_GATE_GUIDE;
  if (!c->last_line) c->saw_gap=1;
  error=o->mag_valid?(car_i16)((car_i32)o->mag_error*30/1000):0;
  if (in->frame_new) {
   if (c->saw_gap && in->line_valid && c->gate_counts>=cfg->gate_min_counts) c->exit_good=inc16(c->exit_good);
   else c->exit_good=0;
  }
  if (c->exit_good>=cfg->exit_frames) {c->gate=GATE_REACQUIRE;c->exit_good=0;}
 }
 if (c->gate==GATE_REACQUIRE) {
  element=CAR_GATE_REACQUIRE;
  if (!c->last_line) {c->gate=GATE_GUIDE;c->exit_good=0;error=o->mag_valid?(car_i16)((car_i32)o->mag_error*30/1000):0;}
  else {
   if (in->frame_new) c->exit_good=inc16(c->exit_good);
   blend=clamp32((car_i32)c->exit_good*1000/cfg->exit_frames,0,1000);
   if (!o->mag_valid) blend=1000;
   error=(car_i16)(((car_i32)o->mag_error*30/1000*(1000-blend)+(car_i32)in->visual_error*blend)/1000);
   if (c->exit_good>=cfg->exit_frames) {c->gate=GATE_CLEAR;c->clear_age=0;c->clear_distance=0;}
  }
 }
 if (c->gate==GATE_CLEAR) {
  c->clear_age=inc16(c->clear_age);c->clear_distance+=(car_u32)distance;
  if (c->line_bad>=cfg->line_grace_ticks) {fail(c,FAULT_LINE);goto finished;}
  /* Stay at reacquire speed until the old gate has been left behind. */
  element=CAR_GATE_REACQUIRE;error=in->visual_error;
  if (c->clear_age>=cfg->clear_ticks && c->clear_distance>=cfg->clear_counts) {
   c->gate=GATE_IDLE;c->confirm=0;
  }
 }
 if (c->active!=element) {
  pid_reset(&c->steering[element]);c->active=element;
  /* No stale integral is carried between element-specific wheel gains. */
  pid_reset(&c->wheel_l);pid_reset(&c->wheel_r);
 }
 g=&car_profiles[element];
 if(c->arm_age<=cfg->fan_arm_ticks)c->fan_command=0;
 /* Fan demand is independent for every element. Slew limited at 5 ms. */
 c->fan_command=(car_u16)clamp32((car_i32)c->fan_command+
  clamp32((car_i32)(c->arm_age<=cfg->fan_arm_ticks?0:g->fan)-c->fan_command,-cfg->fan_slew,cfg->fan_slew),0,1000);
 desired=steer_pid(&c->steering[element],g,error,in->gyro_z);
 delta=(car_i16)clamp32((car_i32)desired-c->last_steer,-g->slew,g->slew);
 c->last_steer+=delta;
 /* Acceleration limited; deceleration uses the same bounded ramp. */
 speed=(car_i16)(c->last_speed+clamp32((car_i32)g->speed-c->last_speed,-4,2));
 c->last_speed=speed;
 /* Tunable differential approximation; replace after chassis calibration. */
 delta=g->differential?(car_i16)((car_i32)speed*c->last_steer/1200):0;
 if (c->arm_age<=cfg->fan_arm_ticks) {speed=0;delta=0;c->last_speed=0;}
 o->target_l=speed-delta;o->target_r=speed+delta;
 o->left_pwm=wheel_pid(&c->wheel_l,g,o->target_l,in->wheel_l);
 o->right_pwm=wheel_pid(&c->wheel_r,g,o->target_r,in->wheel_r);
 o->steer=cfg->steer_mid+c->last_steer;o->element=element;o->enabled=1;
finished:
 o->gate=c->gate;o->fault=c->fault;
 o->finish_count=c->finish_count;o->finished=c->finished;
 o->fan_command=(car_u16)(cfg->fan_min+(car_i32)(cfg->fan_max-cfg->fan_min)*c->fan_command/1000);
 if (!c->armed) {c->fan_command=0;o->fan_command=cfg->fan_idle;o->enabled=0;o->left_pwm=0;o->right_pwm=0;o->target_l=0;o->target_r=0;}
}
