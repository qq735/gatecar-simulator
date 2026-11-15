#ifndef GATECAR_PLUGIN_API_H
#define GATECAR_PLUGIN_API_H
#include <stdint.h>
#define GATECAR_API_VERSION 1
#define GATECAR_CAMERA_WIDTH 80
#define GATECAR_CAMERA_HEIGHT 60
#ifdef _WIN32
#define GATECAR_EXPORT __declspec(dllexport)
#else
#define GATECAR_EXPORT __attribute__((visibility("default")))
#endif
#ifdef __cplusplus
extern "C" {
#endif
/* Fixed ABI. Header is copied into each compiled project by the simulator. */
typedef struct {
    int32_t api_version;
    int32_t struct_size;
    double dt_s, time_s;
    double wheelbase_m, track_width_m, counts_per_m, motor_max_speed_m_s;
    double speed_scale, adc_zero, mag_min_sum;
    int32_t adc_l, adc_r, wheel_l, wheel_r;
    double gyro_z_deg_s, visual_error_deg;
    int32_t frame_new, line_valid, gap_hint, arm, kill;
    uint8_t camera_gray[GATECAR_CAMERA_WIDTH * GATECAR_CAMERA_HEIGHT];
} GateCarInput;
typedef struct {
    double speed_m_s;       /* Open-loop speed demand used unless use_pwm=1 */
    double steer_deg;       /* Positive left, limited to +/-35 degrees */
    double fan;             /* 0..1, display/control demand, not simulated downforce */
    int32_t enabled;
    int32_t use_pwm;
    int32_t left_pwm, right_pwm; /* -10000..10000 */
    int32_t gate_state;     /* 0 vision,1 approach,2 guide,3 reacquire,4 clear */
    int32_t fault;          /* 0 none,1 not-ready,2 camera,3 line,4 magnetic,5 gate-limit,6 stop */
} GateCarOutput;
GATECAR_EXPORT int gatecar_api_version(void);
GATECAR_EXPORT int gatecar_input_size(void);
GATECAR_EXPORT int gatecar_output_size(void);
GATECAR_EXPORT void gatecar_reset(void);
GATECAR_EXPORT void gatecar_step(const GateCarInput *input, GateCarOutput *output);
#ifdef __cplusplus
}
#endif
#endif
