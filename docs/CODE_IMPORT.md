# 控制代码接入说明

## Python

单个`.py`可任意命名。ZIP必须包含唯一的`controller.py`，其他辅助模块可放在入口同目录或以相对包结构组织。接口如下：

```python
def reset(config):
    # 可选。每次复位执行，返回本次测试的持久状态。
    return {'last_error': 0.0}

def step(sensors, state):
    # 必需。每5ms调用一次，state是reset返回的同一个对象。
    return dict(speed_m_s=0.8, steer_deg=0, fan=0.3, enabled=True)
```

`step`返回字典。运行错误、非有限输出、无效状态编号、C崩溃或100 ms无响应均停止测试，显示“用户代码错误”，不会自动切换理想路线。超时限制是本机计算耗时保护，不等于实车实时性验证。

修改文件后重新导入即可。原文件保留在用户电脑，导入快照进入本地`build/user_projects`。导入失败时保留原先已加载控制器。

## 输入 sensors / GateCarInput

| 字段 | 单位/说明 |
|---|---|
| dt_s / time_s | 固定0.005秒 / 仿真累计时间 |
| adc_l / adc_r | 0..4095，两路经过接收延迟的ADC包络，不是150 kHz原始载波 |
| adc_zero / mag_min_sum | 配置中的ADC零点与两路有效幅值和门限 |
| wheel_l / wheel_r | 本5ms内的编码器计数，已包含counts_per_m标定；近似速度＝计数/(dt_s×counts_per_m) |
| gyro_z_deg_s | °/s，正方向沿用主车适配层，为车辆逆时针yaw rate的相反数 |
| visual_error_deg | 从合成相机掩膜估计的前瞻误差，正为左，限制±30° |
| frame_new | 是否刚收到一张新相机帧。含camera_delay_ms延迟，断帧时保持旧图 |
| line_valid / gap_hint | 简化白道有效性 / 断口提示，可用自写视觉算法替换 |
| camera_gray | 长4800的灰度数组，80列×60行，row*80+column索引，白道240、背景24；无首帧时全0 |
| arm / kill | 第一周期启动 / 急停。宿主也会强制执行急停 |
| wheelbase_m / track_width_m | 米，车辆几何参数 |
| counts_per_m | 编码器每米计数 |
| motor_max_speed_m_s | 电机模型全PWM对应速度上限 |
| speed_scale | 用户界面的速度倍率，可由控制器自行采用；不偷偷改变用户编码器标定 |

不提供理想路线或真实位姿。地图中元素识别需要你自己的视觉算法。`reset(config)`只给车辆几何、编码器和接收门限等参数。

## 输出

| 字段 | 默认值及约定 |
|---|---|
| speed_m_s | 0，米/秒，默认开环速度需求，转为两轮相同PWM，经过电机模型后得到实际速度 |
| steer_deg | 0，正左负右，宿主限幅±35°，经过舵机延迟模型 |
| fan | 0，0..1需求，映射到500..1000电调指令，只显示/记录，不产生真实下压力 |
| enabled | false，必须显式置true才输出动力 |
| use_pwm | false，为true时采用left_pwm/right_pwm，忽略speed_m_s |
| left_pwm / right_pwm | 0，有符号-10000..10000，可在用户代码中实现轮速闭环与差速 |
| gate_state | 0..4：循迹、入口准备、磁引导、重捕获、离门确认，仅由代码报告 |
| fault | 0..6：无、未就绪、相机超时、丢线、弱磁、门区限时/限距、急停；非零撤动力 |

宿主对输出做范围检查。急停时即使用户代码继续请求驱动，输出也归零。故障后的车辆仍有电机模型惯性，不是物理刹车距离验证。

## C / C++

下载C示例ZIP，使用随包`gatecar_api.h`。用户实现：

```c
#include "gatecar_api.h"
void gatecar_reset(void) { }
void gatecar_step(const GateCarInput *in, GateCarOutput *out) {
    out->enabled = in->line_valid && !in->kill;
    out->speed_m_s = 0.7;
    out->steer_deg = 1.7 * in->visual_error_deg;
}
```

每周期输出结构体先清零。不要返回指向栈内存的对象，不要自行改头文件结构。仿真自动编译SDK的`abi.c`检查API版本和结构体大小，不要在项目中重复加入`abi.c`。

多文件ZIP要求唯一的`controller.c`或`controller.cpp`入口，最多30个C/C++文件，2 MB压缩包，展开最多8 MB、100个文件。包内只接受源码、头文件、文本、Markdown和JSON。不支持带绝对路径的链接配置、预编译Keil库或MCU专用汇编。

宿主通过PATH或`CC`查找gcc/clang，C++需要g++/clang++。Windows编译DLL，Linux编译SO，macOS编译dylib；编译库由下载者本机生成。

## 迁移现有嵌入式项目

1. 从工程中提取PID、状态机、图像处理等算法文件。
2. 为寄存器、GPIO、串口、ADC、定时器等外设依赖编写PC适配层。
3. 在`gatecar_reset()`初始化参数和控制器状态。
4. 在`gatecar_step()`将输入复制到算法，再把PWM/舵角写入输出。
5. 原图像算法如果使用不同分辨率，需要修改算法接口或先对80×60掩膜做转换。
6. 先用独立直道/单门工况看符号和单位，再测试圆环等复杂元素。

此接口测试PC上运行的控制逻辑，不能确认芯片编译、烧录、中断时序、摄像头DMA或实车接线。串口调试数据协议与接收板固件仍需另外实现。

## 运行权限

上传仅传给本机回环HTTP服务。运行用户代码等于运行本机程序，Python可以访问本机文件，C/C++也有本机程序权限。独立工作进程只提供异常、崩溃和卡死隔离，不是安全沙箱。不要将此服务公开到互联网，也不要运行来历不明的代码。
