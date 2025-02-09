#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Bool.h>

/*====== 车辆相关参数 ======*/
// 实际轮距
const double WHEEL_BASE = 0.85;
// 转速与车速比
const double SPEED_TO_WHEEL = 0.001574;
// 转角比例（车轮转角到底盘）
const double ANGLE_SCALING = 15.0;  // 车轮转角与底盘转角的比例
// 转角偏移量
const int16_t ANGLE_OFFSET = 1024;
// 转角范围（单位：度）
const float ANGLE_MAX = 300.0;
// 根据车轮转角限制范围
const double WHEEL_ANGLE_MAX = ANGLE_MAX / ANGLE_SCALING;  // 最大车轮转角

/*====== 数据偏移量定义 ======*/
// Byte 6
static const uint8_t BRUSH_OFFSET = 0x00;           // 电刷控制数据位偏移
static const uint8_t BRAKE_OFFSET = 0x03;           // 刹车控制数据位偏移
static const uint8_t WATER_OFFSET = 0x05;           // 喷水控制数据位偏移
static const uint8_t FAN_OFFSET = 0x06;             // 风机控制数据位偏移
// Byte 7
static const uint8_t GEAR_OFFSET = 0x00;            // 档位控制数据位偏移
static const uint8_t SIDE_BRUSH_OFFSET = 0x02;      // 边刷控制数据位偏移
static const uint8_t ROLLER_BRUSH_OFFSET = 0x04;    // 滚刷控制数据位偏移
static const uint8_t MAIN_BRUSH_OFFSET = 0x06;      // 主刷控制数据位偏移

/*====== 数据含义定义 ======*/
// 电刷控制
enum class BrushControl : uint8_t {
    OFF = 0x00,         // 关闭
    LEFT = 0x01,        // 左刷
    RIGHT = 0x02,       // 右刷
    BOTH = 0x03         // 双刷
};
// 刹车控制
enum class BrakeControl : uint8_t {
    MCU = 0x00,         // MCU 控制
    RELEASE = 0x01,     // 强制释放
    LOCK = 0x03         // 强制抱死
};
// 喷水控制
enum class WaterControl : uint8_t {
    OFF = 0x00,         // 关闭
    ON = 0x01           // 开启
};
// 风机控制
enum class FanControl : uint8_t {
    OFF = 0x00,         // 关闭
    ON = 0x01           // 开启
};
// 档位控制
enum class GearControl : uint8_t {
    BACKWARD = 0x01,    // 后退
    NEUTRAL = 0x02,     // 空挡
    FORWARD = 0x03      // 前进
};
// 边刷控制
enum class SideBrushControl : uint8_t {
    OFF = 0x00,         // 关闭
    UP = 0x01,          // 上升
    DOWN = 0x02         // 下降
};
// 滚刷控制
enum class RollerBrushControl : uint8_t {
    OFF = 0x00,         // 关闭
    UP = 0x01,          // 上升
    DOWN = 0x02         // 下降
};
// 主刷控制
enum class MainBrushControl : uint8_t {
    OFF = 0x00,         // 关闭
    ON = 0x01           // 开启
};