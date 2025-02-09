#include "cmd_to_can_node.hpp"

ros::Publisher can_pub;

/*====== 清扫相关参数 ======*/
// 当前清扫状态
bool current_sweep_status = false;

// 清扫相关数据
static uint8_t g_sweep_data = 0x00;                 // 清扫相关控制位 (data[6])
static uint8_t g_gear_data = 0x00;                  // 档位控制位 (data[7])

// 清扫控制回调函数
void sweepControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data) {
        // 开启清扫：
        /* Byte 6 
         - 电刷设置为双刷
         - 刹车为MCU控制
         - 喷水关闭
         - 风机开启     */

        /* Byte 7
         - 边刷关闭
         - 滚刷关闭
         - 主刷开启     */

        // 设置清扫数据
        g_sweep_data = (static_cast<uint8_t>(BrushControl::BOTH) << BRUSH_OFFSET) |
                       (static_cast<uint8_t>(BrakeControl::MCU) << BRAKE_OFFSET) |
                       (static_cast<uint8_t>(WaterControl::OFF) << WATER_OFFSET) |
                       (static_cast<uint8_t>(FanControl::ON) << FAN_OFFSET);

        g_gear_data = (static_cast<uint8_t>(BrushControl::OFF) << SIDE_BRUSH_OFFSET) |
                      (static_cast<uint8_t>(BrushControl::OFF) << ROLLER_BRUSH_OFFSET) |
                      (static_cast<uint8_t>(BrushControl::BOTH) << MAIN_BRUSH_OFFSET);
                      
        ROS_INFO("Sweeping ON, setting data: %x %x", g_sweep_data, g_gear_data);
    } else {
        // 关闭清扫
        /* Byte 6 
         - 电刷关闭
         - 刹车为MCU控制
         - 喷水关闭
         - 风机关闭     */

        /* Byte 7
        - 边刷关闭
        - 滚刷关闭
        - 主刷关闭     */
        g_sweep_data = (static_cast<uint8_t>(BrushControl::OFF) << BRUSH_OFFSET) |
                       (static_cast<uint8_t>(BrakeControl::MCU) << BRAKE_OFFSET) |
                       (static_cast<uint8_t>(WaterControl::OFF) << WATER_OFFSET) |
                       (static_cast<uint8_t>(FanControl::OFF) << FAN_OFFSET);

        g_gear_data = (static_cast<uint8_t>(BrushControl::OFF) << SIDE_BRUSH_OFFSET) |
                      (static_cast<uint8_t>(BrushControl::OFF) << ROLLER_BRUSH_OFFSET) |
                      (static_cast<uint8_t>(BrushControl::OFF) << MAIN_BRUSH_OFFSET);

        ROS_INFO("Sweeping OFF, setting data: %x %x", g_sweep_data, g_gear_data);
    }
}

// 车辆控制回调函数
void vehicleCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& msg) {
    can_msgs::Frame can_frame;

    // 初始化 CAN 帧
    can_frame.id = 0x501;  // 底盘协议指令 ID 0x501
    can_frame.dlc = 8;     // 数据长度为 8 字节

    // 提取线速度和角速度
    float speed = msg->twist_cmd.twist.linear.x;                // 线速度 (m/s)
    float angular_velocity = msg->twist_cmd.twist.angular.z;    // 角速度 (rad/s)

    ROS_INFO("Received linear speed %f, angular speed %f", speed, angular_velocity);    

    /*********** 线速度处理 ***********/
    // 判断档位：正值速度为前进，负值速度为后退
    uint8_t gear = (speed >= 0) ? static_cast<uint8_t>(GearControl::FORWARD) : static_cast<uint8_t>(GearControl::BACKWARD);
    speed = fabs(speed);  // 取绝对值用于速度计算

    // 线速度转换为转速
    float wheel_speed = speed / SPEED_TO_WHEEL;  // 单位为 RPM

    // 转换为 CAN 消息格式
    int16_t wheel_speed_can = static_cast<int16_t>(wheel_speed);  // 转换为整数

    /*********** 角速度处理 ***********/
    // 计算车轮转角
    double wheel_angle = 0.0;
    if (std::abs(speed) > 1e-3) {  // 避免速度接近 0 时的计算错误
        wheel_angle = atan(WHEEL_BASE * angular_velocity / speed) * (180.0 / M_PI);  // 转换为度
    }

    // 限制车轮转角范围
    wheel_angle = std::max(-WHEEL_ANGLE_MAX, std::min(wheel_angle, WHEEL_ANGLE_MAX));

    // 映射车轮转角到底盘编码
    int16_t chassis_angle_encoded = static_cast<int16_t>(ANGLE_OFFSET + wheel_angle * ANGLE_SCALING);

    /*********** 数据打包并发送 ***********/

    // 填充数据到 CAN 帧
    can_frame.data[0] = 0x0F;                       // 00001111 使能油门，转向，档位，作业，驱动
    can_frame.data[1] = 0x00;
    can_frame.data[2] = (wheel_speed_can >> 8) & 0xFF;    // 速度高字节
    can_frame.data[3] = wheel_speed_can & 0xFF;           // 速度低字节
    can_frame.data[4] = (chassis_angle_encoded >> 8) & 0xFF;    // 转角高字节
    can_frame.data[5] = chassis_angle_encoded & 0xFF;          // 转角低字节
    can_frame.data[6] = g_sweep_data;                   // 清扫控制数据
    can_frame.data[7] = (gear << GEAR_OFFSET) |    // 档位和清扫设备控制
                        (g_gear_data & 0xFC);       // 保留高6位的清扫设备控制状态

    ROS_INFO("%x %x %x %x %x %x %x %x", can_frame.data[0], can_frame.data[1], can_frame.data[2], can_frame.data[3], can_frame.data[4], can_frame.data[5], can_frame.data[6], can_frame.data[7]);

    // 发布 CAN 消息
    can_pub.publish(can_frame);

    ROS_INFO("Publinshing CAN msg: ID = 0x%x, Steering Angle = %d, Wheel Speed = %d", can_frame.id, chassis_angle_encoded, wheel_speed_can);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_to_can_node");
    ros::NodeHandle nh;

    ROS_INFO("cmd_to_can_node started, subscribing /vehicle_cmd and /sweep_control, publishing /sent_message");

    // 订阅 Autoware 发布的 vehicle_cmd
    ros::Subscriber cmd_sub = nh.subscribe("/vehicle_cmd", 10, vehicleCmdCallback);

    // 订阅清扫控制命令
    ros::Subscriber sweep_sub = nh.subscribe("/sweep_control", 10, sweepControlCallback);

    // 发布转换后的 CAN 消息
    can_pub = nh.advertise<can_msgs::Frame>("/sent_messages", 10);

    ros::spin();
    return 0;
}
