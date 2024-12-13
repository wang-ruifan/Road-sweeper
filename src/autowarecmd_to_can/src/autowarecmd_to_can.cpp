#include <ros/ros.h>
#include <autoware_msgs/VehicleCmd.h>
#include <can_msgs/Frame.h>

ros::Publisher can_pub;

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
    uint8_t gear = (speed >= 0) ? 0x03 : 0x01;  // 0x01: 前进，0x02: 后退
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
    can_frame.data[0] = 0x0F;                       // 使能油门，转向，档位，作业，驱动
    can_frame.data[1] = 0x00;
    can_frame.data[2] = (wheel_speed_can >> 8) & 0xFF;    // 速度高字节
    can_frame.data[3] = wheel_speed_can & 0xFF;           // 速度低字节
    can_frame.data[4] = (chassis_angle_encoded >> 8) & 0xFF;    // 转角高字节
    can_frame.data[5] = chassis_angle_encoded & 0xFF;          // 转角低字节
    can_frame.data[6] = 0x00;                       //暂时不需要电刷，喷水和风机                        
    can_frame.data[7] = gear;                       // 档位标志

    ROS_INFO("%x %x %x %x %x %x %x %x", can_frame.data[0], can_frame.data[1], can_frame.data[2], can_frame.data[3], can_frame.data[4], can_frame.data[5], can_frame.data[6], can_frame.data[7]);

    // 发布 CAN 消息
    can_pub.publish(can_frame);

    ROS_INFO("Publinshing CAN msg: ID = 0x%x, Steering Angle = %d, Wheel Speed = %d", can_frame.id, chassis_angle_encoded, wheel_speed_can);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicle_cmd_to_can");
    ros::NodeHandle nh;

    ROS_INFO("Autowarecmd_to_can_node started, subscribing /vehicle_cmd, publishing /sent_message");

    // 订阅 Autoware 发布的 vehicle_cmd
    ros::Subscriber cmd_sub = nh.subscribe("/vehicle_cmd", 10, vehicleCmdCallback);

    // 发布转换后的 CAN 消息
    can_pub = nh.advertise<can_msgs::Frame>("/sent_messages", 10);

    ros::spin();
    return 0;
}
