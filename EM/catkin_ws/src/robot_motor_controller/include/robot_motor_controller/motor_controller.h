#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "DC_motor.hpp"
#include "Servo_motor.hpp"
#include "PCA9685.hpp"
#include "i2c.hpp"
#include <sensor_msgs/JointState.h>
#include <algorithm>

class MotorController
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Timer control_timer_;
    ros::Time last_cmd_time_;

    // I2C 디바이스 고정값
    const std::string I2C_DEVICE = "/dev/i2c-7";
    const int DC_ADDR = 0x40;
    const int SERVO_ADDR = 0x60;

    // 제어 파라미터 고정값
    const float MAX_LINEAR_VEL = 1.0f;   // m/s
    const float MAX_ANGULAR_VEL = 0.85f; // rad/s
    const float CONTROL_RATE = 50.0f;    // Hz
    const float ACCEL_LIMIT = 0.1f;      // 가속도 제한

    // I2C 관련 객체들
    I2CDevice i2c_dc_;
    I2CDevice i2c_servo_;
    PCA9685 pca_dc_;
    PCA9685 pca_servo_;
    PWMThrottleHat motor_;
    Servo servo_;
    int servo_channel_;

    // 속도 제어 변수들
    float target_linear_vel_;
    float target_angular_vel_;
    float current_linear_vel_;
    float current_angular_vel_;

    bool fgInitsetting;
    int velCmdUpdateCount;

    void controlTimerCallback(const ros::TimerEvent &event);
    float smoothControl(float target, float current, float rate);

public:
    MotorController(ros::NodeHandle &nh);
    ~MotorController();
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
};

#endif // MOTOR_CONTROLLER_H