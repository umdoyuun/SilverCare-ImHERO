#include "robot_motor_controller/motor_controller.h"
#include <iostream>

MotorController::MotorController(ros::NodeHandle &nh) : nh_(nh),
                                                        i2c_dc_(I2C_DEVICE.c_str(), static_cast<uint8_t>(DC_ADDR)),
                                                        i2c_servo_(I2C_DEVICE.c_str(), static_cast<uint8_t>(SERVO_ADDR)),
                                                        pca_dc_(i2c_dc_),
                                                        pca_servo_(i2c_servo_),
                                                        motor_(pca_dc_, 0),
                                                        servo_(pca_servo_),  // 마지막 쉼표 제거
                                                        fgInitsetting(true), // 이 값이 false로 되어있을 수 있음
                                                        velCmdUpdateCount(0)
{
    try
    {
        servo_channel_ = 0;
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 1, &MotorController::cmdVelCallback, this);
        control_timer_ = nh_.createTimer(ros::Duration(1.0 / CONTROL_RATE),
                                         &MotorController::controlTimerCallback, this);
        ROS_INFO("Motor Controller initialized successfully");
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR("Failed to initialize I2C devices: %s", e.what());
        ros::shutdown();
        return;
    }
}

MotorController::~MotorController()
{
    motor_.setThrottle(0.0);
    servo_.setAngle(servo_channel_, 0.0);
    ROS_INFO("Motor Stopped safely");
}

void MotorController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    if (fgInitsetting)
    {
        velCmdUpdateCount++;

        // angular velocity 먼저 처리
        target_angular_vel_ = static_cast<float>(msg->angular.z);
        target_angular_vel_ = (target_angular_vel_ > MAX_ANGULAR_VEL) ? MAX_ANGULAR_VEL : (target_angular_vel_ < -MAX_ANGULAR_VEL) ? -MAX_ANGULAR_VEL
                                                                                                                                   : target_angular_vel_;

        // linear velocity 처리
        target_linear_vel_ = static_cast<float>(msg->linear.x) * 2.0f;

        // linear velocity 제한
        target_linear_vel_ = (target_linear_vel_ > MAX_LINEAR_VEL) ? MAX_LINEAR_VEL : (target_linear_vel_ < -MAX_LINEAR_VEL) ? -MAX_LINEAR_VEL
                                                                                                                             : target_linear_vel_;

        if (fabs(target_angular_vel_) >= 0.75 && target_linear_vel_ >= 0.1)
        {
            target_linear_vel_ = 1.2f;
        }
        last_cmd_time_ = ros::Time::now();
    }
}

float MotorController::smoothControl(float target, float current, float rate)
{
    float diff = target - current;
    if (fabs(diff) < rate)
        return target;
    return current + (diff > 0 ? rate : -rate);
}

void MotorController::controlTimerCallback(const ros::TimerEvent &event)
{
    // Watchdog
    if ((ros::Time::now() - last_cmd_time_).toSec() > 0.5)
    {
        target_linear_vel_ = 0;
        target_angular_vel_ = 0;
    }

    // 부드러운 속도 변화
    current_linear_vel_ = smoothControl(target_linear_vel_, current_linear_vel_, ACCEL_LIMIT);
    current_angular_vel_ = smoothControl(target_angular_vel_, current_angular_vel_, ACCEL_LIMIT);

    // 급격한 회전시 속도 제한
    if (fabs(current_angular_vel_) >= 0.7 && fabs(current_linear_vel_) >= 0.3)
    {
        if (current_angular_vel_ < 0)
        {                                                                  // 우회전일 경우
            current_angular_vel_ = -0.7;                                   // 우회전 각도 감소
            current_linear_vel_ = (current_linear_vel_ >= 0) ? 1.0 : -1.0; // 속도 증가
        }
        else
        {                                                                    // 좌회전일 경우
            current_angular_vel_ = (current_angular_vel_ >= 0) ? 0.8 : -0.8; // 기존 동작 유지
            current_linear_vel_ = (current_linear_vel_ >= 0) ? 0.8 : -0.8;   // 기존 동작 유지
        }
    }

    // 모터 제어값 계산
    float throttle = current_linear_vel_;    // -1.0 ~ 1.0
    float angle = current_angular_vel_ * 45; // rad to degree

    // 실제 모터 제어
    motor_.setThrottle(throttle);
    servo_.setAngle(servo_channel_, -angle);
}
