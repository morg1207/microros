#ifndef ROS_COMMUNICATION_H
#define ROS_COMMUNICATION_H

#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "motor_controller.h"

#include "uvrobot_interfaces/msg/control_status.h"
#include "uvrobot_interfaces/msg/config_params.h"
#include "uvrobot_interfaces/msg/motors.h"

#include "vector"

#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {} }

// Motors setup
#define MOTOR_R MOTOR1
#define MOTOR_L MOTOR2

constexpr const char* LEFT_MOTOR_NAME = "left_joint";
constexpr const char* RIGHT_MOTOR_NAME = "right_joint";

enum Motors
{
  motor_right,
  motor_left,
  MOTORS_COUNT
};

enum MotorsState
{
  motor_state_position,
  motor_state_velocity,
  motor_state_effort,
  MOTORS_STATE_COUNT
};



class RosCommunication{
    public:
        RosCommunication(); // Constructor que acepta los parámetros del Encoder
        //RosCommunication();
        void initialize();

        void subscribers_define();
        void publishers_define();
        void timers_define();

        static void led_callback(const void *msg_recv);
        static void set_vel_motors_callback(const void *msg_recv);
        static void set_switch_control_callback(const void *msg_recv);
        static void set_pwm_motors_callback(const void *msg_recv);
        static void config_params_callback(const void *msg_recv);

        static void  timer_callback(rcl_timer_t *timer, int64_t last_call_time);
        static void  timer_callback_control(rcl_timer_t *timer, int64_t last_call_time);

        unsigned long lastTime_; // Último tiempo para cálculo de velocidad
        void start_receiving_msgs();
        void executors_start();

        void error_loop();

        MotorController* motorControllerL;
        MotorController* motorControllerR;

        void updatWheelsStates();
        void fill_wheels_state_msg(sensor_msgs__msg__JointState* msg);
        void fill_wheels_command_msg(std_msgs__msg__Float32MultiArray* msg);
    private:

    static float linear_vel;
    static float angular_vel;

};

// Error handle loop

#endif