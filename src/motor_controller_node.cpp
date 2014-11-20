#include <cmath>
#include <ros/ros.h>

#include <s8_motor_controller/motor_controller_node.h>
#include <s8_utils/math.h>
#include <s8_common_node/Node.h>
#include <s8_pid/PIDController.h>
#include <actionlib/server/simple_action_server.h>

#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>
#include <s8_motor_controller/StopAction.h>

#define HZ                                          25
#define BUFFER_SIZE                                 1

#define PARAM_NAME_LEFT_KP                          "kp_left"
#define PARAM_NAME_RIGHT_KP                         "kp_right"
#define PARAM_NAME_LEFT_KI                          "ki_left"
#define PARAM_NAME_RIGHT_KI                         "ki_right"
#define PARAM_NAME_LEFT_KD                          "kd_left"
#define PARAM_NAME_RIGHT_KD                         "kd_right"
#define PARAM_NAME_WHEEL_RADIUS                     "wheel_radius"
#define PARAM_NAME_ROBOT_BASE                       "robot_base"
#define PARAM_NAME_TICKS_PER_REV                    "ticks_per_rev"
#define PARAM_NAME_PWM_LIMIT_HIGH                   "pwm_limit_high"
#define PARAM_NAME_PWM_LIMIT_LOW                    "pwm_limit_low"
#define PARAM_NAME_GO_IDLE_TIME                     "go_idle_time"
#define PARAM_NAME_ENCODERS_STILL_TRESHOLD          "encoders_still_treshold"
#define PARAM_NAME_PWM_TRESHOLD_LOW                 "pwm_treshold_low"

// NB might have to increase KP and decrease KI
#define PARAM_DEFAULT_LEFT_KP                       0.92
#define PARAM_DEFAULT_RIGHT_KP                      -1.15
#define PARAM_DEFAULT_LEFT_KI                       0.0
#define PARAM_DEFAULT_RIGHT_KI                      0.0
#define PARAM_DEFAULT_LEFT_KD                       0.5
#define PARAM_DEFAULT_RIGHT_KD                      -0.42
#define PARAM_DEFAULT_WHEEL_RADIUS                  0.05
#define PARAM_DEFAULT_ROBOT_BASE                    0.205
#define PARAM_DEFAULT_TICKS_PER_REV                 360
#define PARAM_DEFAULT_PWM_LIMIT_HIGH                255
#define PARAM_DEFAULT_PWM_LIMIT_LOW                 -255
#define PARAM_DEFAULT_GO_IDLE_TIME                  1.0
#define PARAM_DEFAULT_ENCODERS_STILL_TRESHOLD       3
#define PARAM_DEFAULT_PWM_TRESHOLD_LOW              25

using namespace s8::motor_controller_node;
using namespace s8::pid;
using namespace s8::utils::math;

class MotorController : public s8::Node {
private:
    struct wheel {
        int delta_encoder;
        double pwm;
        PIDController pid_controller;

        wheel(int hz) : pid_controller(hz) {
            reset();
        }

        void reset() {
            delta_encoder = 0;
            pwm = 0;
            pid_controller.reset();
        }
    };

    struct params_struct {
        double wheel_radius;
        double robot_base;
        int ticks_per_rev;
        int pwm_limit_high;
        int pwm_limit_low;
        int pwm_treshold_low;

        params_struct() : wheel_radius(0.0), robot_base(0.0), ticks_per_rev(0), pwm_limit_high(0), pwm_limit_low(0), pwm_treshold_low(0.0) {}
    };

    const int hz;

    ros::Publisher pwm_publisher;
    ros::Publisher actual_twist_publisher;
    ros::Subscriber twist_subscriber;
    ros::Subscriber encoders_subscriber;
    actionlib::SimpleActionServer<s8_motor_controller::StopAction> stop_action;

    params_struct params;
    wheel wheel_left;
    wheel wheel_right;
    double v;
    double w;
    double go_idle_time; //If no packages recieved for this amount of seconds, go idle.
    int updates_since_last_twist;
    bool idle;
    bool is_stopping;
    int encoder_still_treshold;
    bool is_still;
    int ticks_since_last_encoder_callback;

public:
    MotorController(int hz) : Node(), hz(hz), wheel_left(hz), wheel_right(hz), v(0), w(0), updates_since_last_twist(0), idle(false), is_still(false), stop_action(nh, ACTION_STOP, boost::bind(&MotorController::action_execute_stop_callback, this, _1), false), is_stopping(false) {
        init_params();
        print_params();
        pwm_publisher = nh.advertise<ras_arduino_msgs::PWM>(TOPIC_PWM, BUFFER_SIZE);
        actual_twist_publisher = nh.advertise<geometry_msgs::Twist>(TOPIC_ACTUAL_TWIST, BUFFER_SIZE);
        twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_TWIST, BUFFER_SIZE, &MotorController::twist_callback, this);
        encoders_subscriber = nh.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, BUFFER_SIZE, &MotorController::encoders_callback, this);
        stop_action.start();
    }

    void update() {
        if(idle) {
            return;
        }

        if(updates_since_last_twist > go_idle_time * hz) {
            ROS_WARN("Turning idle due to no messages in %.2lf seconds", go_idle_time);
            stop();
            return;
        }

        double left_w;
        double right_w;
        calculate_wheel_velocities(v, w, left_w, right_w);

        double est_left_w = estimate_w(wheel_left.delta_encoder);
        double est_right_w = estimate_w(wheel_right.delta_encoder);

        wheel_left.pid_controller.update(wheel_left.pwm, left_w - est_left_w);
        wheel_right.pid_controller.update(wheel_right.pwm, right_w - est_right_w);

        check_pwm(wheel_left.pwm, wheel_right.pwm);
        publish_pwm(wheel_left.pwm, wheel_right.pwm);

        auto get_actual_v = [&est_right_w, &est_left_w, this]() {
            return ((est_right_w + est_left_w) / 2) * params.wheel_radius;
        };

        auto get_actual_w = [&est_right_w, &est_left_w, this]() {
            return ((est_right_w - est_left_w) / params.robot_base) * params.wheel_radius;
        };

        publish_actual_twist(get_actual_v(), get_actual_w());

        updates_since_last_twist++;
    }

private:
    void stop() {
        v = 0.0;
        w = 0.0;
        wheel_left.reset();
        wheel_right.reset();
        idle = true;
    }

    void action_execute_stop_callback(const s8_motor_controller::StopGoalConstPtr & goal) {
        ROS_INFO("STOP");

        if(is_stopping) {
            ROS_FATAL("Stop action callback executed but is already stopping");
        }

        stop();
        is_stopping = true;

        const int timeout = 10; // 10 seconds.
        const int rate_hz = 10;

        ros::Rate rate(rate_hz);

        const int encoder_callback_ticks_treshold = 10;

        int ticks = 0;

        while(!is_still && ticks <= timeout * rate_hz && ticks_since_last_encoder_callback < encoder_callback_ticks_treshold) {
            rate.sleep();
            ticks++;
            ticks_since_last_encoder_callback++;
        }

        if(ticks_since_last_encoder_callback >= encoder_callback_ticks_treshold) {
            ROS_INFO("No encoder callback in %d ticks. Assuming still.", ticks_since_last_encoder_callback);
            is_still = true;
        }

        if(!is_still) {
            ROS_WARN("Unable to stop. Stop action failed.");
            s8_motor_controller::StopResult stop_action_result;
            stop_action_result.stopped = false;
            stop_action.setAborted(stop_action_result);
        } else {
            s8_motor_controller::StopResult stop_action_result;
            stop_action_result.stopped = true;
            stop_action.setSucceeded(stop_action_result);
            ROS_INFO("Stop action succeeded");
        }

        is_stopping = false;
    }

    void twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
        if(is_stopping) {
            ROS_WARN("Twist recieved while stopping! Ignoring twist until stopped.");
            return;
        }

        v = twist->linear.x;
        w = twist->angular.z;
        updates_since_last_twist = 0;
        idle = false;
        is_still = false;
    }

    void encoders_callback(const ras_arduino_msgs::Encoders::ConstPtr & encoders) {
        ticks_since_last_encoder_callback = 0;

        int left = encoders->delta_encoder2;
        int right = encoders->delta_encoder1;

        if(idle) {
            //If the wheels are moving we need to stop them.
            //Sometimes the encoders show 1/2/3 when its still.
            if(std::abs(left) >= encoder_still_treshold || std::abs(right) >= encoder_still_treshold) {
                publish_pwm(0, 0);
                ROS_INFO("Controller is idle but wheels are moving. Left: %d, right: %d. Stopping...", left, right);
                is_still = false;
            } else {
                is_still = true;
            }
        }

        wheel_left.delta_encoder = left;
        wheel_right.delta_encoder = right;
    }

    void calculate_wheel_velocities(double v, double w, double & left_w, double & right_w) {
        left_w = (v - (params.robot_base / 2) * w) / params.wheel_radius;
        right_w = (v + (params.robot_base / 2) * w) / params.wheel_radius;
    }

    double estimate_w(double encoder_delta) {
        return (encoder_delta * 2 * M_PI * hz) / params.ticks_per_rev;
    }

    void check_pwm(double & left_pwm, double & right_pwm) {
        int r = right_pwm;
        int l = left_pwm;

        if(r > params.pwm_limit_high) {
            ROS_WARN("Right PWM reached positive saturation");
            right_pwm = params.pwm_limit_high;
        } else if(r < params.pwm_limit_low) {
            ROS_WARN("Right PWM reached negative saturation");
            right_pwm = params.pwm_limit_low;
        }
        if(l > params.pwm_limit_high) {
            ROS_WARN("Left PWM reached positive saturation");
            left_pwm = params.pwm_limit_high;
        } else if(l < params.pwm_limit_low) {
            ROS_WARN("Left PWM reached negative saturation");
            left_pwm = params.pwm_limit_low;
        }

        if(is_zero(v)) {
            if(r != 0 && std::abs(r) < params.pwm_treshold_low) {
                ROS_WARN("Right PWM: %d is below treshold", r);
                right_pwm = sign(r) * params.pwm_treshold_low;
            }

            if(l != 0 && std::abs(l) < params.pwm_treshold_low) {
                ROS_WARN("Left PWM: %d is below treshold", l);
                left_pwm = sign(l) * params.pwm_treshold_low;
            }
        }
    }

    void publish_pwm(int left, int right) {
        ras_arduino_msgs::PWM pwm_message;
        pwm_message.PWM1 = right;
        pwm_message.PWM2 = left;

        pwm_publisher.publish(pwm_message);

        ROS_INFO("left: %d right: %d", left, right);
    }

    void publish_actual_twist(double v, double w) {
        geometry_msgs::Twist twist_message;
        twist_message.linear.x = v;
        twist_message.angular.z = w;
        actual_twist_publisher.publish(twist_message);
    }

    void init_params() {
        add_param(PARAM_NAME_LEFT_KP, wheel_left.pid_controller.kp, PARAM_DEFAULT_LEFT_KP);
        add_param(PARAM_NAME_RIGHT_KP, wheel_right.pid_controller.kp, PARAM_DEFAULT_RIGHT_KP);
        add_param(PARAM_NAME_LEFT_KI, wheel_left.pid_controller.ki, PARAM_DEFAULT_LEFT_KI);
        add_param(PARAM_NAME_RIGHT_KI, wheel_right.pid_controller.ki, PARAM_DEFAULT_RIGHT_KI);
        add_param(PARAM_NAME_LEFT_KD, wheel_left.pid_controller.kd, PARAM_DEFAULT_LEFT_KD);
        add_param(PARAM_NAME_RIGHT_KD, wheel_right.pid_controller.kd, PARAM_DEFAULT_RIGHT_KD);
        add_param(PARAM_NAME_ROBOT_BASE, params.robot_base, PARAM_DEFAULT_ROBOT_BASE);
        add_param(PARAM_NAME_WHEEL_RADIUS, params.wheel_radius, PARAM_DEFAULT_WHEEL_RADIUS);
        add_param(PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev, PARAM_DEFAULT_TICKS_PER_REV);
        add_param(PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high, PARAM_DEFAULT_PWM_LIMIT_HIGH);
        add_param(PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low, PARAM_DEFAULT_PWM_LIMIT_LOW);
        add_param(PARAM_NAME_PWM_TRESHOLD_LOW, params.pwm_treshold_low, PARAM_DEFAULT_PWM_TRESHOLD_LOW);
        add_param(PARAM_NAME_GO_IDLE_TIME, go_idle_time, PARAM_DEFAULT_GO_IDLE_TIME);
        add_param(PARAM_NAME_ENCODERS_STILL_TRESHOLD, encoder_still_treshold, PARAM_DEFAULT_ENCODERS_STILL_TRESHOLD);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    MotorController motor_controller(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        motor_controller.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
