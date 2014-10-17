#include <math.h>

#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>

#include <s8_common_node/Node.h>

#define HZ                              10
#define BUFFER_SIZE                     1000

#define NODE_NAME                       "s8_motor_controller"
#define TOPIC_PWM                       "/arduino/pwm"
#define TOPIC_ENCODERS                  "/arduino/encoders"
#define TOPIC_TWIST                     "/s8/twist"

#define PARAM_NAME_LEFT_KP              "kp_left"
#define PARAM_NAME_RIGHT_KP             "kp_right"
#define PARAM_NAME_LEFT_KI              "ki_left"
#define PARAM_NAME_RIGHT_KI             "ki_right"
#define PARAM_NAME_LEFT_KD              "kd_left"
#define PARAM_NAME_RIGHT_KD             "kd_right"
#define PARAM_NAME_WHEEL_RADIUS         "wheel_radius"
#define PARAM_NAME_ROBOT_BASE           "robot_base"
#define PARAM_NAME_TICKS_PER_REV        "ticks_per_rev"
#define PARAM_NAME_PWM_LIMIT_HIGH       "pwm_limit_high"
#define PARAM_NAME_PWM_LIMIT_LOW        "pwm_limit_low"

// NB might have to increase KP and decrease KI
#define PARAM_DEFAULT_LEFT_KP           1.0
#define PARAM_DEFAULT_RIGHT_KP          -1.0
#define PARAM_DEFAULT_LEFT_KI           1.0
#define PARAM_DEFAULT_RIGHT_KI          -1.0
#define PARAM_DEFAULT_LEFT_KD           1.0
#define PARAM_DEFAULT_RIGHT_KD          -1.0
#define PARAM_DEFAULT_WHEEL_RADIUS      0.05
#define PARAM_DEFAULT_ROBOT_BASE        0.225
#define PARAM_DEFAULT_TICKS_PER_REV     360
#define PARAM_DEFAULT_PWM_LIMIT_HIGH    255
#define PARAM_DEFAULT_PWM_LIMIT_LOW     -255

class MotorController : public s8::Node {
private:
    struct wheel {
        int delta_encoder;
        double pwm;
        double kp;
        double ki;
        double kd;
        double sum_errors; //The accumulated error (the same as intergral of error from 0 to t).
        double prev_error; //The previous error to be used for the derivative controller.

        wheel() : delta_encoder(0), pwm(0), kp(0.0), ki(0.0), kd(0.0), sum_errors(0.0), prev_error(0.0) {}
    };

    struct params_struct {
        double wheel_radius;
        double robot_base;
        int ticks_per_rev;
        int pwm_limit_high;
        int pwm_limit_low;

        params_struct() : wheel_radius(0.0), robot_base(0.0), ticks_per_rev(0), pwm_limit_high(0), pwm_limit_low(0) {}
    };

    const int hz;

    ros::Publisher pwm_publisher;
    ros::Subscriber twist_subscriber;
    ros::Subscriber encoders_subscriber;

    params_struct params;
    wheel wheel_left;
    wheel wheel_right;
    double v;
    double w;
    
public:
    MotorController(int hz) : hz(hz), v(0), w(0) {
        init_params();
        print_params();
        pwm_publisher = nh.advertise<ras_arduino_msgs::PWM>(TOPIC_PWM, BUFFER_SIZE);
        twist_subscriber = nh.subscribe<geometry_msgs::Twist>(TOPIC_TWIST, BUFFER_SIZE, &MotorController::twist_callback, this);
        encoders_subscriber = nh.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, BUFFER_SIZE, &MotorController::encoders_callback, this);
    }

    void update() {
        double left_w;
        double right_w;
        calculate_wheel_velocities(v, w, left_w, right_w);

        double est_left_w = estimate_w(wheel_left.delta_encoder);
        double est_right_w = estimate_w(wheel_right.delta_encoder);

        p_controller(wheel_left.pwm, wheel_left.kp, left_w, est_left_w);
        p_controller(wheel_right.pwm, wheel_right.kp, right_w, est_right_w);
        i_controller(wheel_left.pwm, wheel_left.ki, left_w, est_left_w, wheel_left.sum_errors);
        i_controller(wheel_right.pwm, wheel_right.ki, right_w, est_right_w, wheel_right.sum_errors);
        d_controller(wheel_left.pwm, wheel_left.kd, left_w, est_left_w, wheel_left.prev_error);
        d_controller(wheel_right.pwm, wheel_right.kd, right_w, est_right_w, wheel_right.prev_error);

	ROS_INFO("left i-error: %lf, right i-error: %lf", wheel_left.sum_errors, wheel_right.sum_errors);

        check_pwm(wheel_left.pwm, wheel_right.pwm);

        publish_pwm(wheel_left.pwm, wheel_right.pwm);
    }

private:
    void twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
        v = twist->linear.x;
        w = twist->angular.z;
    }

    void encoders_callback(const ras_arduino_msgs::Encoders::ConstPtr & encoders) {
        wheel_left.delta_encoder = encoders->delta_encoder2;
        wheel_right.delta_encoder = encoders->delta_encoder1;
    }

    void calculate_wheel_velocities(double v, double w, double & left_w, double & right_w) {
        left_w = (v - (params.robot_base / 2) * w) / params.wheel_radius;
        right_w = (v + (params.robot_base / 2) * w) / params.wheel_radius;
    }

    double estimate_w(double encoder_delta) {
        return (encoder_delta * 2 * M_PI * hz) / params.ticks_per_rev;
    }

    void p_controller(double & pwm, double kp, double w, double est_w) {
        pwm += kp * (w - est_w);
    }

    void i_controller(double & pwm, double ki, double w, double est_w, double & sum_errors){
        sum_errors += (w - est_w) * (1.0 / hz);
        pwm += ki * sum_errors;
    }

    void d_controller(double & pwm, double kd, double w, double est_w, double & prev_error){
        pwm += kd * ((w - est_w) - prev_error) * hz;
        prev_error = w - est_w;
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
            ROS_WARN("Right PWM reached positive saturation");
            left_pwm = params.pwm_limit_high;
        } else if(l < params.pwm_limit_low) {
            ROS_WARN("Right PWM reached negative saturation");
            left_pwm = params.pwm_limit_low;
        }
    }

    void publish_pwm(int left, int right) {
        ras_arduino_msgs::PWM pwm_message;
        pwm_message.PWM1 = right;
        pwm_message.PWM2 = left;

        pwm_publisher.publish(pwm_message);

        ROS_INFO("left: %d right: %d", left, right);
    }

    void init_params() {
        add_param(PARAM_NAME_LEFT_KP, wheel_left.kp, PARAM_DEFAULT_LEFT_KP);
        add_param(PARAM_NAME_RIGHT_KP, wheel_right.kp, PARAM_DEFAULT_RIGHT_KP);
        add_param(PARAM_NAME_LEFT_KI, wheel_left.ki, PARAM_DEFAULT_LEFT_KI);
        add_param(PARAM_NAME_RIGHT_KI, wheel_right.ki, PARAM_DEFAULT_RIGHT_KI);
        add_param(PARAM_NAME_LEFT_KD, wheel_left.kd, PARAM_DEFAULT_LEFT_KD);
        add_param(PARAM_NAME_RIGHT_KD, wheel_right.kd, PARAM_DEFAULT_RIGHT_KD);
        add_param(PARAM_NAME_ROBOT_BASE, params.robot_base, PARAM_DEFAULT_ROBOT_BASE);
        add_param(PARAM_NAME_WHEEL_RADIUS, params.wheel_radius, PARAM_DEFAULT_WHEEL_RADIUS);
        add_param(PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev, PARAM_DEFAULT_TICKS_PER_REV);
        add_param(PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high, PARAM_DEFAULT_PWM_LIMIT_HIGH);
        add_param(PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low, PARAM_DEFAULT_PWM_LIMIT_LOW);
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
