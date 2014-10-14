#include <math.h>

#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>

#define HZ                              10
#define BUFFER_SIZE                     1000

#define NODE_NAME                       "s8_motor_controller"
#define TOPIC_PWM                       "/arduino/pwm"
#define TOPIC_ENCODERS                  "/arduino/encoders"
#define TOPIC_TWIST                     "/s8/twist"

#define PARAM_NAME_LEFT_ALPHA           "alpha_left"
#define PARAM_NAME_RIGHT_ALPHA          "alpha_right"
#define PARAM_NAME_LEFT_KI              "ki_left"
#define PARAM_NAME_RIGHT_KI             "ki_right"
#define PARAM_NAME_LEFT_KP              "kp_left"
#define PARAM_NAME_RIGHT_KP             "kp_right"
#define PARAM_NAME_WHEEL_RADIUS         "wheel_radius"
#define PARAM_NAME_ROBOT_BASE           "robot_base"
#define PARAM_NAME_TICKS_PER_REV        "ticks_per_rev"
#define PARAM_NAME_PWM_LIMIT_HIGH       "pwm_limit_high"
#define PARAM_NAME_PWM_LIMIT_LOW        "pwm_limit_low"

// NB might have to increase KP and decrease KI
#define PARAM_DEFAULT_LEFT_ALPHA        1.0
#define PARAM_DEFAULT_RIGHT_ALPHA       -1.0
#define PARAM_DEFAULT_LEFT_KI           2.0
#define PARAM_DEFAULT_RIGHT_KI          -2.0
#define PARAM_DEFAULT_LEFT_KP           0.2
#define PARAM_DEFAULT_RIGHT_KP          -0.2
#define PARAM_DEFAULT_WHEEL_RADIUS      0.05
#define PARAM_DEFAULT_ROBOT_BASE        0.225
#define PARAM_DEFAULT_TICKS_PER_REV     360
#define PARAM_DEFAULT_PWM_LIMIT_HIGH    255
#define PARAM_DEFAULT_PWM_LIMIT_LOW     -255

class MotorController {
private:
    struct wheel {
        int delta_encoder;
        int pwm;
        double alpha;
        double ki;
        double kp;

        wheel() : delta_encoder(0), pwm(0), alpha(0.0), ki(0.0), kp(0.0) {}
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

    ros::NodeHandle n;
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
        n = ros::NodeHandle("~");
        init_params();
        print_params();
        pwm_publisher = n.advertise<ras_arduino_msgs::PWM>(TOPIC_PWM, BUFFER_SIZE);
        twist_subscriber = n.subscribe<geometry_msgs::Twist>(TOPIC_TWIST, BUFFER_SIZE, &MotorController::twist_callback, this);
        encoders_subscriber = n.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, BUFFER_SIZE, &MotorController::encoders_callback, this);
    }

    void update() {
        double left_w;
        double right_w;
        double est_left_w, est_right_w;
        double I_left = 0, I_right = 0;
        double p_err_left = 0, p_err_right = 0;

        est_left_w = estimate_w(wheel_left.delta_encoder);
        est_right_w = estimate_w(wheel_right.delta_encoder);
        mps_to_rps(v, w, left_w, right_w);

        p_controller(&wheel_left.pwm, wheel_left.alpha, left_w, est_left_w);
        p_controller(&wheel_right.pwm, wheel_right.alpha, right_w, est_right_w);
        i_controller(&wheel_left.pwm, wheel_left.ki, left_w, est_left_w, &I_left);
        i_controller(&wheel_right.pwm, wheel_right.ki, right_w, est_right_w, &I_right);
        d_controller(&wheel_left.pwm, wheel_left.kp, left_w, est_left_w, &p_err_left);
        d_controller(&wheel_right.pwm, wheel_right.kp, right_w, est_right_w, &p_err_right);

        check_pwm(&wheel_left.pwm, &wheel_right.pwm);

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

    void mps_to_rps(double v, double w, double & left_w, double & right_w) {
        left_w = (v - (params.robot_base / 2) * w) / params.wheel_radius;
        right_w = (v + (params.robot_base / 2) * w) / params.wheel_radius;
    }

    double estimate_w(double encoder_delta) {
        return (encoder_delta * 2 * M_PI * hz) / params.ticks_per_rev;
    }

    void p_controller(int * pwm, double alpha, double w, double est_w) {
        *pwm += alpha * (w - est_w);
    }

    void i_controller(int * pwm, double ki, double w, double est_w, double * sum_i){
        // check type of 1/Hz
        * sum_i += (w- est_w) * (1/HZ);
        * pwm += ki * *sum_i;
    }

    void d_controller(int * pwm, double kp, double w, double est_w, double * prev_err){
        * pwm += kp * ((w-est_w) - *prev_err) * HZ;
        * prev_err = w-est_w;
    }

    void check_pwm(int *left_pwm, int *right_pwm){
        if(*right_pwm > params.pwm_limit_high) {
            ROS_WARN("Right PWM reached positive saturation");
            *right_pwm = params.pwm_limit_high;
        } else if(*right_pwm < params.pwm_limit_low) {
            ROS_WARN("Right PWM reached negative saturation");
            *right_pwm = params.pwm_limit_low;
        }
        if(*left_pwm > params.pwm_limit_high) {
            ROS_WARN("Right PWM reached positive saturation");
            *left_pwm = params.pwm_limit_high;
        } else if(*left_pwm < params.pwm_limit_low) {
            ROS_WARN("Right PWM reached negative saturation");
            *left_pwm = params.pwm_limit_low;
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
        init_param(PARAM_NAME_LEFT_ALPHA, wheel_left.alpha, PARAM_DEFAULT_LEFT_ALPHA);
        init_param(PARAM_NAME_RIGHT_ALPHA, wheel_right.alpha, PARAM_DEFAULT_RIGHT_ALPHA);
        init_param(PARAM_NAME_ROBOT_BASE, params.robot_base, PARAM_DEFAULT_ROBOT_BASE);
        init_param(PARAM_NAME_WHEEL_RADIUS, params.wheel_radius, PARAM_DEFAULT_WHEEL_RADIUS);
        init_param(PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev, PARAM_DEFAULT_TICKS_PER_REV);
        init_param(PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high, PARAM_DEFAULT_PWM_LIMIT_HIGH);
        init_param(PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low, PARAM_DEFAULT_PWM_LIMIT_LOW);
        init_param(PARAM_NAME_LEFT_KI, wheel_left.ki, PARAM_DEFAULT_LEFT_KI);
        init_param(PARAM_NAME_RIGHT_KI, wheel_right.ki, PARAM_DEFAULT_RIGHT_KI);
        init_param(PARAM_NAME_LEFT_KP, wheel_left.kp, PARAM_DEFAULT_LEFT_KP);
        init_param(PARAM_NAME_RIGHT_KP, wheel_right.kp, PARAM_DEFAULT_RIGHT_KP);

    }

    template<class T>
    void init_param(const std::string & name, T & destination, T default_value) {
        if(!n.hasParam(name)) {
            n.setParam(name, default_value);
        }

        if(!n.getParam(name, destination)) {
            ROS_WARN("Failed to get parameter %s from param server. Falling back to default value.", name.c_str());
            destination = default_value;
        }
    }

    void print_params() {
        ROS_INFO("--Params--");
        ROS_INFO("%s: \t\t\t%lf", PARAM_NAME_LEFT_ALPHA, wheel_left.alpha);
        ROS_INFO("%s: \t\t\t%lf", PARAM_NAME_RIGHT_ALPHA, wheel_right.alpha);
        ROS_INFO("%s: \t\t\t%lf", PARAM_NAME_ROBOT_BASE, params.robot_base);
        ROS_INFO("%s: \t\t\t%lf", PARAM_NAME_WHEEL_RADIUS, params.wheel_radius);
        ROS_INFO("%s: \t\t\t%d", PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev);
        ROS_INFO("%s: \t\t%d", PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high);
        ROS_INFO("%s: \t\t\t%d", PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low);
        ROS_INFO("");
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
