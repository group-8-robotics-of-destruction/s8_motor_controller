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
#define PARAM_NAME_WHEEL_RADIUS         "wheel_radius"
#define PARAM_NAME_ROBOT_BASE           "robot_base"
#define PARAM_NAME_TICKS_PER_REV        "ticks_per_rev"
#define PARAM_NAME_PWM_LIMIT_HIGH       "pwm_limit_high"
#define PARAM_NAME_PWM_LIMIT_LOW        "pwm_limit_low"

#define PARAM_DEFAULT_LEFT_ALPHA        1.0
#define PARAM_DEFAULT_RIGHT_ALPHA       1.0
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

        wheel() : delta_encoder(0), pwm(0), alpha(0.0) {}
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
        set_default_params();
        print_params();
        pwm_publisher = n.advertise<ras_arduino_msgs::PWM>(TOPIC_PWM, BUFFER_SIZE);
        twist_subscriber = n.subscribe<geometry_msgs::Twist>(TOPIC_TWIST, BUFFER_SIZE, &MotorController::twist_callback, this);
        encoders_subscriber = n.subscribe<ras_arduino_msgs::Encoders>(TOPIC_ENCODERS, BUFFER_SIZE, &MotorController::encoders_callback, this);
    }

    void update() {
        update_params();

        double left_w;
        double right_w;
        mps_to_rps(v, w, left_w, right_w);

        ROS_INFO("v: %lf w: %lf left_w: %lf right_w: %lf", v, w, left_w, right_w);

        p_controller(&wheel_left.pwm, wheel_left.alpha, left_w, wheel_left.delta_encoder);
        p_controller(&wheel_right.pwm, wheel_right.alpha, right_w, wheel_right.delta_encoder);

        publish_pwm(wheel_left.pwm, wheel_right.pwm);
    }

private:
    void twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
        v = twist->linear.x;
        w = twist->angular.z;
        ROS_INFO("v: %lf w: %lf", v, w);
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

    void p_controller(int * pwm, double alpha, double w, double encoder_delta) {
        *pwm += alpha * (w - estimate_w(encoder_delta));

        if(*pwm > params.pwm_limit_high) {
            *pwm = params.pwm_limit_high;
        } else if(*pwm < params.pwm_limit_low) {
            *pwm = params.pwm_limit_low;
        }
    }

    void publish_pwm(int left, int right) {
        ras_arduino_msgs::PWM pwm_message;
        pwm_message.PWM1 = right;
        pwm_message.PWM2 = left;

        pwm_publisher.publish(pwm_message);

        ROS_INFO("left: %d right: %d", left, right);
    }

    void set_default_params() {
        set_default_param(PARAM_NAME_LEFT_ALPHA, wheel_left.alpha, PARAM_DEFAULT_LEFT_ALPHA);
        set_default_param(PARAM_NAME_RIGHT_ALPHA, wheel_right.alpha, PARAM_DEFAULT_RIGHT_ALPHA);
        set_default_param(PARAM_NAME_ROBOT_BASE, params.robot_base, PARAM_DEFAULT_ROBOT_BASE);
        set_default_param(PARAM_NAME_WHEEL_RADIUS, params.wheel_radius, PARAM_DEFAULT_WHEEL_RADIUS);
        set_default_param(PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev, PARAM_DEFAULT_TICKS_PER_REV);
        set_default_param(PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high, PARAM_DEFAULT_PWM_LIMIT_HIGH);
        set_default_param(PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low, PARAM_DEFAULT_PWM_LIMIT_LOW);
    }

    template<class T>
    void set_default_param(const std::string & name, T & destination, T default_value) {
        if(!n.hasParam(name)) {
                n.setParam(name, default_value);
            }

        if(!n.getParam(name, destination)) {
            ROS_WARN("Failed to get parameter %s from param server. Falling back to default value.", name.c_str());
        }
    }

    void update_params() {
        n.getParam(PARAM_NAME_LEFT_ALPHA, wheel_left.alpha);
        n.getParam(PARAM_NAME_RIGHT_ALPHA, wheel_right.alpha);
        n.getParam(PARAM_NAME_ROBOT_BASE, params.robot_base);
        n.getParam(PARAM_NAME_WHEEL_RADIUS, params.wheel_radius);
        n.getParam(PARAM_NAME_TICKS_PER_REV, params.ticks_per_rev);
        n.getParam(PARAM_NAME_PWM_LIMIT_HIGH, params.pwm_limit_high);
        n.getParam(PARAM_NAME_PWM_LIMIT_LOW, params.pwm_limit_low);
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
