#include <math.h>

#include <ros/ros.h>
#include <ras_arduino_msgs/PWM.h>
#include <geometry_msgs/Twist.h>
#include <ras_arduino_msgs/Encoders.h>

#define DEFAULT_WHEEL_RADIUS	0.05
#define DEFAULT_ROBOT_BASE		0.225
#define DEFAULT_TICKS_PER_REV	360

#define HZ 10
#define BUFFER_SIZE 1000

class MotorController {
private:
	struct params {
		double alpha;
		double i;
		double d;
	};

	ros::NodeHandle n;

	params params_left;
	params params_right;
	double v;
	double w;
	double radius;
	double base;
	int ticks_per_rev;

	int left_encoder_delta;
	int right_encoder_delta;
	int left_pwm;
	int right_pwm;

	ros::Publisher pwm_publisher;
	ros::Subscriber twist_subscriber;
	ros::Subscriber encoders_subscriber;
	
public:
	MotorController(double radius_default, double base_default, int ticks_per_rev_default) : v(0), w(0), left_encoder_delta(0), right_encoder_delta(0), left_pwm(0), right_pwm(0) {
		n = ros::NodeHandle("~");
		n.param("/radius", radius, radius_default);
		n.param("/base", base, base_default);
		n.param("/ticks_per_rev", ticks_per_rev, ticks_per_rev_default);

		pwm_publisher = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm", BUFFER_SIZE);
		twist_subscriber = n.subscribe<geometry_msgs::Twist>("/s8/twist", BUFFER_SIZE, &MotorController::twist_callback, this);
		encoders_subscriber = n.subscribe<ras_arduino_msgs::Encoders>("/arduino/encoders", BUFFER_SIZE, &MotorController::encoders_callback, this);
	}

	void update() {
		updateParams();

		double left_w;
		double right_w;
		mps_to_rps(v, w, &left_w, &right_w);

		p_controller(&left_pwm, params_left.alpha, left_w, left_encoder_delta);
		p_controller(&right_pwm, params_right.alpha, right_w, right_encoder_delta);

		publish_pwm(left_pwm, right_pwm);
	}

private:
	void updateParams() {
		n.getParam("/left_alpha", params_left.alpha);
		n.getParam("/left_i", params_left.i);
		n.getParam("/left_d", params_left.d);

		n.getParam("/right_alpha", params_right.alpha);
		n.getParam("/right_i", params_right.i);
		n.getParam("/right_d", params_right.d);

		n.getParam("/base", base);
		n.getParam("/radius", radius);
	}

	void twist_callback(const geometry_msgs::Twist::ConstPtr & twist) {
		v = twist->linear.x;
		w = twist->angular.z;
	}

	void encoders_callback(const ras_arduino_msgs::Encoders::ConstPtr & encoders) {
		left_encoder_delta = encoders->delta_encoder2;
		right_encoder_delta = encoders->delta_encoder1;
	}

	void mps_to_rps(double v, double w, double *left_w, double *right_w) {
		*left_w = (v - (base / 2) * w) / radius;
		*right_w = (v + (base / 2) * w) / radius;
	}

	double estimate_w(double encoder_delta) {
		return (encoder_delta * 2 * M_PI * HZ) / ticks_per_rev;
	}

	void p_controller(int * pwm, double alpha, double w, double encoder_delta) {
		*pwm += alpha * (w - estimate_w(encoder_delta));

		if(*pwm > 255) {
			*pwm = 255;
		} else if(*pwm < -255) {
			*pwm = -255;
		}
	}

	void publish_pwm(int left, int right) {
		ras_arduino_msgs::PWM pwm_message;
		pwm_message.PWM1 = right;
		pwm_message.PWM2 = left;

		pwm_publisher.publish(pwm_message);

		ROS_INFO("left: %d right: %d", left, right);
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "s8_motor_controller");

	MotorController motor_controller(DEFAULT_WHEEL_RADIUS, DEFAULT_ROBOT_BASE, DEFAULT_TICKS_PER_REV);
	ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        motor_controller.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}
