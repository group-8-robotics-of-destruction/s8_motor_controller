#ifndef __MOTOR_CONTROLLER_NODE_H
#define __MOTOR_CONTROLLER_NODE_H

#include <map>
#include <string>

namespace s8 {
    namespace motor_controller_node {
        const char *        NODE_NAME =                 "s8_motor_controller";
        const char *        TOPIC_PWM =                 "/arduino/pwm";
        const char *        TOPIC_ENCODERS =            "/arduino/encoders";
        const char *        TOPIC_TWIST =               "/s8/twist";
        const char *        TOPIC_ACTUAL_TWIST =        "/s8/actual_twist";

        const char *        ACTION_STOP =               "stop";
    }
}

#endif
