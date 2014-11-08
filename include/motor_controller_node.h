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

        /**
         * Positive angular velocity (w) will make the robot spin counter-clockwise (left).
         * Negative angular velocity (w) will make the robot spin clockwise (right).
         * 0 angular velocity will not make the robot spin in any direction (it will always point forward).
         */
        enum RotationDirection {
            LEFT = 1,
            FORWARD = 0,
            RIGHT = -1
        };

        /**
         * Converts the RotationDirection to a string describing the rotation direction.
         */
        std::string to_string(RotationDirection rotation_direction) {
            switch(rotation_direction) {
                case RotationDirection::LEFT: return "LEFT";
                case RotationDirection::FORWARD: return "FORWARD";
                case RotationDirection::RIGHT: return "RIGHT";
            }

            return "UNKNOWN";
        }
    }
}

#endif
